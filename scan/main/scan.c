/* CSI over Serial - Semi-Radar / Dance-Pose
   Sterowany z Pythona przez UART - konfiguracja, pobieranie danych, rekonfiguracja.

   Protokół (linie tekstowe, \n):
   - ESP startuje, drukuje "READY\n" i czeka na konfigurację
   - Python wysyła: CONFIG|ssid|pass|queue_len|buf_max|mode|interval_ms|channel|channel_width|request_count
  - Tryb pracy: mode=0 PING, mode=1 NULL_FRAME, mode=2 UDP_RX (odbiór UDP od AP)
   - channel_width: 0=HT20, 1=HT40below, 2=HT40above
   - request_count: ile próbek wysłać przy GET_DATA

   Komendy w trakcie pracy:
   - GET_DATA - ESP wysyła request_count ostatnich próbek (format CSV jak wcześniej)
   - RECONFIG|ssid|pass|... - nowa konfiguracja, restart usług

   Baud: 921600 (ustaw w sdkconfig)
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "ping/ping_sock.h"
#include "esp_heap_caps.h"
#include "esp_wifi_types.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG_ENABLED
#include "driver/usb_serial_jtag.h"
#endif

static const char *TAG = "csi";

/* USB Serial/JTAG RX wymaga zainstalowanego drivera. W niektórych konfiguracjach
   (np. gdy USB Serial/JTAG jest secondary console) logi idą na USB, ale driver RX
   może nie być zainstalowany. */
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG_ENABLED
static bool s_usb_jtag_driver_ready = false;

static void proto_init(void)
{
    if (usb_serial_jtag_is_driver_installed()) {
        s_usb_jtag_driver_ready = true;
        return;
    }
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err == ESP_OK) {
        s_usb_jtag_driver_ready = true;
        ESP_LOGI(TAG, "USB Serial/JTAG driver zainstalowany (RX/TX)");
    } else {
        s_usb_jtag_driver_ready = false;
        ESP_LOGW(TAG, "Nie udało się zainstalować USB Serial/JTAG driver (err=%d) - komendy tylko przez UART stdin", (int)err);
    }
}
#else
static void proto_init(void) {}
#endif

/* Odczyt znaków komend hosta.
   UWAGA: w sdkconfig ten projekt ma:
   - primary console: UART0
   - secondary: USB Serial/JTAG
   W praktyce logi mogą iść na COM (USB), ale stdin (VFS) może czytać tylko z UART.
   Dlatego próbujemy czytać z USB Serial/JTAG (jeśli włączone) ORAZ ze stdin. */
static int proto_read_char(char *c, int timeout_ms)
{
    int elapsed = 0;
    while (elapsed < timeout_ms) {
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG_ENABLED
        if (s_usb_jtag_driver_ready) {
            int n_usb = usb_serial_jtag_read_bytes((uint8_t *)c, 1, pdMS_TO_TICKS(10));
            if (n_usb == 1) return 1;
        }
#endif
        int n = read(STDIN_FILENO, c, 1);
        if (n == 1) return 1;
        if (n < 0 && errno != EAGAIN) return -1;
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed += 10;
    }
    return -1;
}

#define CSI_BUF_MAX_DEFAULT  256
#define CSI_BUF_MAX_LIMIT    512
#define CSI_QUEUE_LEN_DEFAULT 24
#define UART_BUF_SIZE        1024
#define CMD_LINE_MAX         512
#define RING_BUF_MAX_SAMPLES 200   /* max próbek w buforze pierścieniowym */

#define CSI_MODE_PING       0
#define CSI_MODE_NULL_FRAME 1
#define CSI_MODE_UDP_RX     2

#define UDP_RX_PORT         3333

static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *s_sta_netif = NULL;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static uint8_t s_ap_bssid[6];
static bool s_ap_bssid_valid = false;

typedef struct {
    uint32_t sample_id; /* monotoniczny ID próbki (do wykrywania braków) */
    wifi_pkt_rx_ctrl_t rx_ctrl;
    uint16_t len;
    bool first_word_invalid;
    int8_t buf[CSI_BUF_MAX_LIMIT];
} csi_queue_item_t;

typedef struct {
    char ssid[33];
    char pass[65];
    uint16_t csi_queue_len;
    uint16_t csi_buf_max;
    uint8_t mode;           /* 0=PING, 1=NULL_FRAME */
    uint16_t interval_ms;
    uint8_t channel;
    uint8_t channel_width;  /* 0=HT20, 1=HT40below, 2=HT40above */
    uint16_t request_count;
} csi_config_t;

static csi_config_t s_config;
static QueueHandle_t s_csi_queue = NULL;
static uint32_t s_csi_seq = 0;
static volatile bool s_reconfig_requested = false;
static volatile bool s_get_data_requested = false;
static uint32_t s_sample_id = 0;
static uint32_t s_csi_dropped = 0;
static uint32_t s_csi_dropped_too_long = 0;
static uint32_t s_csi_rx_total = 0;
static uint16_t s_csi_last_len = 0;

#define WIFI_MAX_RETRY 8
static int s_wifi_retry_num = 0;

/* Bufor pierścieniowy - ostatnie request_count próbek */
static csi_queue_item_t *s_ring_buf = NULL;
static uint16_t s_ring_size = 0;
static uint16_t s_ring_head = 0;  /* następna pozycja do zapisu */
static uint16_t s_ring_count = 0; /* ile próbek w buforze */
static portMUX_TYPE s_ring_spinlock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_task_collector = NULL;
static TaskHandle_t s_task_sender = NULL;
static TaskHandle_t s_task_uart = NULL;
static TaskHandle_t s_task_udp = NULL;

static int s_udp_sock = -1;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        s_wifi_retry_num = 0;
        printf("WIFI_CONNECTING\n");
        fflush(stdout);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_ap_bssid_valid = false;
        if (s_wifi_retry_num < WIFI_MAX_RETRY) {
            s_wifi_retry_num++;
            esp_wifi_connect();
        } else {
            printf("WIFI_FAIL\n");
            fflush(stdout);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            memcpy(s_ap_bssid, ap_info.bssid, 6);
            s_ap_bssid_valid = true;
        }
        s_wifi_retry_num = 0;
        printf("WIFI_OK\n");
        fflush(stdout);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void csi_rx_cb(void *ctx, wifi_csi_info_t *data)
{
    if (data == NULL || data->len == 0) {
        return;
    }
    s_csi_rx_total++;
    s_csi_last_len = data->len;
    if (data->len > s_config.csi_buf_max || data->len > CSI_BUF_MAX_LIMIT) {
        s_csi_dropped_too_long++;
        return;
    }

    csi_queue_item_t item;
    memset(&item, 0, sizeof(item));
    item.sample_id = s_sample_id++;
    item.rx_ctrl = data->rx_ctrl;
    item.len = data->len;
    item.first_word_invalid = data->first_word_invalid;
    memcpy(item.buf, data->buf, data->len);

    if (xQueueSend(s_csi_queue, &item, 0) != pdTRUE) {
        /* Kolejka pełna - odrzuć */
        s_csi_dropped++;
    }
}

/* Zapis do bufora pierścieniowego */
static void ring_buf_push(const csi_queue_item_t *item)
{
    portENTER_CRITICAL(&s_ring_spinlock);
    if (s_ring_buf != NULL && s_ring_size > 0) {
        memcpy(&s_ring_buf[s_ring_head], item, sizeof(csi_queue_item_t));
        s_ring_head = (s_ring_head + 1) % s_ring_size;
        if (s_ring_count < s_ring_size) {
            s_ring_count++;
        }
    }
    portEXIT_CRITICAL(&s_ring_spinlock);
}

/* Odczyt N ostatnich próbek - zwraca liczbę skopiowanych */
static uint16_t ring_buf_read_last(csi_queue_item_t *out, uint16_t max_count)
{
    portENTER_CRITICAL(&s_ring_spinlock);
    uint16_t n = (max_count < s_ring_count) ? max_count : s_ring_count;
    if (n == 0 || s_ring_buf == NULL) {
        portEXIT_CRITICAL(&s_ring_spinlock);
        return 0;
    }
    /* Odczytujemy od najstarszej do najnowszej (kolejność chronologiczna) */
    uint16_t start = (s_ring_head + s_ring_size - s_ring_count) % s_ring_size;
    for (uint16_t i = 0; i < n; i++) {
        uint16_t idx = (start + i) % s_ring_size;
        memcpy(&out[i], &s_ring_buf[idx], sizeof(csi_queue_item_t));
    }
    portEXIT_CRITICAL(&s_ring_spinlock);
    return n;
}

/* Task: odbiera z kolejki CSI, zapisuje do bufora pierścieniowego */
static void csi_collector_task(void *pvParameters)
{
    csi_queue_item_t item;
    while (!s_reconfig_requested) {
        if (xQueueReceive(s_csi_queue, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
            ring_buf_push(&item);
        }
    }
    vTaskDelete(NULL);
}

/* Wysyła jedną próbkę w formacie CSV do stdout */
static void send_csi_line(const csi_queue_item_t *item, uint32_t seq)
{
    int8_t rssi = item->rx_ctrl.rssi;
    unsigned ch = item->rx_ctrl.channel;
    uint32_t ts = item->rx_ctrl.timestamp;
    unsigned fwi = item->first_word_invalid ? 1 : 0;
    unsigned sig = item->rx_ctrl.sig_mode;
    unsigned cwb = item->rx_ctrl.cwb;
    unsigned stbc = item->rx_ctrl.stbc;

    /* UWAGA: nie budujemy linii w buforze o stałej długości (1024),
       bo dla len=384/512 byłoby ucinane. Wypisujemy strumieniowo. */
    printf("CSI,%lu,%d,%u,%lu,%u,%u,%u,%u,%u",
           (unsigned long)seq, rssi, ch, (unsigned long)ts, (unsigned)item->len,
           fwi, sig, cwb, stbc);
    for (uint16_t i = 0; i < item->len; i++) {
        printf(",%d", (int)item->buf[i]);
    }
    printf("\n");
}

/* === Tryb PING === */
static void start_ping_to_gateway(void)
{
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(s_sta_netif, &ip_info) != 0) {
        ESP_LOGW(TAG, "Nie można pobrać IP - ping wyłączony");
        return;
    }

    ip_addr_t target;
    memcpy(&target.u_addr.ip4, &ip_info.gw, sizeof(ip_info.gw));
    target.type = IPADDR_TYPE_V4;

    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
    config.target_addr = target;
    config.count = 0;  /* nieskończenie */
    config.interval_ms = s_config.interval_ms;
    config.timeout_ms = 500;

    esp_ping_callbacks_t cbs = {
        .cb_args = NULL,
        .on_ping_success = NULL,
        .on_ping_timeout = NULL,
        .on_ping_end = NULL,
    };

    esp_ping_handle_t ping;
    if (esp_ping_new_session(&config, &cbs, &ping) == ESP_OK) {
        esp_ping_start(ping);
        ESP_LOGI(TAG, "Tryb PING: brama co %u ms", (unsigned)s_config.interval_ms);
    }
}

/* === Tryb NULL FRAME === */
static void null_frame_task(void *pvParameters)
{
    /* Null Data frame (bez QoS): 24B MAC header */
    uint8_t frame[24];
    uint8_t sta_mac[6];
    uint16_t seq = 0;

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1) {
        if (!s_ap_bssid_valid) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        if (esp_wifi_get_mac(WIFI_IF_STA, sta_mac) != ESP_OK) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        memset(frame, 0, sizeof(frame));
        /* Null Data: Frame Control (type=data, subtype=Null) = 0x48 */
        frame[0] = 0x48;
        /* Flags: ToDS=1 (STA->AP). Reszta 0.
           NIE ustawiaj Retry/PowerMgmt/MoreFrag/MoreData. */
        frame[1] = 0x01;
        /* Duration */
        frame[2] = 0x00;
        frame[3] = 0x00;
        /* Addr1 (RA) = BSSID(AP), Addr2 (TA) = STA, Addr3 (DA) = BSSID(AP) */
        memcpy(&frame[4], s_ap_bssid, 6);
        memcpy(&frame[10], sta_mac, 6);
        memcpy(&frame[16], s_ap_bssid, 6);
        /* Sequence control */
        frame[22] = (uint8_t)((seq << 4) & 0xF0);
        frame[23] = (uint8_t)((seq >> 4) & 0xFF);
        seq++;

        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, frame, sizeof(frame), true);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "80211_tx Null failed: %d", (int)err);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(s_config.interval_ms));
    }
}

/* === Tryb UDP_RX: ESP odbiera stały strumień UDP od AP (drugi ESP) === */
static void udp_rx_task(void *pvParameters)
{
    (void)pvParameters;
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_RX_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGW(TAG, "UDP socket() fail: %d", errno);
        vTaskDelete(NULL);
        return;
    }
    s_udp_sock = sock;

    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGW(TAG, "UDP bind() fail: %d", errno);
        close(sock);
        s_udp_sock = -1;
        vTaskDelete(NULL);
        return;
    }

    printf("UDP_READY|%u\n", (unsigned)UDP_RX_PORT);
    fflush(stdout);

    uint8_t buf[512];
    uint32_t pkt = 0;
    uint32_t bytes = 0;
    uint32_t last_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    while (!s_reconfig_requested) {
        int n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        if (n > 0) {
            pkt++;
            bytes += (uint32_t)n;
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (now_ms - last_ms >= 1000) {
            last_ms = now_ms;
            printf("UDP_RX|%u|%u\n", (unsigned)pkt, (unsigned)bytes);
            fflush(stdout);
        }
    }

    close(sock);
    s_udp_sock = -1;
    vTaskDelete(NULL);
}

static bool wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();
    assert(s_sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_config.sta.ssid, s_config.ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, s_config.pass, sizeof(wifi_config.sta.password) - 1);
    /* 0 = skan wszystkich kanałów; >0 = skan tylko tego kanału */
    wifi_config.sta.channel = (uint8_t)s_config.channel;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Łączenie z %s...", s_config.ssid);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        const char *mode_str =
            (s_config.mode == CSI_MODE_PING) ? "PING" :
            (s_config.mode == CSI_MODE_NULL_FRAME) ? "NULL_FRAME" :
            "UDP_RX";
        ESP_LOGI(TAG, "Połączono. Tryb pracy: %s", mode_str);
        /* Opcjonalnie: spróbuj wymusić HT40 po połączeniu. */
        if (s_config.channel_width != 0) {
            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
                if (s_config.channel_width == 1) second = WIFI_SECOND_CHAN_BELOW;
                if (s_config.channel_width == 2) second = WIFI_SECOND_CHAN_ABOVE;
                esp_err_t e1 = esp_wifi_set_channel(ap_info.primary, second);
                wifi_bandwidth_t bw = (second == WIFI_SECOND_CHAN_NONE) ? WIFI_BW_HT20 : WIFI_BW_HT40;
                esp_err_t e2 = esp_wifi_set_bandwidth(WIFI_IF_STA, bw);
                ESP_LOGI(TAG, "BW po połączeniu: primary=%u second=%d bw=%d (set_channel=%d, set_bw=%d)",
                         (unsigned)ap_info.primary, (int)second, (int)bw, (int)e1, (int)e2);
            }
        }
        if (s_config.mode == CSI_MODE_PING) {
            start_ping_to_gateway();
        } else if (s_config.mode == CSI_MODE_NULL_FRAME) {
            xTaskCreate(null_frame_task, "null_frame", 2048, NULL, 5, NULL);
        } else {
            xTaskCreate(udp_rx_task, "udp_rx", 4096, NULL, 5, &s_task_udp);
        }
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Błąd połączenia z %s", s_config.ssid);
        return false;
    }
    return false;
}

static void csi_init(void)
{
#if CONFIG_ESP_WIFI_CSI_ENABLED
    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = true,
        .manu_scale = false,
        .shift = 0,
        .dump_ack_en = (s_config.mode == CSI_MODE_NULL_FRAME),
    };

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
#else
    ESP_LOGE(TAG, "CSI wyłączone! Włącz CONFIG_ESP_WIFI_CSI_ENABLED");
#endif
}

/* Parsuje linię CONFIG|ssid|pass|queue_len|buf_max|mode|interval_ms|channel|channel_width|request_count
   Dokładnie 10 pól rozdzielonych |. Bez spacji na początku. */
static bool parse_config_line(const char *line, csi_config_t *cfg)
{
    char buf[CMD_LINE_MAX];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *tok = strtok(buf, "|");
    if (tok == NULL || strcmp(tok, "CONFIG") != 0) {
        return false;
    }

    if ((tok = strtok(NULL, "|")) == NULL) return false;
    strncpy(cfg->ssid, tok, sizeof(cfg->ssid) - 1);
    cfg->ssid[sizeof(cfg->ssid) - 1] = '\0';

    if ((tok = strtok(NULL, "|")) == NULL) return false;
    strncpy(cfg->pass, tok, sizeof(cfg->pass) - 1);
    cfg->pass[sizeof(cfg->pass) - 1] = '\0';

    if ((tok = strtok(NULL, "|")) == NULL) return false;
    cfg->csi_queue_len = (uint16_t)atoi(tok);
    if (cfg->csi_queue_len < 4) cfg->csi_queue_len = CSI_QUEUE_LEN_DEFAULT;

    if ((tok = strtok(NULL, "|")) == NULL) return false;
    cfg->csi_buf_max = (uint16_t)atoi(tok);
    if (cfg->csi_buf_max < 64 || cfg->csi_buf_max > CSI_BUF_MAX_LIMIT) cfg->csi_buf_max = CSI_BUF_MAX_DEFAULT;

    if ((tok = strtok(NULL, "|")) == NULL) return false;
    cfg->mode = (uint8_t)atoi(tok);
    if (cfg->mode > 2) cfg->mode = CSI_MODE_PING;

    if ((tok = strtok(NULL, "|")) == NULL) return false;
    cfg->interval_ms = (uint16_t)atoi(tok);
    if (cfg->interval_ms < 5) cfg->interval_ms = 20;

    if ((tok = strtok(NULL, "|")) == NULL) return false;
    cfg->channel = (uint8_t)atoi(tok);
    /* channel=0 => AUTO (skan). W STA wymuszenie złego kanału blokuje połączenie. */
    if (cfg->channel > 14) cfg->channel = 0;

    if ((tok = strtok(NULL, "|")) == NULL) return false;
    cfg->channel_width = (uint8_t)atoi(tok);
    if (cfg->channel_width > 2) cfg->channel_width = 0;

    if ((tok = strtok(NULL, "|")) == NULL) return false;
    cfg->request_count = (uint16_t)atoi(tok);
    if (cfg->request_count < 1) cfg->request_count = 50;
    if (cfg->request_count > RING_BUF_MAX_SAMPLES) cfg->request_count = RING_BUF_MAX_SAMPLES;

    return true;
}

/* Parsuje RECONFIG|ssid|pass|... - ten sam format co CONFIG */
static bool parse_reconfig_line(const char *line, csi_config_t *cfg)
{
    char buf[CMD_LINE_MAX];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *tok = strtok(buf, "|");
    if (tok == NULL || strcmp(tok, "RECONFIG") != 0) {
        return false;
    }

    /* Reszta jak CONFIG - sklejamy i parsujemy jako CONFIG */
    size_t off = strlen("RECONFIG") + 1;
    if (off >= strlen(line)) return false;
    char reconf[CMD_LINE_MAX];
    snprintf(reconf, sizeof(reconf), "CONFIG|%s", line + off);
    return parse_config_line(reconf, cfg);
}

/* Task: monitoruje UART, obsługuje GET_DATA i RECONFIG */
static void uart_cmd_task(void *pvParameters)
{
    char line_buf[CMD_LINE_MAX];
    int line_len = 0;

    while (!s_reconfig_requested) {
        char c;
        if (proto_read_char(&c, 10) <= 0) {
            continue;
        }
        if (c == '\n' || c == '\r') {
            if (line_len > 0) {
                line_buf[line_len] = '\0';
                line_len = 0;

                if (strncmp(line_buf, "GET_DATA", 8) == 0) {
                    s_get_data_requested = true;
                    printf("GET_DATA_OK\n");
                    fflush(stdout);
                } else if (strcmp(line_buf, "READY_ACK") == 0) {
                    printf("READY_ACK_OK\n");
                    fflush(stdout);
                } else if (strncmp(line_buf, "RECONFIG", 8) == 0) {
                    csi_config_t new_cfg;
                    if (parse_reconfig_line(line_buf, &new_cfg)) {
                        memcpy(&s_config, &new_cfg, sizeof(s_config));
                        printf("RECONFIG_OK\n");
                        fflush(stdout);
                        s_reconfig_requested = true;
                    } else {
                        printf("RECONFIG_ERR\n");
                        fflush(stdout);
                    }
                }
            }
        } else if (line_len < (int)sizeof(line_buf) - 1) {
            line_buf[line_len++] = (char)c;
        }
    }
    vTaskDelete(NULL);
}

/* Task: reaguje na GET_DATA - wysyła dane z bufora */
static void data_sender_task(void *pvParameters)
{
    csi_queue_item_t *send_buf = heap_caps_malloc(s_config.request_count * sizeof(csi_queue_item_t), MALLOC_CAP_INTERNAL);
    if (send_buf == NULL) {
        ESP_LOGE(TAG, "Brak pamięci na send_buf");
        vTaskDelete(NULL);
        return;
    }

    while (!s_reconfig_requested) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (!s_get_data_requested) continue;
        s_get_data_requested = false;

        /* GET_DATA ma zwrócić request_count ostatnich próbek.
           Bez timeoutu: czekaj aż bufor uzbiera pełne okno. */
        uint16_t need = s_config.request_count;
        uint16_t n = ring_buf_read_last(send_buf, need);
        uint32_t last_wait_log = 0;
        while (n < need && !s_reconfig_requested) {
            uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (now_ms - last_wait_log >= 1000) {
                last_wait_log = now_ms;
                printf("WAIT_SAMPLES|%u|%u\n", (unsigned)n, (unsigned)need);
                printf("CSI_STAT|rx=%lu|drop_q=%lu|drop_len=%lu|last_len=%u\n",
                       (unsigned long)s_csi_rx_total,
                       (unsigned long)s_csi_dropped,
                       (unsigned long)s_csi_dropped_too_long,
                       (unsigned)s_csi_last_len);
                fflush(stdout);
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            n = ring_buf_read_last(send_buf, need);
        }
        printf("DATA_BEGIN|%u\n", (unsigned)n);
        fflush(stdout);
        for (uint16_t i = 0; i < n; i++) {
            send_csi_line(&send_buf[i], send_buf[i].sample_id);
            /* Wysyłanie długich linii CSI może zająć długo – oddaj CPU, żeby nie wywołać task_wdt. */
            if ((i & 3u) == 3u) {
                fflush(stdout);
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        fflush(stdout);
        printf("DATA_END|%u\n", (unsigned)n);
        fflush(stdout);
    }
    free(send_buf);
    vTaskDelete(NULL);
}

#define CONFIG_WAIT_MS        5000   /* Czekaj max 5s na CONFIG, potem użyj domyślnej konfiguracji */
#define READY_ACK_WARN_MS     1500   /* Po tylu ms bez READY_ACK wypisz ostrzeżenie (bez blokowania) */

static bool wait_for_config(void)
{
    char line_buf[CMD_LINE_MAX];
    int line_len = 0;
    /* Wymagany handshake + konfiguracja.
       Zgodnie z wymaganiami hosta: nie przechodzimy dalej bez informacji zwrotnej. */
    while (1) {
        /* 1) READY + wymagany READY_ACK */
        printf("READY\n");
        fflush(stdout);

        ESP_LOGI(TAG, "Czekam na READY_ACK od hosta...");
        int ack_wait_ms = 0;
        line_len = 0;
        while (ack_wait_ms < 5000) {
            char c;
            if (proto_read_char(&c, 100) <= 0) {
                ack_wait_ms += 100;
                continue;
            }
            if (c == '\n' || c == '\r') {
                if (line_len > 0) {
                    line_buf[line_len] = '\0';
                    line_len = 0;
                    if (strcmp(line_buf, "READY_ACK") == 0) {
                        printf("READY_ACK_OK\n");
                        fflush(stdout);
                        break;
                    }
                }
            } else if (line_len < (int)sizeof(line_buf) - 1) {
                line_buf[line_len++] = (char)c;
            }
        }
        if (ack_wait_ms >= 5000) {
            ESP_LOGW(TAG, "Brak READY_ACK - ponawiam READY");
            continue;
        }

        /* 2) Wymagany CONFIG */
        ESP_LOGI(TAG, "Czekam na CONFIG|... (bez timeoutu)");
        line_len = 0;
        while (1) {
            char c;
            if (proto_read_char(&c, 200) <= 0) {
                continue;
            }
            if (c == '\n' || c == '\r') {
                if (line_len > 0) {
                    line_buf[line_len] = '\0';
                    line_len = 0;
                    if (strcmp(line_buf, "READY_ACK") == 0) {
                        printf("READY_ACK_OK\n");
                        fflush(stdout);
                        continue;
                    }
                    if (strncmp(line_buf, "CONFIG", 6) == 0) {
                        if (parse_config_line(line_buf, &s_config)) {
                            printf("CONFIG_OK\n");
                            fflush(stdout);
                            return true;
                        }
                        printf("CONFIG_ERR\n");
                        fflush(stdout);
                        ESP_LOGW(TAG, "Parse CONFIG nieudany");
                    } else if (strncmp(line_buf, "RECONFIG", 8) == 0) {
                        if (parse_reconfig_line(line_buf, &s_config)) {
                            printf("CONFIG_OK\n");
                            fflush(stdout);
                            return true;
                        }
                        printf("CONFIG_ERR\n");
                        fflush(stdout);
                        ESP_LOGW(TAG, "Parse RECONFIG nieudany");
                    }
                }
            } else if (line_len < (int)sizeof(line_buf) - 1) {
                line_buf[line_len++] = (char)c;
            }
        }
    }
}

static void stop_wifi_and_cleanup(void)
{
    if (s_udp_sock >= 0) {
        close(s_udp_sock);
        s_udp_sock = -1;
    }
    esp_wifi_stop();
    esp_wifi_deinit();
    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }
    if (s_csi_queue) {
        vQueueDelete(s_csi_queue);
        s_csi_queue = NULL;
    }
    if (s_ring_buf) {
        free(s_ring_buf);
        s_ring_buf = NULL;
    }
    s_ring_size = 0;
    s_ring_count = 0;
    s_ring_head = 0;
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Inicjalizacja wejścia protokołu (USB Serial/JTAG RX jeśli dostępne) */
    proto_init();

    /* Domyślna konfiguracja */
    memset(&s_config, 0, sizeof(s_config));
    strncpy(s_config.ssid, "Domek_na_prerii_2.4GHz", sizeof(s_config.ssid) - 1);
    strncpy(s_config.pass, "Jakub12345678!", sizeof(s_config.pass) - 1);
    s_config.csi_queue_len = CSI_QUEUE_LEN_DEFAULT;
    s_config.csi_buf_max = CSI_BUF_MAX_DEFAULT;
    s_config.mode = CSI_MODE_PING;
    s_config.interval_ms = 20;
    s_config.channel = 1;
    s_config.channel_width = 0;
    s_config.request_count = 50;

    while (1) {
        if (!wait_for_config()) {
            continue;
        }

        s_csi_queue = xQueueCreate(s_config.csi_queue_len, sizeof(csi_queue_item_t));
        if (s_csi_queue == NULL) {
            ESP_LOGE(TAG, "Błąd tworzenia kolejki CSI");
            continue;
        }

        s_ring_size = s_config.request_count;
        s_ring_buf = heap_caps_malloc(s_ring_size * sizeof(csi_queue_item_t), MALLOC_CAP_INTERNAL);
        if (s_ring_buf == NULL) {
            ESP_LOGE(TAG, "Brak pamięci na bufor pierścieniowy");
            vQueueDelete(s_csi_queue);
            s_csi_queue = NULL;
            continue;
        }
        s_ring_head = 0;
        s_ring_count = 0;
        s_csi_seq = 0;
        s_sample_id = 0;
        s_csi_dropped = 0;
        s_csi_dropped_too_long = 0;
        s_csi_rx_total = 0;
        s_csi_last_len = 0;
        s_reconfig_requested = false;
        s_get_data_requested = false;

        /* Nie uruchamiaj CSI/workerów jeśli nie ma WIFI_OK */
        if (!wifi_init_sta()) {
            ESP_LOGW(TAG, "WiFi niepołączone - wracam do oczekiwania na konfigurację");
            stop_wifi_and_cleanup();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        csi_init();

        xTaskCreate(csi_collector_task, "csi_collect", 4096, NULL, 5, &s_task_collector);
        xTaskCreate(data_sender_task, "data_sender", 4096, NULL, 5, &s_task_sender);
        xTaskCreate(uart_cmd_task, "uart_cmd", 2048, NULL, 5, &s_task_uart);

        /* Czekaj na RECONFIG - wtedy zatrzymujemy i wracamy do pętli głównej */
        while (!s_reconfig_requested) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        ESP_LOGI(TAG, "Rekonfiguracja - zatrzymuję usługi...");
        vTaskDelay(pdMS_TO_TICKS(200));  /* Czas na zakończenie zadań */
        if (s_task_collector) vTaskDelete(s_task_collector);
        if (s_task_sender) vTaskDelete(s_task_sender);
        if (s_task_uart) vTaskDelete(s_task_uart);
        if (s_task_udp) vTaskDelete(s_task_udp);
        s_task_collector = s_task_sender = s_task_uart = s_task_udp = NULL;
        stop_wifi_and_cleanup();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
