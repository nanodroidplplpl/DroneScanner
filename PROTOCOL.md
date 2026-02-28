# Protokół UART Python ↔ ESP32 CSI

## Ogólne zasady

- **Kodowanie**: UTF-8 (dla tekstu) / raw bytes (dla CSI)
- **Końcówka linii**: `\n` (LF, 0x0A)
- **Baud**: 115200
- **Port**: `\\.\COMx` na Windows (np. `\\.\COM4`)

---

## Sekwencja startowa

1. ESP32 startuje, wypisuje boot logi.
2. ESP32 wysyła linię: `READY\n`
3. (Test łącza TX) Python odsyła: `READY_ACK\n`, ESP odpowiada: `READY_ACK_OK\n`
4. Python wysyła konfigurację.

---

## Komendy Python → ESP32

### CONFIG

Format jednej linii:

```
CONFIG|<ssid>|<pass>|<queue_len>|<buf_max>|<mode>|<interval_ms>|<channel>|<channel_width>|<request_count>\n
```

Przykład:

```
CONFIG|Domek_na_prerii_2.4GHz|Jakub12345678!|24|256|0|20|1|0|50\n
```

| Pole           | Typ   | Opis                                      |
|----------------|-------|-------------------------------------------|
| ssid           | str   | Nazwa sieci WiFi                          |
| pass           | str   | Hasło WiFi                                |
| queue_len      | int   | Długość kolejki CSI (np. 24)              |
| buf_max        | int   | Maks. rozmiar bufora (np. 256)            |
| mode           | int   | Tryb (0=normalny)                         |
| interval_ms    | int   | Interwał wysyłania CSI w ms (np. 20)     |
| channel        | int   | Kanał WiFi (np. 1)                        |
| channel_width  | int   | Szerokość kanału (0=20MHz, 1=40MHz)       |
| request_count  | int   | Ile razy wysłać CSI (0=bez limitu)        |

ESP32 po poprawnym parsowaniu odpowiada: `CONFIG_OK\n`

### RECONFIG

Format identyczny jak CONFIG, z prefiksem `RECONFIG` zamiast `CONFIG`:

```
RECONFIG|<ssid>|<pass>|<queue_len>|<buf_max>|<mode>|<interval_ms>|<channel>|<channel_width>|<request_count>\n
```

ESP32 odpowiada: `CONFIG_OK\n`

### GET_DATA

Format:

```
GET_DATA\n
```

ESP32 po otrzymaniu wysyła dane CSI (format CSI poniżej).

---

## Odpowiedzi ESP32 → Python

### READY_ACK_OK

```
READY_ACK_OK\n
```

Odpowiedź na `READY_ACK\n` (handshake testowy: potwierdza, że ESP odebrał dane z hosta).

### WIFI_CONNECTING / WIFI_OK / WIFI_FAIL

```
WIFI_CONNECTING\n
WIFI_OK\n
WIFI_FAIL\n
```

Status połączenia WiFi. Host **nie powinien** przechodzić do pomiarów/GET_DATA bez `WIFI_OK`.

### CONFIG_OK

```
CONFIG_OK\n
```

Wysyłane po poprawnym parsowaniu linii CONFIG.

### CONFIG_ERR / RECONFIG_ERR

```
CONFIG_ERR\n
RECONFIG_ERR\n
```

Wysyłane, gdy ESP nie potrafi sparsować konfiguracji (zły format/pola).

### GET_DATA_OK

```
GET_DATA_OK\n
```

Potwierdza odebranie komendy `GET_DATA`.

### UDP_READY / UDP_RX (tryb mode=2 UDP_RX)

```
UDP_READY|3333\n
UDP_RX|<pkts>|<bytes>\n
```

W trybie `mode=2` ESP uruchamia odbiór UDP na porcie 3333 (to generuje regularne ramki i pomaga w gęstym CSI).

### Logi tekstowe

ESP32 wypisuje logi w formacie ESP-IDF (np. `I (123) tag: message`).  
Python może je ignorować lub logować.

### Dane CSI (format CSV)

ESP32 wysyła próbki CSI jako linie tekstowe CSV:

```
CSI,<seq>,<rssi>,<ch>,<ts>,<len>,<fwi>,<sig>,<cwb>,<stbc>,<v0>,<v1>,...,<vN>\n
```

| Pole   | Opis                          |
|--------|--------------------------------|
| seq    | ID próbki (`sample_id`, monotoniczny) |
| rssi   | RSSI (dBm)                    |
| ch     | Kanał WiFi                    |
| ts     | Timestamp                     |
| len    | Liczba wartości CSI           |
| fwi    | first_word_invalid (0/1)      |
| sig    | sig_mode                      |
| cwb    | cwb                           |
| stbc   | stbc                          |
| v0..vN | Wartości CSI (int)            |

Po wszystkich próbkach ESP32 wysyła:

```
DATA_BEGIN|<n>\n
DATA_END|<n>\n
```

gdzie `<n>` to liczba wysłanych próbek.

---

## Uwagi implementacyjne

- Python na Windows może blokować się przy `ser.write()` – stosowany jest zapis w pętli z `write_timeout=0`.
- ESP32 może mieć timeout na CONFIG (np. 5 s) – po nim używa domyślnej konfiguracji.
- W trybie `read_only` Python nie wysyła CONFIG ani GET_DATA; ESP32 używa domyślnej konfiguracji i wysyła CSI automatycznie co N sekund.
