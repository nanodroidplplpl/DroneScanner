#!/usr/bin/env python3
"""
Kontroler ESP32 CSI - sterowanie konfiguracją i pobieranie danych.
"""

# ============== KONFIGURACJA - zmień tutaj ==============
PORT = "COM4"
BAUD = 115200

# Na Windows idf_monitor używa \\.\COM4 zamiast COM4 (lepsza kompatybilność z odczytem)
def _port_str(port: str) -> str:
    import sys
    if sys.platform == "win32" and port.upper().startswith("COM"):
        return r"\\.\COM" + port[3:]  # COM4 -> \\.\COM4
    return port

# WiFi

WIFI_SSID = "CSI_AP"
WIFI_PASSWORD = "12345678"
# CSI
CSI_MODE = 2              # 0=PING, 1=NULL_FRAME, 2=UDP_RX (drugi ESP jako AP wysyła UDP do tego STA)
CSI_INTERVAL_MS = 20
CSI_QUEUE_LEN = 100
CSI_BUF_MAX = 512
CSI_CHANNEL = 0            # 0 = AUTO (skan). Ustaw na konkretny kanał tylko jeśli znasz kanał AP.
CSI_CHANNEL_WIDTH = 2     # 0=HT20, 1=HT40below, 2=HT40above - to jest szerokosc kanalow
CSI_REQUEST_COUNT = 80

# Po starcie (po WIFI_OK) poczekaj, żeby ESP zebrało próbki
FIRST_GET_DATA_DELAY_SEC = 15.0

# Tryb działania: "get_data" | "poll" | "config_only"
RUN_MODE = "get_data"
SAVE_CSI_FILE = "dane_csi.jsonl"   # TYLKO dane CSI (JSONL)
SAVE_INFO_FILE = "info.jsonl"      # wszystko inne: logi/zdarzenia/status (JSONL)
POLL_INTERVAL_SEC = 5.0   # przy poll
# ========================================================

import sys
import json
import os
import time
import threading
from dataclasses import dataclass
from typing import Optional, Callable, TextIO, Any, Dict

def _log(msg: str) -> None:
    """Wypisz na stderr (niebuforowane) - zawsze widoczne."""
    sys.stderr.write(msg + "\n")
    sys.stderr.flush()

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # type: ignore


class JsonlLogger:
    """Logger: zapisuje WSZYSTKO odebrane z ESP do JSONL (1 JSON na linię)."""

    def __init__(self, path: str, meta: Optional[Dict[str, Any]] = None):
        self.path = path
        self._meta = meta
        self._fh: Optional[TextIO] = None

    def open(self) -> None:
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        is_new = (not os.path.exists(self.path)) or os.path.getsize(self.path) == 0
        self._fh = open(self.path, "a", encoding="utf-8", newline="\n")
        if is_new:
            if self._meta is not None:
                self.write(self._meta)
            else:
                self.write(
                    {
                        "type": "meta",
                        "schema_version": 1,
                        "description": "Log UART/USB Serial/JTAG ESP32 CSI. Każda linia = 1 JSON.",
                        "fields": {
                            "type": "meta|event|line|csi",
                            "t_host_epoch": "czas hosta (time.time())",
                            "t_host_mono": "czas hosta (time.monotonic())",
                            "phase": "np. boot|handshake|config|wifi|get_data|poll|reconfig",
                            "line": "surowa linia tekstowa odebrana z ESP (bez CR/LF)",
                            "parsed": "opcjonalnie: zparsowane dane (np. CSI)",
                        },
                        "csi_format": {
                            "line_prefix": "CSI",
                            "fields": [
                                "seq(sample_id)",
                                "rssi",
                                "channel",
                                "esp_timestamp",
                                "len",
                                "first_word_invalid",
                                "sig_mode",
                                "cwb",
                                "stbc",
                                "v0..vN (int)",
                            ],
                        },
                    }
                )

    def close(self) -> None:
        if self._fh:
            self._fh.flush()
            self._fh.close()
            self._fh = None

    def write(self, obj: Dict[str, Any]) -> None:
        if not self._fh:
            self.open()
        assert self._fh is not None
        self._fh.write(json.dumps(obj, ensure_ascii=False) + "\n")
        self._fh.flush()

    def log_line(self, phase: str, line: str, parsed: Optional[Dict[str, Any]] = None) -> None:
        evt: Dict[str, Any] = {
            "type": "line",
            "t_host_epoch": time.time(),
            "t_host_mono": time.monotonic(),
            "phase": phase,
            "line": line,
        }
        if parsed is not None:
            evt["parsed"] = parsed
        self.write(evt)


class JsonlSplitLogger:
    """Logger rozdzielony: CSI do jednego pliku, reszta do drugiego."""

    def __init__(self, csi_path: str, info_path: str):
        self.csi_path = csi_path
        self.info_path = info_path
        self._csi = JsonlLogger(csi_path, meta=self._csi_meta())
        self._info = JsonlLogger(info_path, meta=self._info_meta())

    def _info_meta(self) -> Dict[str, Any]:
        return {
            "type": "meta",
            "schema_version": 1,
            "description": "INFO: logi/status/zdarzenia z ESP + host. Bez rekordów CSI.",
            "fields": {
                "type": "meta|event|line",
                "t_host_epoch": "czas hosta (time.time())",
                "t_host_mono": "czas hosta (time.monotonic())",
                "phase": "np. boot|handshake|config|wifi|get_data|poll|reconfig|stop",
                "line": "surowa linia tekstowa odebrana z ESP (bez CR/LF)",
            },
        }

    def _csi_meta(self) -> Dict[str, Any]:
        return {
            "type": "meta",
            "schema_version": 1,
            "description": "DATA: tylko próbki CSI (zparsowane).",
            "fields": {
                "type": "meta|csi",
                "t_host_epoch": "czas hosta (time.time())",
                "t_host_mono": "czas hosta (time.monotonic())",
                "phase": "zwykle 'csi'",
                "seq": "sample_id z ESP (monotoniczny)",
                "rssi": "RSSI (dBm)",
                "channel": "kanał",
                "timestamp": "timestamp z ESP",
                "len": "liczba wartości w 'csi'",
                "first_word_invalid": "bool",
                "sig_mode": "int",
                "cwb": "int (channel width flag z CSI)",
                "stbc": "int",
                "csi": "lista int (I/Q naprzemiennie, jak w firmware)",
            },
        }

    def open(self) -> None:
        self._csi.open()
        self._info.open()

    def close(self) -> None:
        self._csi.close()
        self._info.close()

    def write(self, obj: Dict[str, Any]) -> None:
        """Kompatybilność: zdarzenia/system -> do INFO."""
        self._info.write(obj)

    def log_line(self, phase: str, line: str, parsed: Optional[Dict[str, Any]] = None) -> None:
        # CSI trafia wyłącznie do pliku DATA (bez duplikowania w INFO).
        is_csi = phase == "csi" or (parsed is not None and "csi" in parsed)
        if is_csi and parsed is not None:
            evt: Dict[str, Any] = {
                "type": "csi",
                "t_host_epoch": time.time(),
                "t_host_mono": time.monotonic(),
                "phase": phase,
            }
            evt.update(parsed)
            self._csi.write(evt)
            return
        # Surowe linie "CSI,..." pojawiają się najpierw jako phase="get_data" (parsed=None),
        # a dopiero potem jako phase="csi" (parsed=dict). Nie zapisujemy ich do INFO.
        if line.startswith("CSI,"):
            return
        self._info.log_line(phase, line, parsed=None)


@dataclass
class CsiConfig:
    """Konfiguracja pracy ESP32 CSI."""
    ssid: str = ""
    password: str = ""
    csi_queue_len: int = 24
    csi_buf_max: int = 256
    mode: int = 0  # 0=PING, 1=NULL_FRAME
    interval_ms: int = 20
    channel: int = 1
    channel_width: int = 0  # 0=HT20, 1=HT40below, 2=HT40above
    request_count: int = 50

    def to_config_line(self) -> str:
        """Format: CONFIG|ssid|pass|queue_len|buf_max|mode|interval_ms|channel|channel_width|request_count
        Zgodny z scan.c parse_config_line. Końcówka: \\n (protokół scan.c)."""
        return (
            f"CONFIG|{self.ssid}|{self.password}|{self.csi_queue_len}|"
            f"{self.csi_buf_max}|{self.mode}|{self.interval_ms}|"
            f"{self.channel}|{self.channel_width}|{self.request_count}\n"
        )

    def to_reconfig_line(self) -> str:
        """Format linii RECONFIG do wysłania."""
        return (
            f"RECONFIG|{self.ssid}|{self.password}|{self.csi_queue_len}|"
            f"{self.csi_buf_max}|{self.mode}|{self.interval_ms}|"
            f"{self.channel}|{self.channel_width}|{self.request_count}\n"
        )


class EspSendConf:
    """Wysyłanie konfiguracji do ESP32."""

    def __init__(
        self,
        port: str,
        baud: int = 3000000,
        timeout: float = 0.1,
        on_line: Optional[Callable[[str, str, Optional[Dict[str, Any]]], None]] = None,
    ):
        if serial is None:
            raise ImportError("Zainstaluj: pip install pyserial")
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._on_line = on_line

    def open(self) -> None:
        """Otwórz port szeregowy."""
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

    def close(self) -> None:
        """Zamknij port."""
        if self._ser and self._ser.is_open:
            self._ser.close()
            self._ser = None

    def send_config(self, config: CsiConfig) -> bool:
        """
        Wyślij konfigurację do ESP32.
        Zwraca True jeśli wysłano poprawnie.
        """
        if not self._ser or not self._ser.is_open:
            self.open()
        line = config.to_config_line()
        data = line.encode("utf-8")
        _log(f"  [SEND] %d bajtów: %r" % (len(data), data[:80]))
        # NIE ustawiaj RTS/DTR - może blokować TX na niektórych USB-UART
        # Tryb nieblokujący: write_timeout=0, retry przy braku miejsca w buforze
        old_wt = self._ser.write_timeout
        self._ser.write_timeout = 0
        try:
            pos = 0
            deadline = time.monotonic() + 30.0
            no_progress = 0
            while pos < len(data) and time.monotonic() < deadline:
                n = self._ser.write(data[pos:])
                if n > 0:
                    pos += n
                    no_progress = 0
                else:
                    no_progress += 1
                    if no_progress > 500:  # ~10s bez postępu
                        break
                time.sleep(0.02)
            if pos < len(data):
                _log(f"  [UWAGA] Wysłano tylko {pos}/{len(data)} bajtów (timeout)")
        finally:
            self._ser.write_timeout = old_wt
        self._ser.flush()
        return True

    def send_reconfig(self, config: CsiConfig) -> bool:
        """Wyślij RECONFIG (zmiana konfiguracji w trakcie pracy)."""
        if not self._ser or not self._ser.is_open:
            self.open()
        line = config.to_reconfig_line()
        data = line.encode("utf-8")
        _log(f"  [SEND] %d bajtów: %r" % (len(data), data[:80]))
        old_wt = self._ser.write_timeout
        self._ser.write_timeout = 0
        try:
            pos = 0
            deadline = time.monotonic() + 30.0
            no_progress = 0
            while pos < len(data) and time.monotonic() < deadline:
                n = self._ser.write(data[pos:])
                if n > 0:
                    pos += n
                    no_progress = 0
                else:
                    no_progress += 1
                    if no_progress > 500:
                        break
                time.sleep(0.02)
            if pos < len(data):
                _log(f"  [UWAGA] Wysłano tylko {pos}/{len(data)} bajtów (timeout)")
        finally:
            self._ser.write_timeout = old_wt
        self._ser.flush()
        return True

    def wait_ready(self) -> bool:
        """Czekaj bez timeoutu na linię READY od ESP32."""
        if not self._ser:
            return False
        line_buf = ""
        last_log = time.monotonic()
        while True:
            chunk = self._ser.read(256).decode("utf-8", errors="ignore")
            if chunk:
                _log(f"  [READ] {len(chunk)} bajtów: {repr(chunk[:60])}...")
                line_buf += chunk
                while "\n" in line_buf or "\r" in line_buf:
                    sep = "\n" if "\n" in line_buf else "\r"
                    line, line_buf = line_buf.split(sep, 1)
                    line = line.strip()
                    if self._on_line and line:
                        self._on_line("handshake", line, None)
                    if line == "READY":
                        return True
                    if line.startswith("READY"):
                        return True
            now = time.monotonic()
            if now - last_log >= 5.0:
                _log("  [INFO] Czekam na READY...")
                last_log = now
            time.sleep(0.01)
        return False  # unreachable

    def send_ready_ack(self) -> bool:
        """Wyślij testowy handshake: READY_ACK\\n."""
        if not self._ser or not self._ser.is_open:
            self.open()
        data = b"READY_ACK\n"
        old_wt = self._ser.write_timeout
        self._ser.write_timeout = 0
        try:
            pos = 0
            deadline = time.monotonic() + 5.0
            while pos < len(data) and time.monotonic() < deadline:
                n = self._ser.write(data[pos:])
                if n > 0:
                    pos += n
                time.sleep(0.02)
            if pos < len(data):
                _log(f"  [UWAGA] READY_ACK: wysłano tylko {pos}/{len(data)} bajtów")
        finally:
            self._ser.write_timeout = old_wt
        self._ser.flush()
        return True

    def wait_ready_ack_ok(self) -> bool:
        """Czekaj bez timeoutu na READY_ACK_OK od ESP (potwierdzenie)."""
        if not self._ser:
            return False
        line_buf = ""
        seen_any = False
        last_log = time.monotonic()
        while True:
            chunk = self._ser.read(256).decode("utf-8", errors="ignore")
            if chunk:
                if not seen_any:
                    _log(f"  [DEBUG] ACK RX {len(chunk)} bajtów: {repr(chunk[:80])}...")
                    seen_any = True
                line_buf += chunk
                while "\n" in line_buf or "\r" in line_buf:
                    sep = "\n" if "\n" in line_buf else "\r"
                    line, line_buf = line_buf.split(sep, 1)
                    line = line.strip()
                    if self._on_line and line:
                        self._on_line("handshake", line, None)
                    if "READY_ACK_OK" in line:
                        return True
            now = time.monotonic()
            if now - last_log >= 5.0:
                _log("  [INFO] Czekam na READY_ACK_OK...")
                last_log = now
            time.sleep(0.01)
        return False  # unreachable


class EspGetData:
    """Żądanie i odbiór danych CSI z ESP32."""

    def __init__(self, ser: Optional[serial.Serial] = None, port: str = "", baud: int = 921600):
        if serial is None:
            raise ImportError("Zainstaluj: pip install pyserial")
        self._ser = ser
        self._own_ser = ser is None
        self._port = port
        self._baud = baud
        self._on_line: Optional[Callable[[str, str, Optional[Dict[str, Any]]], None]] = None

    def set_on_line(self, on_line: Optional[Callable[[str, str, Optional[Dict[str, Any]]], None]]) -> None:
        self._on_line = on_line

    def _ensure_ser(self) -> serial.Serial:
        if self._ser is None or not self._ser.is_open:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.1)
        return self._ser

    def send_get_data(self) -> None:
        """Wyślij komendę GET_DATA do ESP32 (format: GET_DATA\\n - jak w scan.c)."""
        ser = self._ensure_ser()
        data = b"GET_DATA\n"
        old_wt = ser.write_timeout
        ser.write_timeout = 0
        try:
            pos = 0
            deadline = time.monotonic() + 10.0
            while pos < len(data) and time.monotonic() < deadline:
                n = ser.write(data[pos:])
                if n > 0:
                    pos += n
                time.sleep(0.02)
        finally:
            ser.write_timeout = old_wt
        ser.flush()

    def receive_data(
        self,
        save_to: Optional[str] = None,
        file_handle: Optional[TextIO] = None,
        on_packet: Optional[Callable[[dict], None]] = None,
        timeout_sec: Optional[float] = None,
        append: bool = False,
        send_get_data_first: bool = True,
    ) -> list:
        """
        Odbierz dane CSI. Gdy send_get_data_first=False (tryb read_only), tylko czyta -
        ESP wysyła dane automatycznie co 5s.
        """
        if send_get_data_first:
            _log("  [DEBUG] receive_data: wysyłam GET_DATA...")
            self.send_get_data()
            _log("  [DEBUG] receive_data: GET_DATA wysłane, zaczynam czytać...")
        else:
            _log("  [DEBUG] receive_data: tryb read_only - tylko odczyt (ESP wysyła co 5s)")
        ser = self._ensure_ser()
        packets: list = []
        line_buf = ""
        deadline = (time.monotonic() + timeout_sec) if timeout_sec is not None else None
        data_end = False
        fh = file_handle

        if save_to and not fh:
            fh = open(save_to, "a" if append else "w", newline="")

        try:
            from csi_parse import parse_csi_line
        except ImportError:
            parse_csi_line = lambda x: None  # type: ignore

        last_progress = 0.0
        debug_raw = True  # Wypisuj co otrzymujemy z COM
        read_count = 0
        total_bytes = 0
        start_time = time.monotonic()
        _log(f"  [DEBUG] Rozpoczynam odbiór (port={ser.port}, baud={ser.baudrate})...")
        while (deadline is None or time.monotonic() < deadline) and not data_end:
            try:
                chunk = ser.read(256).decode("utf-8", errors="ignore")
            except KeyboardInterrupt:
                _log("  [INFO] Przerwano odbiór (Ctrl+C)")
                break
            read_count += 1
            if chunk:
                total_bytes += len(chunk)
                _log(f"  [COM<-] {len(chunk)} bajtów: {repr(chunk[:120])}")
                line_buf += chunk
                while "\n" in line_buf or "\r" in line_buf:
                    sep = "\n" if "\n" in line_buf else "\r"
                    line, line_buf = line_buf.split(sep, 1)
                    line = line.strip()
                    if not line:
                        continue
                    if debug_raw:
                        preview = line[:80] + "..." if len(line) > 80 else line
                        _log(f"  [LINIA] {preview}")
                    if self._on_line:
                        self._on_line("get_data", line, None)
                    if line.startswith("WAIT_SAMPLES|") and self._on_line:
                        self._on_line("get_data_wait", line, None)
                    if line.startswith("DATA_END|"):
                        if send_get_data_first:
                            data_end = True
                            break
                        # read_only: nie kończ - ESP wyśle kolejną paczkę za 5s
                    pkt = parse_csi_line(line)
                    if pkt is not None:
                        packets.append(pkt)
                        if self._on_line:
                            # zapisz także w formie zparsowanej
                            self._on_line("csi", line, pkt)
                        if on_packet:
                            on_packet(pkt)
            # Co 5 s: komunikat postępu
            now = time.monotonic()
            if now - last_progress >= 5.0:
                elapsed = int(now - start_time)
                msg = f"  ... czekam ({elapsed}s, odebrano {len(packets)} pkt, {total_bytes} bajtów)"
                if total_bytes == 0 and elapsed >= 5:
                    msg += " - BRAK DANYCH! Sprawdź port COM i baud."
                _log(msg)
                last_progress = now
            time.sleep(0.01)

        if save_to and fh and not file_handle:
            fh.close()

        # Diagnostyka braków: seq w firmware = sample_id
        if packets:
            try:
                seqs = sorted(int(p["seq"]) for p in packets if "seq" in p)
                gaps = 0
                for a, b in zip(seqs, seqs[1:]):
                    if b > a + 1:
                        gaps += (b - a - 1)
                if gaps > 0:
                    _log(f"  [UWAGA] Wykryto luki w seq (prawdopodobnie drop): gaps={gaps}, range={seqs[0]}..{seqs[-1]}")
            except Exception:
                pass

        return packets


class EspController:
    """
    Główny kontroler ESP32 CSI.
    Łączy EspSendConf i EspGetData, zapewnia inicjalizację i okresowe odpytywanie.
    """

    def __init__(
        self,
        port: str,
        config: CsiConfig,
        baud: int = 3000000,
        poll_interval_sec: float = 0.0,
        csi_log_path: Optional[str] = None,
        info_log_path: Optional[str] = None,
        log_path: Optional[str] = None,  # legacy: traktuj jako info_log_path
    ):
        if serial is None:
            raise ImportError("Zainstaluj: pip install pyserial")
        self.port = port
        self.config = config
        self.baud = baud
        self.poll_interval_sec = poll_interval_sec
        self._ser: Optional[serial.Serial] = None
        self._send_conf: Optional[EspSendConf] = None
        self._get_data: Optional[EspGetData] = None
        self._running = False
        self._poll_thread: Optional[threading.Thread] = None
        if info_log_path is None and log_path:
            info_log_path = log_path
        if csi_log_path and info_log_path:
            self._logger: Optional[JsonlSplitLogger] = JsonlSplitLogger(csi_log_path, info_log_path)
        else:
            # fallback: pojedynczy plik (stare zachowanie)
            self._logger = JsonlLogger(info_log_path) if info_log_path else None  # type: ignore[assignment]

    def _on_line(self, phase: str, line: str, parsed: Optional[Dict[str, Any]]) -> None:
        if self._logger:
            self._logger.log_line(phase, line, parsed)

    def start(self, wait_ready: bool = True, timeout_sec: float = 10.0) -> bool:
        """
        Uruchom - połącz z ESP32, wyślij konfigurację.
        Jeśli wait_ready=True, czeka na ESP (po resecie) i wysyła config.
        Gdy ESP jest już w trybie pracy, wysyła RECONFIG aby wymusić restart.
        Zwraca True jeśli sukces.
        """
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=60.0)
        self._ser.reset_input_buffer()
        # Reset ESP przez DTR
        self._ser.dtr = False
        time.sleep(0.05)
        self._ser.dtr = True
        time.sleep(8.0)  # Czekaj na boot ESP (ESP32-C3 może potrzebować ~6-7s)
        if self._logger:
            self._logger.open()
            self._logger.write(
                {
                    "type": "event",
                    "t_host_epoch": time.time(),
                    "t_host_mono": time.monotonic(),
                    "phase": "start",
                    "event": "serial_opened_and_reset",
                    "port": self.port,
                    "baud": self.baud,
                }
            )

        self._send_conf = EspSendConf(port=self.port, baud=self.baud, on_line=self._on_line)
        self._send_conf._ser = self._ser  # type: ignore
        self._get_data = EspGetData(ser=self._ser)
        self._get_data.set_on_line(self._on_line)

        if wait_ready:
            # Czekaj na READY - boot ESP32-C3 może trwać ~7-8s
            got_ready = self._send_conf.wait_ready()
            if got_ready:
                _log("  [OK] Otrzymano READY od ESP32")
                _log("  [INFO] Wysyłam READY_ACK (test TX host→ESP)...")
                self._send_conf.send_ready_ack()
                if self._send_conf.wait_ready_ack_ok():
                    _log("  [OK] Otrzymano READY_ACK_OK od ESP (TX działa)")
                else:
                    _log("  [BŁĄD] Brak READY_ACK_OK - przerywam (ESP nie odbiera komend hosta)")
                    return False
            else:
                _log("  [BŁĄD] Brak READY - przerywam (brak synchronizacji z ESP)")
                return False
            time.sleep(0.5)  # Pauza po odczycie - USB-UART potrzebuje chwili
        _log("  [INFO] Wysyłam CONFIG...")
        self._send_conf.send_config(self.config)
        _log("  [INFO] Czekam na CONFIG_OK od ESP (bez timeoutu)...")
        if not self._wait_for_config_ok(timeout_sec=None):
            _log("  [BŁĄD] Brak CONFIG_OK - przerywam (ESP nie odebrał konfiguracji)")
            return False
        _log("  [INFO] Czekam na WIFI_OK od ESP (bez timeoutu)...")
        if not self._wait_for_wifi_ready(timeout_sec=None):
            return False
        self._running = True
        return True

    def _wait_for_config_ok(self, timeout_sec: Optional[float] = None) -> bool:
        """Czytaj z portu aż zobaczysz CONFIG_OK/CONFIG_ERR. timeout_sec=None => bez timeoutu."""
        if not self._ser:
            return False
        deadline = (time.monotonic() + timeout_sec) if timeout_sec is not None else None
        line_buf = ""
        last_log = time.monotonic()
        while deadline is None or time.monotonic() < deadline:
            chunk = self._ser.read(256).decode("utf-8", errors="ignore")
            if chunk:
                line_buf += chunk
                while "\n" in line_buf or "\r" in line_buf:
                    sep = "\n" if "\n" in line_buf else "\r"
                    line, line_buf = line_buf.split(sep, 1)
                    line = line.strip()
                    if line and self._logger:
                        self._on_line("config", line, None)
                    if "CONFIG_OK" in line:
                        _log("  [OK] ESP potwierdził odbiór CONFIG")
                        return True
                    if "CONFIG_ERR" in line:
                        _log(f"  [BŁĄD] ESP odrzucił CONFIG: {line.strip()}")
                        return False
            now = time.monotonic()
            if now - last_log >= 5.0:
                _log("  [INFO] Czekam na CONFIG_OK...")
                last_log = now
            time.sleep(0.01)
        _log("  [UWAGA] Brak CONFIG_OK - ESP mógł nie odebrać konfiguracji")
        return False

    def _wait_for_wifi_ready(self, timeout_sec: Optional[float] = None) -> bool:
        """Czekaj na WIFI_OK (albo legacy 'Połączono'). timeout_sec=None => bez timeoutu."""
        if not self._ser:
            return False
        deadline = (time.monotonic() + timeout_sec) if timeout_sec is not None else None
        line_buf = ""
        chunks_seen = 0
        last_log = time.monotonic()
        while deadline is None or time.monotonic() < deadline:
            chunk = self._ser.read(256).decode("utf-8", errors="ignore")
            if chunk:
                chunks_seen += 1
                if chunks_seen <= 3:  # Pierwsze odczyty - diagnostyka
                    _log(f"  [WiFi] {len(chunk)} bajtów: {repr(chunk[:80])}...")
                line_buf += chunk
                while "\n" in line_buf or "\r" in line_buf:
                    sep = "\n" if "\n" in line_buf else "\r"
                    line, line_buf = line_buf.split(sep, 1)
                    line = line.strip()
                    if line and self._logger:
                        self._on_line("wifi", line, None)
                    if "WIFI_CONNECTING" in line:
                        _log("  [ESP] WIFI_CONNECTING")
                    if "WIFI_FAIL" in line:
                        _log("  [BŁĄD] WIFI_FAIL (ESP nie połączył się z WiFi)")
                        return False
                    if "WIFI_OK" in line:
                        _log("  [OK] WIFI_OK (ESP połączone z WiFi)")
                        time.sleep(1.0)
                        return True
                    if "Połączono" in line or "Łączenie" in line:
                        _log(f"  [ESP] {line.strip()}")
                    if "Połączono" in line:
                        _log("  [INFO] ESP połączone z WiFi - czekam 2s na utworzenie zadań...")
                        time.sleep(2.0)
                        return True
            now = time.monotonic()
            if now - last_log >= 5.0:
                _log("  [INFO] Czekam na WIFI_OK...")
                last_log = now
            time.sleep(0.01)
        return False

    def start_read_only(self) -> bool:
        """Tryb read_only: otwórz port, reset ESP, czekaj na WiFi. BEZ wysyłania CONFIG."""
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self._ser.reset_input_buffer()
        self._ser.dtr = False
        time.sleep(0.05)
        self._ser.dtr = True
        time.sleep(8.0)
        self._get_data = EspGetData(ser=self._ser)
        _log("  [INFO] Tryb read_only - ESP użyje domyślnej konfiguracji (timeout 5s)")
        _log("  [INFO] Czekam na połączenie WiFi ESP (max 30s)...")
        self._wait_for_wifi_ready(timeout_sec=30.0)
        self._running = True
        return True

    def request_data(
        self,
        save_to: Optional[str] = None,
        on_packet: Optional[Callable[[dict], None]] = None,
        timeout_sec: float = 30.0,
        append: bool = False,
        send_get_data_first: bool = True,
    ) -> list:
        """Wyślij GET_DATA i odbierz dane. Zapisz do save_to jeśli podano."""
        if not self._get_data:
            return []
        return self._get_data.receive_data(
            save_to=save_to,
            on_packet=on_packet,
            timeout_sec=timeout_sec,
            append=append,
            send_get_data_first=send_get_data_first,
        )

    def reconfigure(self, new_config: CsiConfig) -> bool:
        """Wyślij RECONFIG - ESP zatrzyma usługi i uruchomi z nową konfiguracją."""
        if not self._send_conf:
            return False
        self.config = new_config
        return self._send_conf.send_reconfig(new_config)

    def send_get_data(self) -> None:
        """Tylko wyślij GET_DATA (bez odbioru - np. do asynchronicznego odbioru)."""
        if self._get_data:
            self._get_data.send_get_data()

    def stop(self) -> None:
        """Zamknij połączenie."""
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None
        if self._logger:
            self._logger.write(
                {
                    "type": "event",
                    "t_host_epoch": time.time(),
                    "t_host_mono": time.monotonic(),
                    "phase": "stop",
                    "event": "controller_stopped",
                }
            )
            self._logger.close()

    def run_polling(
        self,
        save_to: Optional[str] = None,
        poll_callback: Optional[Callable[[list], None]] = None,
        interval_sec: Optional[float] = None,
    ) -> None:
        """
        Uruchom w tle okresowe odpytywanie (GET_DATA co interval_sec).
        save_to: plik do zapisu (opcjonalnie)
        poll_callback: wywołanie po każdym pobraniu z listą pakietów
        """
        iv = interval_sec or self.poll_interval_sec
        if iv <= 0:
            raise ValueError("interval_sec musi być > 0")

        def _poll_loop():
            first = True
            while self._running:
                packets = self.request_data(
                    save_to=save_to,
                    timeout_sec=None,
                    append=not first and save_to is not None,
                )
                first = False
                if poll_callback and packets:
                    poll_callback(packets)
                time.sleep(iv)

        self._poll_thread = threading.Thread(target=_poll_loop, daemon=True)
        self._poll_thread.start()


def main():
    def out(msg: str) -> None:
        print(msg, flush=True)

    out("=== Kontroler ESP32 CSI ===")

    # Pokaż dostępne porty COM (diagnostyka - upewnij się że PORT jest właściwy)
    try:
        if serial:
            ports = list(serial.tools.list_ports.comports())
            if ports:
                out("Dostępne porty: " + ", ".join(f"{p.device}" for p in ports))
            else:
                out("Brak wykrytych portów COM!")
    except Exception:
        pass

    config = CsiConfig(
        ssid=WIFI_SSID,
        password=WIFI_PASSWORD,
        csi_queue_len=CSI_QUEUE_LEN,
        csi_buf_max=CSI_BUF_MAX,
        mode=CSI_MODE,
        interval_ms=CSI_INTERVAL_MS,
        channel=CSI_CHANNEL,
        channel_width=CSI_CHANNEL_WIDTH,
        request_count=CSI_REQUEST_COUNT,
    )

    port_use = _port_str(PORT)
    out(f"Port: {PORT} (używam: {port_use}), baud: {BAUD}")
    out(f"WiFi: {WIFI_SSID}")
    out("Otwieram port...")

    try:
        ctrl = EspController(
            port_use,
            config,
            baud=BAUD,
            csi_log_path=SAVE_CSI_FILE,
            info_log_path=SAVE_INFO_FILE,
        )
    except Exception as e:
        out(f"Błąd otwarcia portu: {e}")
        return 1

    if RUN_MODE == "get_data":
        out("Czekam na READY od ESP32...")
        if not ctrl.start():
            out("Błąd: inicjalizacja nie powiodła się (READY/READY_ACK_OK/CONFIG_OK)")
            out("Sprawdź: 1) port COM, 2) czy monitor szeregowy nie jest otwarty, 3) firmware ESP ma nowy protokół")
            return 1
        out("Konfiguracja wysłana. ESP32 w trybie pracy.")
        if FIRST_GET_DATA_DELAY_SEC > 0:
            out(f"Czekam {FIRST_GET_DATA_DELAY_SEC}s, żeby ESP zebrało próbki...")
            time.sleep(FIRST_GET_DATA_DELAY_SEC)
        out(f"Wysyłam GET_DATA (ostatnie {CSI_REQUEST_COUNT}), czekam na dane...")
        _log("  [DEBUG] Wywołuję request_data...")
        try:
            packets = ctrl.request_data(save_to=None, timeout_sec=None)
        except KeyboardInterrupt:
            out("\nPrzerwano (Ctrl+C).")
            ctrl.stop()
            return 0
        out(f"Odebrano {len(packets)} pakietów CSI")
        out(f"Zapisano CSI do {SAVE_CSI_FILE}")
        out(f"Zapisano INFO do {SAVE_INFO_FILE}")

    elif RUN_MODE == "poll":
        def on_data(pkts):
            out(f"Pobrano {len(pkts)} pakietów")

        out(f"Odpytywanie co {POLL_INTERVAL_SEC}s (Ctrl+C = koniec)")
        ctrl.run_polling(save_to=None, poll_callback=on_data, interval_sec=POLL_INTERVAL_SEC)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            out("\nKoniec.")

    else:
        out("Tryb config_only - konfiguracja wysłana. Zmień RUN_MODE na get_data lub poll.")

    ctrl.stop()
    out("Zamknięto.")
    return 0


if __name__ == "__main__":
    exit(main())
