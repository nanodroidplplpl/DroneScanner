#!/usr/bin/env python3
"""
Parser linii CSI CSV wysyłanych przez firmware ESP (`scan.c`).

Obsługiwane formaty:
- Nowy: CSI,seq,rssi,ch,ts,len,fwi,sig,cwb,stbc,v0,v1,...,vN
- Stary: CSI,seq,rssi,ch,ts,len,v0,v1,...,vN

Uwaga:
`len` w naszym protokole oznacza liczbę wartości v0..vN (int) do odczytu
(zwykle int8 wypisane jako liczby dziesiętne).
"""

from __future__ import annotations

from typing import Optional, Dict, Any, List, Tuple


def parse_csi_line(line: str) -> Optional[Dict[str, Any]]:
    line = line.strip()
    if not line.startswith("CSI,"):
        return None

    # obcinamy prefix "CSI," i parsujemy resztę
    parts = line[4:].split(",")
    if len(parts) < 6:
        return None
    try:
        seq = int(parts[0])
        rssi = int(parts[1])
        ch = int(parts[2])
        ts = int(parts[3])
        length = int(parts[4])
        if length < 0:
            return None

        values: List[int]
        meta: Dict[str, Any]

        # Nowy format: ...len,fwi,sig,cwb,stbc,v0..vN
        if len(parts) >= 9 + length:
            fwi = int(parts[5])
            sig_mode = int(parts[6])
            cwb = int(parts[7])
            stbc = int(parts[8])
            values = [int(x) for x in parts[9 : 9 + length]] if length > 0 else []
            meta = {"first_word_invalid": bool(fwi), "sig_mode": sig_mode, "cwb": cwb, "stbc": stbc}
        # Stary format: ...len,v0..vN
        elif len(parts) >= 5 + length:
            values = [int(x) for x in parts[5 : 5 + length]] if length > 0 else []
            meta = {"first_word_invalid": False, "sig_mode": 0, "cwb": 0, "stbc": 0}
        else:
            return None

        if len(values) != length:
            return None

        return {
            "seq": seq,
            "rssi": rssi,
            "channel": ch,
            "timestamp": ts,
            "len": length,
            "csi": values,
            **meta,
        }
    except (ValueError, IndexError):
        return None
