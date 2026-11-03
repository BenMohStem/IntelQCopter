#!/usr/bin/env python3
"""Stable JSON enumeration for Windows PowerShell 5.1 serial-port selection."""
import json
from serial.tools import list_ports


def all_ports():
    return sorted(({
        'device': str(p.device),
        'description': str(p.description or ''),
        'hwid': str(p.hwid or ''),
        'vid': p.vid,
        'pid': p.pid,
        'serial_number': str(p.serial_number or ''),
        'is_target': bool((p.vid == 0x1A86 and p.pid == 0x55D3) or
                          ('CH343' in str(p.description).upper()) or
                          ('CH343' in str(p.hwid).upper()))
    } for p in list_ports.comports()), key=lambda p: p['device'])


if __name__ == '__main__':
    print(json.dumps({'ports': all_ports()}, separators=(',', ':')))
