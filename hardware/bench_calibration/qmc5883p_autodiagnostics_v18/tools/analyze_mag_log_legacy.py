#!/usr/bin/env python3
"""Analyze IntelQCopter QMC5883P v13/v14/v15 diagnostic logs.

Physical sample frequency is reported from the firmware fresh counter, NOT
from CSV lines (CSV can be intentionally decimated by IQ_MAG_CSV_DIVIDER).
WiFi pre/post compares HEALTH windows, not a test of physical causality alone.
"""
import re
import sys
from pathlib import Path
from statistics import mean, median

CSV_RE = re.compile(r"CSV,(\d+),(-?\d+),(-?\d+),(-?\d+),0x([0-9a-fA-F]+),(\d+)(?:,(\d+))?")
KV_RE = re.compile(r"\b([a-zA-Z_]+)=([^\s]+)")


def health_fields(line):
    return {m.group(1): m.group(2) for m in KV_RE.finditer(line)}


def numeric(dic, key, default=None):
    try:
        return float(dic[key])
    except (KeyError, ValueError, TypeError):
        return default


def analyze(text):
    csv = []
    for line in text.splitlines():
        m = CSV_RE.search(line)
        if m:
            row = tuple(int(m.group(i), 16) if i == 5 else int(m.group(i)) for i in range(1, 7))
            csv.append(row + (int(m.group(7)) if m.group(7) else None,))

    hs = [line[line.index('HEALTH '):] for line in text.splitlines() if 'HEALTH ' in line]
    hparsed = [health_fields(h) for h in hs]
    req = re.search(r'BUILD rate=(\d+)', text) or re.search(r'BOOT_PROFILE requested=(\d+)', text)
    wifi_mode = re.search(r'BUILD wifi_mode=(\d+)', text)
    last = hparsed[-1] if hparsed else {}
    raw_last = hs[-1] if hs else ''
    intervals = [(b[0]-a[0])/1000 for a,b in zip(csv,csv[1:]) if b[0]>a[0]]
    phases = {}
    for active, label in ((0,'before_wifi'),(1,'after_wifi')):
        windows = [d for d in hparsed if numeric(d,'wifi_active')==active]
        rates = [numeric(d,'measured_hz_window') for d in windows]
        rates = [r for r in rates if r is not None]
        phases[label] = {
            'health_windows':len(windows),
            'mean_window_hz':round(mean(rates),2) if rates else None,
            'last_window_hz':rates[-1] if rates else None,
            'last_status_fail':numeric(windows[-1], 'status_fail') if windows else None,
            'last_verify_fail':numeric(windows[-1], 'verify_fail') if windows else None,
            'last_regs_ok':int(numeric(windows[-1], 'regs_ok', -1)) if windows else None,
        }
    # In WiFi OFF runs all windows are recorded as before_wifi.
    latest_rate = numeric(last, 'measured_hz_window', numeric(last, 'measured_fresh_hz'))
    out = {
        'requested_hz':int(req.group(1)) if req else None,
        'wifi_stress_mode':int(wifi_mode.group(1)) if wifi_mode else None,
        'wifi_start_success': 'WIFI_EVENT stage=READY' in text and 'active=1' in text,
        'wifi_events':sum(1 for line in text.splitlines() if 'WIFI_EVENT' in line),
        'fresh_count_final':int(numeric(last,'fresh',len(csv))),
        'fresh_rate_last_hz':latest_rate,
        'status_fail_final':int(numeric(last,'status_fail',-1)),
        'verify_fail_final':int(numeric(last,'verify_fail',-1)),
        'configured_final':int(numeric(last,'regs_ok',-1)),
        'CSV_count':len(csv),
        'CSV_duration_s':round((csv[-1][0]-csv[0][0])/1e6,3) if len(csv)>1 else None,
        'CSV_interval_median_ms':round(median(intervals),3) if intervals else None,
        'CSV_interval_mean_ms':round(mean(intervals),3) if intervals else None,
        'verified_csv_rows':sum(1 for row in csv if row[5]==1),
        'reset_signatures':text.count('RESET_SIGNATURE:'),
        'recovery_events':text.count('RECOVERY_ATTEMPT'),
        'fallback_events':text.count('FALLBACK_100HZ'),
        'before_wifi':phases['before_wifi'],
        'after_wifi':phases['after_wifi'],
        'last_health':raw_last,
    }
    out['pass_200hz_provisional'] = (out['requested_hz']==200
            and out['fresh_rate_last_hz'] is not None
            and 180 <= out['fresh_rate_last_hz'] <= 220
            and out['fresh_count_final'] >= 500
            and out['configured_final'] == 1
            and out['fallback_events']==0 and out['reset_signatures']==0)
    return out


def main():
    if len(sys.argv)!=2:
        print('Usage: python tools/analyze_mag_log.py serial_output.txt')
        return 2
    r=analyze(Path(sys.argv[1]).read_text(encoding='utf-8-sig',errors='replace'))
    for key,val in r.items(): print(f'{key}: {val}')
    print('VERDICT: 200 Hz provisionally supported (repeat cold boots and endurance tests).'
          if r['pass_200hz_provisional'] else
          'VERDICT: This file does NOT prove stable 200 Hz operation.')
    return 0

if __name__=='__main__':
    raise SystemExit(main())
