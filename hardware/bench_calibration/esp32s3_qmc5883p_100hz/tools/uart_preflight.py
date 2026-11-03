#!/usr/bin/env python3
"""Read existing IntelQCopter v3 52-byte binary packets via CH343 serial."""
import argparse
import collections
import csv
import datetime
from pathlib import Path
import struct
import time
import serial

BAUD=921600
SIZE=52

def crc16(b):
    c=0xFFFF
    for v in b:
        c ^= v<<8
        for _ in range(8):
            c=((c<<1)^0x1021)&0xFFFF if c&0x8000 else (c<<1)&0xFFFF
    return c

def decode(b):
    if len(b)!=SIZE or b[:2]!=b'\xa5\x5a' or b[2]!=3: return None
    if crc16(b[:50])!=struct.unpack_from('<H',b,50)[0]: return None
    flags=b[3]
    seq,t_us,late=struct.unpack_from('<IIH',b,4)
    a=struct.unpack_from('<hhh',b,14)
    g=struct.unpack_from('<hhh',b,20)
    m=struct.unpack_from('<hhh',b,28)
    temp,pr=struct.unpack_from('<hI',b,42)
    missed=struct.unpack_from('<H',b,48)[0]
    return (seq,t_us,late,flags,*a,*g,*m,temp,pr,missed)

def run(port,seconds):
    directory=Path('captures')
    directory.mkdir(exist_ok=True)
    path=directory/('usb_preflight_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.csv')
    columns=['sequence','t_us','lateness_us','flags','ax','ay','az','gx','gy','gz','mx','my','mz','bmp_centi_c','bmp_centi_pa','missed_deadlines']
    valid=0
    first=None
    last=None
    flagsum=collections.Counter()
    first_mag=None
    last_mag=None
    invalid=0
    lost=0
    bytes_total=0
    buf=bytearray()
    ser=serial.Serial(port=port,baudrate=BAUD,timeout=0.05,rtscts=False,dsrdtr=False)
    try:
        print('Connected to',port,'at',BAUD,'baud. No Wi-Fi. No motor output.')
        with path.open('w',newline='',encoding='utf-8') as f:
            w=csv.writer(f)
            w.writerow(columns)
            begin=time.monotonic()
            previous=None
            last_report=begin
            while time.monotonic()-begin<seconds:
                data=ser.read(max(1,ser.in_waiting or 128))
                bytes_total+=len(data)
                buf.extend(data)
                if len(buf)>100000: del buf[:-100]
                while True:
                    j=buf.find(b'\xa5\x5a')
                    if j<0:
                        if len(buf)>1: del buf[:-1]
                        break
                    if j: del buf[:j]
                    if len(buf)<SIZE:break
                    p=decode(buf[:SIZE])
                    if p is None:
                        invalid+=1
                        del buf[0]
                        continue
                    del buf[:SIZE]
                    seq,t_us,late,flag,*_=p
                    if previous is not None:
                        gap=(seq-previous)&0xFFFFFFFF
                        if gap>1 and gap<100000:lost+=gap-1
                    previous=seq
                    first=t_us if first is None else first
                    last=t_us
                    valid+=1
                    flagsum['imu']+=bool(flag&1)
                    flagsum['bmp']+=bool(flag&2)
                    flagsum['mag']+=bool(flag&4)
                    flagsum['mag_fresh']+=bool(flag&16)
                    if flag&4:
                        m=tuple(p[10:13])  # decoded x,y,z after six MPU axis values
                        if first_mag is None: first_mag=m
                        last_mag=m
                    flagsum['bmp_fresh']+=bool(flag&8)
                    w.writerow(p)
                now=time.monotonic()
                if now-last_report>=2:
                    hz=(valid-1)*1e6/((last-first)&0xFFFFFFFF) if valid>1 and last!=first else 0
                    print(f'bytes={bytes_total} frames={valid} imu={flagsum["imu"]} mag_valid={flagsum["mag"]} mag_fresh={flagsum["mag_fresh"]} bmp_fresh={flagsum["bmp_fresh"]} link_hz={hz:.1f} invalid={invalid} lost={lost}')
                    last_report=now
    finally:
        ser.close()
    secs=((last-first)&0xFFFFFFFF)/1e6 if valid>1 else 0
    frame_rate=(valid-1)/secs if secs else 0
    mag_rate=flagsum['mag_fresh']/secs if secs else 0
    print(f'FINAL bytes={bytes_total}, frames={valid}, packet_hz={frame_rate:.1f}, mag_valid={flagsum["mag"]}, mag_fresh_hz={mag_rate:.1f}, bmp_fresh={flagsum["bmp_fresh"]}, first_mag={first_mag}, last_mag={last_mag}, crc_bad={invalid}, seq_lost={lost}, output={path}')
    if bytes_total==0:
        print(f'FAIL: No UART bytes received at {BAUD} baud. Confirm the full calibration firmware is running; text diagnostics at 115200 are not GUI packets.')
        return 5
    if not valid:
        print(f'FAIL: Received {bytes_total} bytes but no CRC-valid v3 frames. Check baud, protocol and firmware image.')
        return 6
    if not valid or not flagsum['imu'] or not flagsum['mag_fresh']:
        print('FAIL: Required MPU or MAG data not observed. This is not flight-ready.')
        print('Next: .\\tools\\start_calibration.ps1 -MagProbe -Seconds 20')
        return 2
    if mag_rate<70:
        print('WARNING: Effective MAG rate below 70Hz. Investigate sensor I2C/readbacks.')
        return 3
    if flagsum['bmp_fresh']==0:
        print('WARNING: BMP280 fresh flag never observed.')
        return 4
    print('PASS: UART packets and 3 sensor streams observed. MAG fresh uses QMC internal 0x09 data-ready bit.')
    return 0

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--port',default='COM3')
    p.add_argument('--seconds',type=float,default=20)
    a=p.parse_args()
    raise SystemExit(run(a.port,a.seconds))
