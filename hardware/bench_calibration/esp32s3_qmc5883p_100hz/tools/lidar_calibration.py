#!/usr/bin/env python3
"""Optional TOFSense F2 Mini bench calibration; does NOT run in FC firmware.

The USB-UART adapter must have 3.3V logic and share GND; F2 Mini is
powered at 4.3–5.2V according to its datasheet. Do not feed 5V into RX.
Manufacturer NLink_TOFSense_Frame0, 16 bytes, 921600 baud, up to 50 Hz.
GUI can be launched directly or from IntelQCopter's main calibration GUI.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
from dataclasses import dataclass
from datetime import datetime, timezone
import struct
import threading
import time
import queue

ROOT=Path(__file__).resolve().parent.parent
DEFAULT_DISTANCES=[0.25,0.50,1.00,2.00]
CSV_HEADERS=('utc_iso','host_monotonic_s','stage','reference_m','module_id',
             'sensor_time_ms','distance_m','status','signal_strength','range_precision_cm','valid')

@dataclass(frozen=True)
class RangeFrame:
    module_id:int
    sensor_time_ms:int
    distance_m:float
    status:int
    signal_strength:int
    range_precision_cm:int
    @property
    def valid(self):
        return self.status==0 and math.isfinite(self.distance_m) and 0.02<=self.distance_m<=7.8

class NLinkDecoder:
    """Streaming parser; rejects bad sum/header and handles partial frames."""
    def __init__(self):
        self.buf=bytearray()
        self.good=0
        self.bad_checksum=0
        self.resync=0
    def feed(self,data):
        self.buf.extend(data)
        parsed=[]
        while len(self.buf)>=16:
            i=self.buf.find(b'\x57\x00')
            if i<0:
                self.resync+=len(self.buf)-1
                del self.buf[:-1]
                break
            if i:
                self.resync+=i
                del self.buf[:i]
            if len(self.buf)<16:
                break
            p=self.buf[:16]
            if (sum(p[:15])&255)!=p[15]:
                self.bad_checksum+=1
                del self.buf[0]
                continue
            mm=int.from_bytes(p[8:11],'little',signed=False)
            if mm&0x800000:mm-=1<<24
            parsed.append(RangeFrame(p[3],int.from_bytes(p[4:8],'little'),
                 mm/1000.0,p[11],int.from_bytes(p[12:14],'little'),p[14]))
            self.good+=1
            del self.buf[:16]
        return parsed

def make_test_frame(distance_mm=1000,status=0,sys_ms=1234,signal=10):
    p=bytearray([0x57,0x00,0xff,0x00])
    p+=struct.pack('<I',sys_ms)
    p+=int(distance_mm%(1<<24)).to_bytes(3,'little')
    p+=bytes([status])+struct.pack('<H',signal)+b'\x03'
    p+=bytes([sum(p)&255])
    assert len(p)==16
    return bytes(p)


def analyze_csv(path, output=None, min_samples=20, required_distances=None):
    import numpy as np
    paths=[Path(item) for item in path] if isinstance(path,(list,tuple)) else [Path(path)]
    grouped={}
    invalids=0
    for source in paths:
        with source.open(newline='',encoding='utf-8-sig') as stream:
            reader=csv.DictReader(stream)
            columns=set(reader.fieldnames or ())
            gui_csv='lidar_reference_m' in columns
            source_groups={}
            latest_attempt_by_distance={}
            for r in reader:
                if gui_csv:
                    if r.get('kind')!='L':continue
                    try:
                        d=float(r['lidar_reference_m']); z=float(r['lidar_distance_m'])
                        attempt_num=int(r.get('phase_attempt') or 0)
                    except (KeyError,ValueError,TypeError):
                        invalids+=1;continue
                    good=(r.get('phase_accepted')=='1' and r.get('lidar_status')=='0'
                          and 0.02<=z<=7.8)
                    latest_attempt_by_distance[d]=max(
                        attempt_num,latest_attempt_by_distance.get(d,attempt_num))
                else:
                    if r.get('valid')!='1':
                        invalids+=1;continue
                    try:
                        d=float(r['reference_m']); z=float(r['distance_m'])
                    except (KeyError,ValueError):continue
                    good=True
                    attempt_num=0
                if not (math.isfinite(d) and math.isfinite(z)):
                    invalids+=1;continue
                if not good:
                    invalids+=1;continue
                source_groups.setdefault((d,attempt_num),[]).append(z)
        # Count valid data only from the newest attempt for each reference. If
        # that attempt has no valid rows, an older attempt stays superseded.
        latest={}
        for (distance,attempt),values in source_groups.items():
            if gui_csv and attempt!=latest_attempt_by_distance.get(distance):
                continue
            if distance not in latest or attempt>latest[distance][0]:
                latest[distance]=(attempt,values)
        for distance,(_,values) in latest.items():
            grouped.setdefault(distance,[]).extend(values)
    min_samples=max(1,int(min_samples))
    stats=[]
    for target, values in sorted(grouped.items()):
        if len(values)<min_samples:continue
        zz=np.array(values,dtype=float)
        med=float(np.median(zz))
        mad=float(1.4826*np.median(np.abs(zz-med)))
        # High noise or moving the target does not silently improve the fit.
        stats.append({'reference_m':target,'samples':len(zz),'mean_m':float(np.mean(zz)),
                      'median_m':med,'std_m':float(np.std(zz,ddof=1)),
                      'robust_std_m':mad,'bias_at_pose_m':med-target})
    expected=([float(x) for x in required_distances] if required_distances else [])
    counts={f'{target:.3f}':len(values) for target,values in sorted(grouped.items())}
    missing=[distance for distance in expected if len(grouped.get(distance,()))<min_samples]
    report={'schema':'IntelQCopter F2 Mini range calibration v2',
            'source':str(paths[0].resolve()) if len(paths)==1 else None,
            'valid_references':len(stats),
            'input_files':[str(item.resolve()) for item in paths],
            'minimum_samples_per_reference':min_samples,
            'valid_sample_counts_by_reference_m':counts,
            'required_references_m':expected,
            'incomplete_references_m':missing,
            'invalid_frames':invalids,'per_distance':stats,
            'manufacturer_max_rate_hz':50,
            'warning':'No flight use. Valid only for verified target surface, orientation, illumination, supply and thermal conditions.'}
    if len(stats)>=3 and not missing:
        x=np.array([s['reference_m'] for s in stats]); y=np.array([s['median_m'] for s in stats])
        # Fit z=a*d+b. Use point-level medians to avoid overweighting repeated samples
        # (serial samples are autocorrelated). Nonlinear range error needs more points.
        a,b=np.polyfit(x,y,1)
        r=y-(a*x+b)
        report['affine_fit']={'measured_equals_a_times_reference_plus_b':{'a':float(a),'b_m':float(b)},
                   'corrected_distance':'(measured_m-b_m)/a',
                   'residual_rmse_m':float(np.sqrt(np.mean(r*r))),
                   'per_reference_residual_m':r.tolist(),
                   'note':'Small sample of target distances; check independent holdout range and alternative target materials before use.'}
    else:
        report['fit_warning']=(f'At least 3 references with {min_samples} valid readings each are required; '
                               'when required distances are listed, every one must meet that count.')
    base=paths[0]
    output=Path(output) if output else base.with_suffix('.lidar_report.json')
    output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report,output

class BenchApp:
    def __init__(self,port='',targets=None,samples=200,settle=3.0):
        import tkinter as tk
        from tkinter import ttk,messagebox
        self.tk=tk;self.ttk=ttk;self.msg=messagebox
        self.root=tk.Tk()
        self.root.title('IntelQCopter | Optional TOFSense F2 Mini bench calibration')
        self.root.geometry('900x570')
        self.targets=targets or DEFAULT_DISTANCES
        self.samples=int(samples)
        self.settle=float(settle)
        self.port=tk.StringVar(value=port)
        self.state=tk.StringVar(value='DISCONNECTED (optional; no LiDAR soldering required)')
        self.stage=tk.StringVar(value='Connect a 3.3V-logic USB-UART adapter; F2 Mini VCC needs 4.3–5.2V.')
        self.progress=tk.StringVar(value='0 samples')
        self.events=queue.Queue()
        self.stop=threading.Event()
        self.active_ref=None
        self.stage_index=0
        self.started_at=0.0
        self.count=0
        self.reader=None
        self.decoder=NLinkDecoder()
        self.path=ROOT/'captures'/('lidar_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.csv')
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.fp=self.path.open('x',newline='',encoding='utf-8')
        self.writer=csv.writer(self.fp)
        self.writer.writerow(CSV_HEADERS)
        top=ttk.Frame(self.root,padding=14);top.pack(fill='x')
        ttk.Label(top,text='Optional TOFSense F2 Mini calibration',font=('Segoe UI',17,'bold')).pack(anchor='w')
        ttk.Label(top,text='USB-UART bench mode only: no ESP32 firmware changes, no motors, 50 Hz maximum.',foreground='#8A4D00').pack(anchor='w')
        ttk.Label(top,text='Measure from LiDAR optical reference plane to a flat matte target; record actual distance with a ruler.').pack(anchor='w',pady=(4,0))
        row=ttk.Frame(top);row.pack(fill='x',pady=7)
        ttk.Label(row,text='LiDAR COM port: ').pack(side='left')
        ttk.Entry(row,textvariable=self.port,width=12).pack(side='left')
        ttk.Button(row,text='Connect',command=self.connect).pack(side='left',padx=7)
        ttk.Button(row,text='Start / Redo this reference',command=self.start_stage).pack(side='left',padx=7)
        ttk.Button(row,text='Skip reference',command=self.skip_stage).pack(side='left',padx=7)
        ttk.Button(row,text='Analyze saved CSV',command=self.analyze).pack(side='left',padx=7)
        ttk.Label(top,textvariable=self.state).pack(anchor='w')
        ttk.Label(top,textvariable=self.stage,font=('Segoe UI',12,'bold'),wraplength=820).pack(anchor='w',pady=8)
        ttk.Label(top,textvariable=self.progress).pack(anchor='w')
        self.bar=ttk.Progressbar(top,maximum=100,mode='determinate');self.bar.pack(fill='x',pady=6)
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        fig=Figure(figsize=(9,3.8),dpi=90)
        self.ax=fig.add_subplot(111)
        self.ax.set_title('Real-time valid distance readings');self.ax.set_xlabel('Samples');self.ax.set_ylabel('Measured range (m)');self.ax.grid(alpha=.25)
        self.line,=self.ax.plot([],[],lw=1.2)
        self.xx=[];self.yy=[]
        self.canvas=FigureCanvasTkAgg(fig,master=self.root);self.canvas.draw();self.canvas.get_tk_widget().pack(fill='both',expand=True,padx=10,pady=5)
        self.root.protocol('WM_DELETE_WINDOW',self.close)
        self.update_stage()
        self.root.after(120,self.pump)
    def update_stage(self):
        if self.stage_index>=len(self.targets):
            self.stage.set('Reference stages completed. Click Analyze saved CSV.')
        else:
            d=self.targets[self.stage_index]
            self.stage.set(f'{self.stage_index+1}/{len(self.targets)} — Put the TARGET at an accurately measured {d:.3f} m from the LiDAR. Press Start; wait {self.settle:.1f}s, then collect {self.samples} valid readings.')
    def connect(self):
        if self.reader and self.reader.is_alive():return
        try:
            import serial
            ser=serial.Serial(self.port.get(),921600,timeout=.06)
        except Exception as e:
            self.msg.showerror('LiDAR serial unavailable',str(e));return
        self.serial=ser
        def read_loop():
            try:
                while not self.stop.is_set():
                    frames=self.decoder.feed(ser.read(512))
                    for f in frames:self.events.put(('frame',f))
            except Exception as e:self.events.put(('error',str(e)))
            finally:ser.close()
        self.reader=threading.Thread(target=read_loop,daemon=True)
        self.reader.start()
        self.state.set(f'CONNECTED {self.port.get()} at 921600 baud; manufacturer 50 Hz maximum')
    def start_stage(self):
        if not self.reader or not self.reader.is_alive():
            self.msg.showwarning('Connect first','Connect the external LiDAR USB-UART first.');return
        if self.stage_index>=len(self.targets):return
        # Repeated attempt data is marked by stage index and latest start time.
        # Calibration users should select only good sessions; no silent filtering.
        self.active_ref=self.targets[self.stage_index]
        self.started_at=time.monotonic()+self.settle
        self.count=0;self.xx.clear();self.yy.clear()
        self.progress.set(f'PREPARING for {self.settle:.1f} seconds')
    def skip_stage(self):
        self.active_ref=None
        self.stage_index+=1
        self.update_stage()
    def pump(self):
        for i in range(800):
            try:k,v=self.events.get_nowait()
            except queue.Empty:break
            if k=='error':self.state.set('READ ERROR '+v);continue
            now=time.monotonic()
            if v.valid:
                self.xx.append(len(self.xx));self.yy.append(v.distance_m)
                if len(self.xx)>1500:
                    self.xx=self.xx[-800:];self.yy=self.yy[-800:]
            is_ready=self.active_ref is not None and now>=self.started_at
            if is_ready:
                row=[datetime.now(timezone.utc).isoformat(timespec='milliseconds'),f'{now:.6f}',
                     self.stage_index+1, f'{self.active_ref:.4f}',v.module_id,v.sensor_time_ms,
                     f'{v.distance_m:.4f}',v.status,v.signal_strength,v.range_precision_cm,int(v.valid)]
                self.writer.writerow(row)
                if v.valid:self.count+=1
                if self.count>=self.samples:
                    self.fp.flush();self.active_ref=None;self.stage_index+=1;self.update_stage()
            if self.active_ref is not None:
                self.progress.set(f'{self.count} / {self.samples} fresh valid readings (bad checksum={self.decoder.bad_checksum})')
                self.bar['value']=100*self.count/self.samples
        if self.active_ref is not None and time.monotonic()<self.started_at:
            self.progress.set(f'PREPARING {max(0,self.started_at-time.monotonic()):.1f}s (not counting yet)')
        if self.xx:
            self.line.set_data(self.xx,self.yy)
            self.ax.set_xlim(max(0,self.xx[-1]-500),max(501,self.xx[-1]+1))
            lo=min(self.yy[-500:]);hi=max(self.yy[-500:]);pad=max(.03,(hi-lo)*.2)
            self.ax.set_ylim(lo-pad,hi+pad)
            self.canvas.draw_idle()
        if not self.stop.is_set():self.root.after(200,self.pump)
    def analyze(self):
        self.fp.flush()
        try:
            result,path=analyze_csv(self.path)
            self.msg.showinfo('LiDAR benchmark saved',f'{path}\nCalibration distances: {result["valid_references"]}\n'
                f'Frames excluded: {result["invalid_frames"]}\n'
                'This is bench-only; inspect the report before use.')
        except Exception as e:self.msg.showerror('Analysis not ready',str(e))
    def close(self):
        self.stop.set()
        if self.reader and self.reader.is_alive():self.reader.join(timeout=1)
        self.fp.flush();self.fp.close();self.root.destroy()

def self_test():
    d=NLinkDecoder()
    x=make_test_frame(2221,sys_ms=36766)
    assert d.feed(x[:6])==[]
    out=d.feed(x[6:]);assert len(out)==1 and abs(out[0].distance_m-2.221)<1e-10
    assert out[0].valid
    bad=bytearray(x);bad[13]^=1
    assert not d.feed(bad)
    assert d.bad_checksum==1
    assert d.feed(x)[0].sensor_time_ms==36766
    print('PASS F2 Mini 16-byte frame, partial reads, checksum, resolution, and recovery')

if __name__=='__main__':
    ap=argparse.ArgumentParser()
    ap.add_argument('--gui',action='store_true')
    ap.add_argument('--port',default='')
    ap.add_argument('--analyze',type=Path,nargs='+')
    ap.add_argument('--min-samples',type=int,default=20,
                    help='Minimum accepted samples at each reference for fitting (default: 20)')
    ap.add_argument('--required-distances',type=float,nargs='*',default=None,
                    help='Require these distances in metres before producing a fit')
    ap.add_argument('--self-test',action='store_true')
    ns=ap.parse_args()
    if ns.self_test:self_test()
    elif ns.analyze:
        r,p=analyze_csv(ns.analyze,min_samples=ns.min_samples,
                        required_distances=ns.required_distances)
        print(p);print(json.dumps(r,indent=2))
    else:
        BenchApp(ns.port).root.mainloop()
