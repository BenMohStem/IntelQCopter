#!/usr/bin/env python3
"""Offline research/bench-only *linear* asynchronous vertical Kalman filter.

State x=[height_up_m, velocity_up_mps, accel_bias_mps2, baro_height_bias_m].
Predict at recorded timestamps and update with NEW barometer and optional
future lidar observations; no flight-control outputs, no ToF driver in firmware.

CSV input is the 500-Hz IntelQCopter capture, extended optionally with:
  az_world_mps2  (physically correct, attitude-rotated, gravity-removed accel)
  lidar_new, lidar_ok, lidar_range_m, roll_rad, pitch_rad, lidar_age_s
  lidar_sensor_offset_z_m (optional, body mounting translation, verify sign)
If az_world_mps2 is absent, input is zero with large process noise; raw az_mps2
MUST NOT be used as vertical acceleration without attitude/gravity correction.

Only use ranges against a level, known plane. The magnetometer does not supply
altitude; it may help maintain yaw attitude, but is not an altitude observation.
"""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
import numpy as np
G=9.80665
MOD32=2**32

class VerticalKF:
    def __init__(self, sigma_acc=1.5, q_acc_bias=2e-5, q_baro_bias=0.002,
                 baro_sigma_m=0.5, lidar_sigma_m=0.07, nis_gate=10.827566):
        self.x=np.zeros(4,dtype=float)
        self.P=np.diag([1.,4.,0.2**2,1.5**2]).astype(float)
        self.sigma_acc=float(sigma_acc)
        self.q_acc_bias=float(q_acc_bias)  # (m/s²)^2/s
        self.q_baro_bias=float(q_baro_bias) # m²/s
        self.baro_sigma=float(baro_sigma_m)
        self.lidar_sigma=float(lidar_sigma_m)
        self.nis_gate=float(nis_gate)
        self.stats={'baro_accepted':0,'baro_rejected':0,'lidar_accepted':0,'lidar_rejected':0,
                    'timing_gaps_gt_10ms':0,'long_timing_outages':0}
        self.last_innovations=[]

    def _predict_piece(self,dt,a):
        # xdot=[v,a-b_a,0,0]. Exact ZOH dynamics for this linear plant.
        F=np.eye(4);F[0,1]=dt;F[0,2]=-.5*dt*dt;F[1,2]=-dt
        B=np.array([.5*dt*dt,dt,0.,0.])
        self.x=F@self.x+B*a
        # Continuous white acceleration noise with spectral intensity
        # sigma_acc^2 [m²/s³] + Brownian accel-bias/pressure-height-bias.
        q=self.sigma_acc**2
        Q=np.zeros((4,4))
        Q[:2,:2]=q*np.array([[dt**3/3,dt**2/2],[dt**2/2,dt]])
        ba=self.q_acc_bias
        # exact joint covariance from white random-walk accel bias
        Q[:3,:3]+=ba*np.array([[dt**5/20,dt**4/8,-dt**3/6],
                                [dt**4/8,dt**3/3,-dt**2/2],
                                [-dt**3/6,-dt**2/2,dt]])
        Q[3,3]=self.q_baro_bias*dt
        self.P=F@self.P@F.T+Q
        self.P=.5*(self.P+self.P.T)

    def predict(self,dt,a_world_up=0.):
        if not (math.isfinite(dt) and dt>0):raise ValueError('dt must be a finite positive measured interval')
        if not math.isfinite(a_world_up):raise ValueError('accelerometer input invalid')
        if dt>.010:self.stats['timing_gaps_gt_10ms']+=1
        if dt>.50:
            self.stats['long_timing_outages']+=1
            # This uncertainty inflation is illustrative, NOT automatic flight recovery.
            self.P+=np.diag([1.,25.,0.1,0.2])
        # Substeps improve numerical behavior; sum is precisely the actual elapsed time.
        count=max(1,int(math.ceil(dt/.02)))
        d=dt/count
        for _ in range(count):self._predict_piece(d,a_world_up)

    def update(self,kind,z,sigma=None):
        """Scalar Joseph update, NIS gate; return True when accepted."""
        if kind not in ('baro','lidar'):raise ValueError('kind must be baro/lidar')
        if not math.isfinite(z):return False
        H=np.array([1.,0.,0.,1.]) if kind=='baro' else np.array([1.,0.,0.,0.])
        r=float((self.baro_sigma if kind=='baro' else self.lidar_sigma) if sigma is None else sigma)**2
        if not math.isfinite(r) or r<=0:raise ValueError('measurement sigma must be positive')
        innovation=float(z-H@self.x)
        innovation_variance=float(H@self.P@H+r)
        nis=innovation*innovation/innovation_variance
        self.last_innovations.append((kind,innovation,nis))
        if len(self.last_innovations)>3000:self.last_innovations=self.last_innovations[-1000:]
        if nis>self.nis_gate:
            self.stats[kind+'_rejected']+=1
            return False
        K=(self.P@H)/innovation_variance
        self.x=self.x+K*innovation
        I=np.eye(4);A=I-np.outer(K,H)
        self.P=A@self.P@A.T+np.outer(K,K)*r # Joseph stable
        self.P=.5*(self.P+self.P.T)
        self.stats[kind+'_accepted']+=1
        return True

def relative_baro_height(pressure,p0,temperature_c=20.):
    # Locally isothermal hypsometric equation with sensor-provided temperature
    # as an APPROXIMATION to atmospheric column mean temperature.
    if min(pressure,p0)<=0:raise ValueError('Pressure must be positive')
    return 287.05*(temperature_c+273.15)/G*math.log(p0/pressure)

def lidar_vertical_height(slant_m,roll_rad,pitch_rad,lever_z_m=0.):
    """Flat horizontal target plane, downward-aligned beam, pure RzRyRx roll/pitch.

    roll/pitch MUST be verified sensor-frame-to-body aligned radians. A vertical
    lever arm can be added if accurately surveyed; assume 0 otherwise.
    """
    if not math.isfinite(slant_m) or slant_m<=0:raise ValueError('Invalid range')
    cos_t=math.cos(roll_rad)*math.cos(pitch_rad)
    if cos_t<0.85:raise ValueError('High tilt: flat-plane range aid disabled')
    return slant_m*cos_t+lever_z_m

def replay(path,output_prefix,rate_hz=100,baro_thermal_report=None):
    path=Path(path);target=Path(output_prefix);target.parent.mkdir(parents=True,exist_ok=True)
    kf=VerticalKF()
    thermal=None
    if baro_thermal_report is not None:
        models=json.loads(Path(baro_thermal_report).read_text(encoding='utf-8'))
        thermal=models.get('bmp_pressure_residual_model',{})
        if thermal.get('status')!='REFERENCE_BASED_PROVISIONAL':
            raise ValueError('Reject BMP thermal model: requires independent pressure-reference sessions')
        if not isinstance(thermal.get('slope_per_c'),(int,float)):
            raise ValueError('BMP thermal model must be scalar pressure slope')
    raw_rows=0;p0=[];first_p0=None;prev=None;time_us=0;prev_out_us=0;dropped_rows=0
    output=[];input_a_count=0;lidar_count=0
    with path.open(newline='',encoding='utf-8-sig') as f:
        rdr=csv.DictReader(f)
        for r in rdr:
            if r.get('kind')!='D':continue
            raw_rows+=1
            try:t=int(r['t_us']);p=float(r['pressure_pa']);temp=float(r.get('bmp_temp_c',20))
            except (KeyError,ValueError,TypeError):continue
            if prev is not None:
                du=(t-prev)&0xFFFFFFFF
                if not 0<du<5_000_000:continue
                time_us+=du
                a_str=r.get('az_world_mps2','')
                try:
                    a=float(a_str)
                    if not math.isfinite(a):raise ValueError()
                    input_a_count+=1
                except (ValueError,TypeError):a=0.
                kf.predict(du*1.e-6,a)
            prev=t
            if r.get('bmp_ok')=='1' and r.get('bmp_new')=='1' and math.isfinite(p) and 40000<p<115000:
                if thermal is not None:
                    if not math.isfinite(temp):
                        kf.stats['baro_rejected']+=1
                        continue
                    if not thermal['temperature_min_c'] <= temp <= thermal['temperature_max_c']:
                        # No extrapolation of thermal corrections beyond measured range.
                        kf.stats['baro_rejected']+=1
                        continue
                    p -= thermal['intercept']+thermal['slope_per_c']*(temp-thermal['temperature_ref_c'])
                    if not 40000<p<115000:
                        kf.stats['baro_rejected']+=1
                        continue
                if first_p0 is None:
                    p0.append(p)
                    if len(p0)>=30:first_p0=float(np.median(p0))
                if first_p0 is not None:
                    kf.update('baro',relative_baro_height(p,first_p0,temp if math.isfinite(temp) else 20.))
            if r.get('lidar_new')=='1' and r.get('lidar_ok')=='1':
                # No such columns in the existing firmware: designed for a FUTURE
                # independently logged and validated ToF sensor stream only.
                try:
                    dist=float(r['lidar_range_m']);roll=float(r['roll_rad']);pitch=float(r['pitch_rad'])
                    age=float(r.get('lidar_age_s',0) or 0)
                    if not 0<=age<=.03:raise ValueError('stale lidar sample')
                    correction=lidar_vertical_height(dist,roll,pitch)
                    if .03<=dist<=7.8:
                        lidar_count+=1;kf.update('lidar',correction)
                except (ValueError,TypeError,KeyError):kf.stats['lidar_rejected']+=1
            if time_us-prev_out_us>=1e6/max(rate_hz,1):
                prev_out_us=time_us
                output.append((time_us/1e6,*kf.x,float(math.sqrt(max(0,kf.P[0,0])))))
    o_csv=Path(str(target)+'_altitude.csv');o_json=Path(str(target)+'_altitude.json')
    with o_csv.open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['t_s','h_est_m','vertical_speed_mps','accel_bias_mps2','baro_bias_m','sigma_h_m'])
        w.writerows(output)
    report={'schema':'IntelQCopter offline linear vertical KF v1','input_csv':str(path),'rows':raw_rows,
            'samples_with_verified_world_up_acceleration':input_a_count,'lidar_returns_seen':lidar_count,
            'barometer_reference_pa':first_p0,'filter_statistics':kf.stats,
            'bmp_reference_based_thermal_model_file':str(baro_thermal_report) if thermal else None,
            'limitations':[
              'OFFLINE algorithm prototype only: no flight actuator output.',
              'No lidar in calibration firmware; lidar columns optional for future physical dataset.',
              'Without verified world-up acceleration, uses a=0 and uncertain process noise; not an inertial altitude predictor.',
              'Without independent lidar/height reference, barometer bias and absolute altitude offset are unobservable.',
              'LiDAR measures height ABOVE LOCAL GROUND, not a fixed inertial height over obstacles/slopes.',
              'Sensor sample times/latencies require synchronization; current CSV timestamps are MCU read-call times.',
              'The state is locally LINEAR conditional on independently established attitude/geometry.'
            ]}
    o_json.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    return o_csv,o_json

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv',type=Path)
    ap.add_argument('-o','--output-prefix',type=Path,default=None)
    ap.add_argument('--plot-rate-hz',type=float,default=100.)
    ap.add_argument('--baro-thermal-report',type=Path,help='Validated reference-based pressure residual model (experimental)')
    ns=ap.parse_args()
    csvout,report=replay(ns.csv,ns.output_prefix or ns.csv.with_suffix(''),ns.plot_rate_hz,ns.baro_thermal_report)
    print('Saved',csvout,report)
    print('No LiDAR was used unless your input CSV has verified lidar_new/lidar_ok/range/attitude columns.')

if __name__=='__main__':main()
