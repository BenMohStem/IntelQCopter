#!/usr/bin/env python3
"""Offline research/bench-only *linear* asynchronous vertical Kalman filter.

State x=[local_z_m, velocity_up_mps, accel_bias_mps2, baro_bias_m, terrain_z_m].
Predict at recorded timestamps and update with fresh barometer and optional
verified LiDAR observations. This script does not acquire sensors or output controls.

CSV input is the 500-Hz IntelQCopter capture, extended optionally with:
  az_world_mps2  (physically correct, attitude-rotated, gravity-removed accel)
  lidar_new, lidar_ok, lidar_range_m, roll_rad, pitch_rad, lidar_age_s
  lidar_sensor_offset_x_m, lidar_sensor_offset_y_m, lidar_sensor_offset_z_m
  Optional yaw_rad (ZYX yaw does not affect world-vertical projection).
If az_world_mps2 is absent, input is zero with large process noise; raw az_mps2
MUST NOT be used as vertical acceleration without attitude/gravity correction.

The default body-frame LiDAR offset is the user's preliminary F450 estimate
(0, 0, -0.026658) m relative to COM; override it when measurements are available.
The geometry projects to the beam hit, not to ground beneath COM. A level known
plane is useful for validation; uneven terrain needs a footprint/terrain model.
The magnetometer does not supply altitude; it may help maintain yaw attitude.
"""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
import numpy as np
G=9.80665
MOD32=2**32
PROVISIONAL_LIDAR_SENSOR_OFFSET_BODY_M=(0.0,0.0,-0.026658)

class VerticalKF:
    def __init__(self, sigma_acc=1.5, q_acc_bias=2e-5, q_baro_bias=0.002,
                 q_terrain=1e-4,baro_sigma_m=0.5,lidar_sigma_m=0.07,
                 nis_gate=10.827566):
        self.x=np.zeros(5,dtype=float)
        self.P=np.diag([1.,4.,0.2**2,1.5**2,1.]).astype(float)
        self.sigma_acc=float(sigma_acc)
        self.q_acc_bias=float(q_acc_bias)  # (m/s²)^2/s
        self.q_baro_bias=float(q_baro_bias) # m²/s
        self.q_terrain=float(q_terrain) # terrain-height random-walk variance, m²/s
        self.baro_sigma=float(baro_sigma_m)
        self.lidar_sigma=float(lidar_sigma_m)
        self.nis_gate=float(nis_gate)
        self.stats={'baro_accepted':0,'baro_rejected':0,'lidar_accepted':0,'lidar_rejected':0,
                    'timing_gaps_gt_10ms':0,'long_timing_outages':0}
        self.last_innovations=[]
        self.ground_reference_initialized=False

    def initialize_confirmed_ground_reference(self,*,stationary_confirmed,ground_confirmed):
        """Set local z and terrain datum once at a confirmed stationary ground start."""
        if not (stationary_confirmed and ground_confirmed):
            raise ValueError('Ground reference requires explicit stationary and ground confirmation')
        if self.ground_reference_initialized:
            raise RuntimeError('Ground reference is one-shot; do not re-zero in flight')
        self.x[:]=0.
        self.P=np.diag([0.05**2,0.1**2,0.2**2,1.5**2,0.05**2]).astype(float)
        self.ground_reference_initialized=True

    def _predict_piece(self,dt,a):
        # xdot=[v,a-b_a,0,0]. Exact ZOH dynamics for this linear plant.
        F=np.eye(5);F[0,1]=dt;F[0,2]=-.5*dt*dt;F[1,2]=-dt
        B=np.array([.5*dt*dt,dt,0.,0.,0.])
        self.x=F@self.x+B*a
        # Continuous white acceleration noise with spectral intensity
        # sigma_acc^2 [m²/s³] + Brownian accel-bias/pressure-height-bias.
        q=self.sigma_acc**2
        Q=np.zeros((5,5))
        Q[:2,:2]=q*np.array([[dt**3/3,dt**2/2],[dt**2/2,dt]])
        ba=self.q_acc_bias
        # exact joint covariance from white random-walk accel bias
        Q[:3,:3]+=ba*np.array([[dt**5/20,dt**4/8,-dt**3/6],
                                [dt**4/8,dt**3/3,-dt**2/2],
                                [-dt**3/6,-dt**2/2,dt]])
        Q[3,3]=self.q_baro_bias*dt
        Q[4,4]=self.q_terrain*dt
        self.P=F@self.P@F.T+Q
        self.P=.5*(self.P+self.P.T)

    def predict(self,dt,a_world_up=0.):
        if not (math.isfinite(dt) and dt>0):raise ValueError('dt must be a finite positive measured interval')
        if not math.isfinite(a_world_up):raise ValueError('accelerometer input invalid')
        if dt>.010:self.stats['timing_gaps_gt_10ms']+=1
        if dt>.50:
            self.stats['long_timing_outages']+=1
            # This uncertainty inflation is illustrative, NOT automatic flight recovery.
            self.P+=np.diag([1.,25.,0.1,0.2,1.])
        # Substeps improve numerical behavior; sum is precisely the actual elapsed time.
        count=max(1,int(math.ceil(dt/.02)))
        d=dt/count
        for _ in range(count):self._predict_piece(d,a_world_up)

    def update(self,kind,z,sigma=None):
        """Scalar Joseph update, NIS gate; return True when accepted."""
        if kind not in ('baro','lidar'):raise ValueError('kind must be baro/lidar')
        if not math.isfinite(z):return False
        H=np.array([1.,0.,0.,1.,0.]) if kind=='baro' else np.array([1.,0.,0.,0.,-1.])
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
        I=np.eye(5);A=I-np.outer(K,H)
        self.P=A@self.P@A.T+np.outer(K,K)*r # Joseph stable
        self.P=.5*(self.P+self.P.T)
        self.stats[kind+'_accepted']+=1
        return True

def relative_baro_height(pressure,p0,temperature_c=20.):
    # Locally isothermal hypsometric equation with sensor-provided temperature
    # as an APPROXIMATION to atmospheric column mean temperature.
    if min(pressure,p0)<=0:raise ValueError('Pressure must be positive')
    return 287.05*(temperature_c+273.15)/G*math.log(p0/pressure)

def _body_to_world_rotation(roll_rad,pitch_rad,yaw_rad):
    """Return Rz(yaw) Ry(pitch) Rx(roll), mapping body vectors to world."""
    angles=(roll_rad,pitch_rad,yaw_rad)
    if not all(math.isfinite(float(a)) for a in angles):
        raise ValueError('Non-finite attitude')
    cr,sr=math.cos(roll_rad),math.sin(roll_rad)
    cp,sp=math.cos(pitch_rad),math.sin(pitch_rad)
    cy,sy=math.cos(yaw_rad),math.sin(yaw_rad)
    return np.array([
        [cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
        [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
        [-sp,cp*sr,cp*cr],
    ],dtype=float)

def lidar_com_height_above_hit(slant_m,roll_rad,pitch_rad,yaw_rad=0.,
                               sensor_offset_body_m=PROVISIONAL_LIDAR_SENSOR_OFFSET_BODY_M,
                               beam_dir_body=(0.,0.,-1.),min_vertical_cos=0.85,
                               min_range_m=0.02,max_range_m=7.8):
    """Estimate COM world-vertical separation above the beam hit point.

    Uses h=-[R r_s]_z-D[R d_s]_z, where r_s is sensor position relative to
    COM, d_s is a unit beam vector in body coordinates, and R=RzRyRx maps body
    to world (+Z up). Mounting, attitude and beam vectors must use the same body
    axes. Tilt gating uses the downward component of the world-frame beam.
    A surface plane is not needed to compute the hit point, but this does not
    infer terrain beneath the COM or map the footprint over sloped terrain.
    """
    if not math.isfinite(slant_m) or not min_range_m<=slant_m<=max_range_m:
        raise ValueError('Invalid or out-of-range LiDAR measurement')
    if not math.isfinite(min_vertical_cos) or not 0.<min_vertical_cos<=1.:
        raise ValueError('min_vertical_cos must be in (0, 1]')
    r_s=np.asarray(sensor_offset_body_m,dtype=float)
    d_s=np.asarray(beam_dir_body,dtype=float)
    if r_s.shape!=(3,) or d_s.shape!=(3,) or not np.all(np.isfinite(r_s)) or not np.all(np.isfinite(d_s)):
        raise ValueError('Sensor offset and beam direction must be finite 3-vectors')
    beam_norm=float(np.linalg.norm(d_s))
    if abs(beam_norm-1.)>1e-6:
        raise ValueError('Beam direction must be a unit vector')
    R=_body_to_world_rotation(roll_rad,pitch_rad,yaw_rad)
    beam_world=R@d_s
    downward_cos=-float(beam_world[2])
    if downward_cos<min_vertical_cos:
        raise ValueError('High tilt or beam misalignment: range aid disabled')
    height=-float((R@r_s)[2])-slant_m*float(beam_world[2])
    if not math.isfinite(height):raise ValueError('Invalid projected height')
    return height

def lidar_vertical_height(slant_m,roll_rad,pitch_rad,lever_z_m=0.):
    """Compatibility wrapper; lever_z_m is COM height above a centered sensor."""
    return lidar_com_height_above_hit(
        slant_m,roll_rad,pitch_rad,yaw_rad=0.,
        sensor_offset_body_m=(0.,0.,-float(lever_z_m)))

def replay(path,output_prefix,rate_hz=100,baro_thermal_report=None,
           lidar_sensor_offset_body_m=PROVISIONAL_LIDAR_SENSOR_OFFSET_BODY_M,
           lidar_sigma_m=0.07,lidar_max_age_s=0.03,baro_reference_pa=None,
           terrain_rw_variance=1e-4):
    path=Path(path);target=Path(output_prefix);target.parent.mkdir(parents=True,exist_ok=True)
    kf=VerticalKF(lidar_sigma_m=lidar_sigma_m,q_terrain=terrain_rw_variance)
    first_p0=None;reference_samples=[];reference_method=None
    if baro_reference_pa is not None:
        first_p0=float(baro_reference_pa)
        if not math.isfinite(first_p0) or not 40000.<first_p0<115000.:
            raise ValueError('Barometer reference pressure must be within 40000..115000 Pa')
        kf.initialize_confirmed_ground_reference(stationary_confirmed=True,ground_confirmed=True)
        reference_method='explicit CLI reference; caller confirms stationary ground startup'
    thermal=None
    if baro_thermal_report is not None:
        models=json.loads(Path(baro_thermal_report).read_text(encoding='utf-8'))
        thermal=models.get('bmp_pressure_residual_model',{})
        if thermal.get('status')!='REFERENCE_BASED_PROVISIONAL':
            raise ValueError('Reject BMP thermal model: requires independent pressure-reference sessions')
        if not isinstance(thermal.get('slope_per_c'),(int,float)):
            raise ValueError('BMP thermal model must be scalar pressure slope')
    raw_rows=0;prev=None;time_us=0;prev_out_us=0;dropped_rows=0
    output=[];input_a_count=0;lidar_count=0
    last_baro_height=None;last_lidar_clearance=None;last_lidar_age=None
    with path.open(newline='',encoding='utf-8-sig') as f:
        rdr=csv.DictReader(f)
        for r in rdr:
            if r.get('kind')!='D':continue
            baro_new_this_row=False;lidar_new_this_row=False
            raw_rows+=1
            try:t=int(r['t_us'])
            except (KeyError,ValueError,TypeError):continue
            try:p=float(r.get('pressure_pa',''))
            except (ValueError,TypeError):p=float('nan')
            try:temp=float(r.get('bmp_temp_c',''))
            except (ValueError,TypeError):temp=float('nan')
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
                if first_p0 is None and r.get('ground_startup_confirmed')=='1' and r.get('stationary')=='1':
                    reference_samples.append(p)
                    if len(reference_samples)>=30:
                        first_p0=float(np.median(reference_samples))
                        kf.initialize_confirmed_ground_reference(
                            stationary_confirmed=True,ground_confirmed=True)
                        reference_method='median of 30+ fresh BMP samples with ground_startup_confirmed=1 and stationary=1'
                if first_p0 is not None:
                    last_baro_height=relative_baro_height(p,first_p0,temp if math.isfinite(temp) else 20.)
                    baro_new_this_row=True
                    kf.update('baro',last_baro_height)
            if r.get('lidar_new')=='1' and r.get('lidar_ok')=='1':
                # Use these fields only after pinning and validating the source
                # firmware and independent LiDAR logger that produced them.
                try:
                    dist=float(r['lidar_range_m']);roll=float(r['roll_rad']);pitch=float(r['pitch_rad'])
                    if 'lidar_age_s' not in r:raise ValueError('LiDAR timestamp age is required')
                    age=float(r['lidar_age_s'])
                    if not 0<=age<=lidar_max_age_s:raise ValueError('stale lidar sample')
                    offset_keys=('lidar_sensor_offset_x_m','lidar_sensor_offset_y_m','lidar_sensor_offset_z_m')
                    if all(k in r and r[k]!='' for k in offset_keys):
                        offset=tuple(float(r[k]) for k in offset_keys)
                    elif any(r.get(k,'')!='' for k in offset_keys):
                        raise ValueError('LiDAR sensor offset requires all three body axes')
                    else:
                        offset=lidar_sensor_offset_body_m
                    yaw=float(r['yaw_rad']) if r.get('yaw_rad','')!='' else 0.
                    correction=lidar_com_height_above_hit(
                        dist,roll,pitch,yaw,offset,min_range_m=0.02,max_range_m=7.8)
                    last_lidar_clearance=correction;last_lidar_age=age;lidar_new_this_row=True
                    lidar_count+=1;kf.update('lidar',correction)
                except (ValueError,TypeError,KeyError):kf.stats['lidar_rejected']+=1
            if time_us-prev_out_us>=1e6/max(rate_hz,1):
                prev_out_us=time_us
                clearance_var=max(0.,kf.P[0,0]+kf.P[4,4]-2.*kf.P[0,4])
                output.append((time_us/1e6,last_baro_height,int(baro_new_this_row),
                               last_lidar_clearance,int(lidar_new_this_row),last_lidar_age,
                               kf.x[0],kf.x[1],kf.x[2],kf.x[3],kf.x[4],
                               kf.x[0]-kf.x[4],float(math.sqrt(max(0,kf.P[0,0]))),
                               float(math.sqrt(max(0,kf.P[4,4]))),float(math.sqrt(clearance_var))))
    o_csv=Path(str(target)+'_altitude.csv');o_json=Path(str(target)+'_altitude.json')
    with o_csv.open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['t_s','baro_relative_height_m','baro_measurement_new',
            'lidar_clearance_measurement_m','lidar_measurement_new','lidar_sample_age_s',
            'local_z_m','vertical_speed_mps','accel_bias_mps2','baro_bias_m','terrain_z_m',
            'lidar_clearance_est_m','sigma_local_z_m','sigma_terrain_z_m','sigma_lidar_clearance_m'])
        w.writerows(output)
    report={'schema':'IntelQCopter offline terrain-separated vertical KF v2','input_csv':str(path),'rows':raw_rows,
            'samples_with_verified_world_up_acceleration':input_a_count,'lidar_returns_seen':lidar_count,
            'lidar_geometry':{
                'sensor_offset_body_m_default':list(lidar_sensor_offset_body_m),
                'offset_status':'USER_REPORTED_PRELIMINARY; verify with physical measurements',
                'beam_direction_body':[0.0,0.0,-1.0],
                'rotation':'Rz(yaw) Ry(pitch) Rx(roll), body to world, +Z up',
                'min_range_m':0.02,'max_range_m':7.8,'max_sample_age_s':lidar_max_age_s,
                'minimum_downward_beam_cosine':0.85,
                'measurement_sigma_m_assumed':lidar_sigma_m,
                'terrain_random_walk_variance_m2_per_s_assumed':terrain_rw_variance,
            },
            'vertical_reference':{
                'barometer_reference_pa':first_p0,
                'method':reference_method,
                'initialized_from_confirmed_ground':kf.ground_reference_initialized,
                'candidate_reference_samples':len(reference_samples),
            },
            'filter_statistics':kf.stats,
            'bmp_reference_based_thermal_model_file':str(baro_thermal_report) if thermal else None,
            'limitations':[
              'OFFLINE algorithm prototype only: no flight actuator output.',
              'The replay does not validate the source, driver, wiring, or timestamps behind optional LiDAR columns.',
              'LiDAR offset and constant range sigma are preliminary defaults, not measured calibration uncertainty.',
              'The measurement is relative to the beam hit; footprint motion and terrain slope are not mapped back beneath the COM.',
              'Without verified world-up acceleration, uses a=0 and uncertain process noise; not an inertial altitude predictor.',
              'The filter keeps local vertical position, terrain height, and LiDAR terrain clearance distinct; terrain process noise and sensor sigmas are provisional.',
              'A confirmed ground reference requires either an explicit pressure baseline or 30 fresh pressure samples tagged stationary and ground_startup_confirmed; no automatic midair re-zero occurs.',
              'Without a confirmed ground reference, barometer data are not used to assert local altitude.',
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
    ap.add_argument('--lidar-offset-body-m',type=float,nargs=3,metavar=('X','Y','Z'),
                    default=PROVISIONAL_LIDAR_SENSOR_OFFSET_BODY_M,
                    help='Sensor position relative to COM in body metres (default is preliminary user estimate)')
    ap.add_argument('--lidar-sigma-m',type=float,default=0.07,
                    help='Assumed scalar LiDAR measurement sigma in metres for offline innovation gating')
    ap.add_argument('--lidar-max-age-s',type=float,default=0.03,
                    help='Maximum accepted logged LiDAR sample age')
    ap.add_argument('--baro-reference-pa',type=float,default=None,
                    help='Known stationary ground pressure reference in Pa; supplying it asserts confirmed ground startup')
    ap.add_argument('--terrain-rw-variance',type=float,default=1e-4,
                    help='Assumed terrain-height random-walk variance in m^2/s')
    ns=ap.parse_args()
    csvout,report=replay(ns.csv,ns.output_prefix or ns.csv.with_suffix(''),ns.plot_rate_hz,
                         ns.baro_thermal_report,tuple(ns.lidar_offset_body_m),
                         ns.lidar_sigma_m,ns.lidar_max_age_s,ns.baro_reference_pa,
                         ns.terrain_rw_variance)
    print('Saved',csvout,report)
    print('No LiDAR was used unless your input CSV has verified lidar_new/lidar_ok/range/attitude columns.')

if __name__=='__main__':main()
