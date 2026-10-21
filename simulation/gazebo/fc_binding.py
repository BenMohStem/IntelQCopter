"""Sensor-side binding to the actual ESP32 C core; pilot input is raw HID only."""
import ctypes as c
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
class Sample(c.Structure):
    _fields_=[('accel_g',c.c_float*3),('gyro_dps',c.c_float*3),('pressure_pa',c.c_float),('pressure_sequence',c.c_uint32),('imu_valid',c.c_bool),('pressure_valid',c.c_bool)]
class Status(c.Structure):
    _fields_=[('motor_demand',c.c_float*4),('roll_deg',c.c_float),('pitch_deg',c.c_float),('height_m',c.c_float),('vertical_mps',c.c_float),('gyro_sigma_dps',c.c_float*3),
       ('sensor_fault',c.c_bool),('pilot_fresh',c.c_bool),('gyro_verified',c.c_bool),('accel_calibrated',c.c_bool),('barometer_ready',c.c_bool),('battery_warning',c.c_bool),('battery_disarm',c.c_bool),
       ('faces_done',c.c_uint8),('capturing_face',c.c_int8),('flight_mode',c.c_uint8),('user_disarm',c.c_bool),('companion_fresh',c.c_bool),('xbox_reports',c.c_uint32),('mode_events',c.c_uint32)]

class Debug(c.Structure):
 _fields_=[(n,c.c_uint32) for n in 'steps hid_rejected uart_rejected invalid_calls sensor_faults stale_pilot stale_companion saturation_cycles torque_scaled_cycles numeric_faults deadline_misses max_execution_us'.split()]+[('execution_bins',c.c_uint32*8)]+[(n,c.c_float) for n in 'max_dt_s collective_requested collective_allocated collective_ceiling torque_scale motor_min motor_max roll_nis pitch_nis height_nis min_covariance'.split()]+[('trace_written',c.c_uint32),('level',c.c_uint8),('gentle',c.c_uint8)]

class FlightCore:
    def __init__(self,seed,companion=True,settings=None,dll_path=None):
        self.dll=c.CDLL(str(dll_path or ROOT/'build/sitl/esp32_flight_core.dll'))
        self.dll.fc_init.argtypes=[c.c_uint64]
        self.dll.fc_control_step.argtypes=[c.POINTER(Sample),c.c_uint64,c.c_float,c.POINTER(Status)]
        for name in ('fc_xbox_report','fc_companion_report'):
            f=getattr(self.dll,name);f.argtypes=[c.POINTER(c.c_uint8),c.c_uint,c.c_uint64];f.restype=c.c_bool
        self.dll.fc_cal_command.argtypes=[c.c_char]
        self.dll.fc_cal_load.argtypes=[c.POINTER(c.c_uint8)];self.dll.fc_cal_load.restype=c.c_bool
        self.dll.fc_cal_take_pending.argtypes=[c.POINTER(c.c_uint8)];self.dll.fc_cal_take_pending.restype=c.c_bool
        self.dll.fc_status_frame.argtypes=[c.POINTER(Status),c.POINTER(c.c_uint8)]
        self.dll.fc_horizontal_observation.argtypes=[c.POINTER(c.c_float),c.c_uint64];self.dll.fc_horizontal_observation.restype=c.c_bool
        self.dll.fc_init(0x3200);self.rng=np.random.default_rng(seed);self.last_velocity=np.zeros(3);self.acc=np.zeros(3)
        self.parameters=settings['model'] if settings else json.loads((ROOT/'simulation/model/parameters.json').read_text())
        self.status=Status();self.pressure_sequence=0;self.tick=0;self.pressure=101325.
        sample=Sample();sample.imu_valid=sample.pressure_valid=True;sample.pressure_pa=101325.;sample.accel_g[2]=1
        def bench(n):
            for _ in range(n):
                self.tick+=1
                if self.tick%10==0:sample.pressure_sequence+=1
                self.dll.fc_control_step(c.byref(sample),self.tick*2000,.002,c.byref(self.status))
        bench(1100)
        for f in range(6):
            sample.accel_g[:]=[0,0,0];sample.accel_g[f//2]=-1 if f%2 else 1
            self.dll.fc_cal_command(str(f).encode());bench(200)
        record=(c.c_uint8*160)();self.dll.fc_cal_command(b'S');assert self.dll.fc_cal_take_pending(record)
        self.dll.fc_init(0x3200);assert self.dll.fc_cal_load(record)
        sample.accel_g[:]=[0,0,1];bench(1100)
        assert self.status.gyro_verified and self.status.accel_calibrated and self.status.barometer_ready
        self.dll.fc_configure_companion.argtypes=[c.c_bool]
        self.dll.fc_configure_companion(companion)
        self.dll.fc_configure_gentle.argtypes=[c.c_bool];self.dll.fc_configure_gentle.restype=c.c_bool
        assert self.dll.fc_configure_gentle(False) # UI shapes raw HID once; test pilots use explicitly full-scale decoded inputs.
        self.dll.fc_debug_configure.argtypes=[c.c_uint];self.dll.fc_debug_configure.restype=c.c_bool
        assert self.dll.fc_debug_configure(3)
        if settings:
            from experiment_settings import defaults,fc_values
            if settings['fc']!=defaults()['fc']:
                values=fc_values(settings['fc']);buf=(c.c_float*len(values))(*values)
                self.dll.fc_sim_configure.argtypes=[c.POINTER(c.c_float),c.c_uint];self.dll.fc_sim_configure.restype=c.c_bool
                if not self.dll.fc_sim_configure(buf,len(values)):raise ValueError('C core rejected boot-only tuning')
        self.dll.fc_debug_snapshot.argtypes=[c.POINTER(Debug)]
        self.dll.fc_sim_observe.argtypes=[c.POINTER(c.c_float),c.c_uint];self.dll.fc_sim_observe.restype=c.c_uint
        self.tick=0;self.time_offset_us=20_000_000
        print('Synthetic sensor bench calibration complete; not physical calibration.',flush=True)
    def report(self,data,companion,now):
        buf=(c.c_uint8*len(data)).from_buffer_copy(bytes(data))
        return bool((self.dll.fc_companion_report if companion else self.dll.fc_xbox_report)(buf,len(data),int(now*1e6)+self.time_offset_us))
    def step(self,state,rotation):
        self.tick+=1;now=state['sim_time_s'];velocity=np.array(state['velocity'])
        measured=(velocity-self.last_velocity)/.002;self.last_velocity=velocity
        self.acc=.25*measured+.75*self.acc
        raw=Sample();raw.imu_valid=raw.pressure_valid=True
        proper=rotation.T@(np.clip(self.acc,-20,20)+[0,0,9.80665])/9.80665
        raw.accel_g[:]=(proper+self.rng.normal(0,self.parameters['accel_sigma_g'],3)).tolist()
        raw.gyro_dps[:]=(rotation.T@np.array(state['omega'])*180/np.pi+self.rng.normal(0,self.parameters['gyro_sigma_dps'],3)).tolist()
        if self.tick%10==0:
            self.pressure_sequence+=1
            h=max(-1,state['position'][2]-.082)+self.rng.normal(0,self.parameters['barometer_sigma_m'])
            self.pressure=101325*(1-h/44330)**(1/.190294957)
        raw.pressure_sequence=self.pressure_sequence;raw.pressure_pa=self.pressure
        # Explicit synthetic localization SENSOR input. The C FC computes all
        # braking/hold angles; Python never alters the aircraft pose or velocity.
        if not getattr(self,'disable_horizontal_aiding',False):
            q=state['quaternion'] if 'quaternion' in state else None
            yaw=float(np.arctan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))) if q else 0.
            obs=(c.c_float*5)(state['position'][0],state['position'][1],velocity[0],velocity[1],yaw)
            assert self.dll.fc_horizontal_observation(obs,int(now*1e6)+self.time_offset_us)
        self.dll.fc_control_step(c.byref(raw),int(now*1e6)+self.time_offset_us,.002,c.byref(self.status))
        return np.array(self.status.motor_demand)
    def observe(self):
        keys=['roll_est','pitch_est','height_est','vz_est','roll_sigma','pitch_sigma','height_sigma','vz_sigma','roll_K','pitch_K','height_K','velocity_K','roll_innovation','pitch_innovation','height_innovation','height_S','roll_bias','pitch_bias','roll_command','pitch_command','yaw_rate_command','vz_command','fc_calls','xy_hold_active','xy_aiding_fresh','xy_target_x','xy_target_y','xy_x','xy_y','xy_vx','xy_vy','xy_acceleration_norm','angle_gravity_correction']
        out=(c.c_float*33)();assert self.dll.fc_sim_observe(out,33)==33
        result=dict(zip(keys,map(float,out)));d=Debug();self.dll.fc_debug_snapshot(c.byref(d))
        result.update({'debug_'+name:float(getattr(d,name)) for name,_ in Debug._fields_ if name!='execution_bins'})
        return result
    def frame(self):
        out=(c.c_uint8*16)();self.dll.fc_status_frame(c.byref(self.status),out);return bytes(out)
