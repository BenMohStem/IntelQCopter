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

class FlightCore:
    def __init__(self,seed,companion=True):
        self.dll=c.CDLL(str(ROOT/'build/sitl/esp32_flight_core.dll'))
        self.dll.fc_init.argtypes=[c.c_uint64]
        self.dll.fc_control_step.argtypes=[c.POINTER(Sample),c.c_uint64,c.c_float,c.POINTER(Status)]
        for name in ('fc_xbox_report','fc_companion_report'):
            f=getattr(self.dll,name);f.argtypes=[c.POINTER(c.c_uint8),c.c_uint,c.c_uint64];f.restype=c.c_bool
        self.dll.fc_cal_command.argtypes=[c.c_char]
        self.dll.fc_cal_load.argtypes=[c.POINTER(c.c_uint8)];self.dll.fc_cal_load.restype=c.c_bool
        self.dll.fc_cal_take_pending.argtypes=[c.POINTER(c.c_uint8)];self.dll.fc_cal_take_pending.restype=c.c_bool
        self.dll.fc_status_frame.argtypes=[c.POINTER(Status),c.POINTER(c.c_uint8)]
        self.dll.fc_init(0x3200);self.rng=np.random.default_rng(seed);self.last_velocity=np.zeros(3);self.acc=np.zeros(3)
        self.parameters=json.loads((ROOT/'simulation/model/parameters.json').read_text())
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
        self.dll.fc_control_step(c.byref(raw),int(now*1e6)+self.time_offset_us,.002,c.byref(self.status))
        return np.array(self.status.motor_demand)
    def frame(self):
        out=(c.c_uint8*16)();self.dll.fc_status_frame(c.byref(self.status),out);return bytes(out)
