"""Validated simulation experiments. Defaults preserve the existing simulator."""
import copy,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
PID_NAMES=['roll_angle','pitch_angle','roll_rate','pitch_rate','yaw_rate','altitude']
PID_DEFAULTS=[[3,0,0,0],[3,0,0,0],[.000829338,.003870245,0,15],[.000829338,.003870245,0,15],[.005375614,.008362066,0,8],[.108045,.03087,.1225,2]]
ROS_DEFAULTS=dict(position_gain=1.,velocity_gain=1.8,max_speed_mps=.8,max_acceleration_mps2=1.4,max_angle_deg=15.,heading_gain=1.2,max_yaw_rate_dps=45.)
def defaults():
 return dict(schema=1,model=json.loads((ROOT/'simulation/model/parameters.json').read_text()),fc=dict(pids=dict(zip(PID_NAMES,copy.deepcopy(PID_DEFAULTS))),angle_kf=[.04,.0001,4.],altitude_kf=[1.5,2.],hover_collective=.5),ros=copy.deepcopy(ROS_DEFAULTS))
def fields():
 result=[]
 def field(path,label,unit,lo,hi,step,group,help=''):result.append(dict(path=path,label=label,unit=unit,min=lo,max=hi,step=step,group=group,help=help))
 for name in PID_NAMES:
  for i,key in enumerate(['Kp','Ki','Kd','integral limit']):
   hi=1000 if i==3 else 20 if name.endswith('angle') else 5 if name=='altitude' else 1
   field(['fc','pids',name,i],name+' '+key,'see units in guide',0,hi,.000001,'FC / six PID loops','Integral limit zero disables accumulated integral, even if Ki is nonzero.')
 for path,label,unit,lo,hi,step in [(['angle_kf',0],'Gyro measurement variance','(deg/s)²',.000001,100,.001),(['angle_kf',1],'Gyro bias random-walk density','(deg/s)²/s',.00000001,10,.00001),(['angle_kf',2],'Accelerometer angle variance','deg²',.000001,1000,.01),(['altitude_kf',0],'Vertical acceleration sigma','m/s²',.00001,30,.01),(['altitude_kf',1],'Barometer height variance','m²',.000001,100,.01)]:field(['fc',*path],label,unit,lo,hi,step,'FC / Kalman filters')
 field(['fc','hover_collective'],'Nominal hover collective','0..1',.1,.8,.001,'FC / collective')
 for key,label,unit,lo,hi,step in [('mass_kg','Complete mass','kg',.1,10,.01),('motor_diagonal_m','Motor diagonal','m',.1,1,.001),('maximum_thrust_per_motor_n','Maximum thrust per motor','N',.1,50,.1),('thrust_time_constant_s','Thrust lag','s',.001,1,.001),('linear_drag_ns_per_m','Linear drag','N·s/m',0,5,.01),('reaction_torque_per_thrust_m','Reaction torque/thrust','m',.001,.1,.001),('gyro_sigma_dps','Synthetic gyro sigma','deg/s',.00001,10,.001),('accel_sigma_g','Synthetic accel sigma','g',.000001,1,.0001),('barometer_sigma_m','Synthetic barometer sigma','m',.00001,10,.001)]:field(['model',key],label,unit,lo,hi,step,'Simulation / plant and sensors','Assumed simulation parameter; editing it is not a hardware measurement.')
 for i,axis in enumerate('xyz'):field(['model','inertia_kg_m2',i],'Inertia J'+axis,'kg·m²',.00001,1,.0001,'Simulation / plant and sensors')
 for key,value in ROS_DEFAULTS.items():field(['ros',key],key.replace('_',' '),({'position_gain':'1/s','velocity_gain':'1/s','max_speed_mps':'m/s','max_acceleration_mps2':'m/s²','max_angle_deg':'degrees','heading_gain':'1/s','max_yaw_rate_dps':'degrees/s'}[key]),.001,10 if 'speed' in key else 45 if 'angle' in key else 180 if 'yaw_rate' in key else 20,.01,'ROS2 / navigator')
 return result
def value_at(data,path):
 for k in path:data=data[k]
 return data
def validate(data):
 base=defaults()
 if not isinstance(data,dict) or set(data)!=set(base) or data.get('schema')!=1:raise ValueError('Expected experiment schema 1: model, fc and ros')
 # Only catalogued values may differ; timings, motor arming and acquisition layout stay fixed.
 result=copy.deepcopy(base)
 for f in fields():
  try:v=value_at(data,f['path'])
  except (KeyError,TypeError,IndexError):raise ValueError('Missing '+f['label'])
  if isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or not f['min']<=v<=f['max']:raise ValueError(f"{f['label']}: finite value {f['min']}..{f['max']} required")
  target=result
  for k in f['path'][:-1]:target=target[k]
  target[f['path'][-1]]=float(v)
 # Reject unknown or silently modified non-editable values.
 if result!=data:raise ValueError('Unknown or non-editable settings changed; use the current catalog')
 j=result['model']['inertia_kg_m2']
 if any(j[i]>sum(j)-j[i]+1e-10 for i in range(3)):raise ValueError('Inertias must satisfy Jx+Jy≥Jz and cyclic permutations')
 return result
def load(path):return validate(json.loads(Path(path).read_text(encoding='utf8')))
def fc_values(fc):return [v for n in PID_NAMES for v in fc['pids'][n]]+fc['angle_kf']+fc['altitude_kf']+[fc['hover_collective']]
