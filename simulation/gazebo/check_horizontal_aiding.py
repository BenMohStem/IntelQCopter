"""C API validation/freshness checks. Synthetic sensor bench, no flight physics."""
import ctypes as c,math,json
import numpy as np
from fc_binding import FlightCore
def main():
 core=FlightCore(3200,companion=False);checks={};neutral=bytes.fromhex('00800080008000800000000000000000');base=core.time_offset_us
 def tick(i):
  now=i*.002;core.report(neutral,False,now);core.step(dict(sim_time_s=now,position=[0,0,1.1],velocity=[0,0,0],omega=[0,0,0],quaternion=[0,0,0,1]),np.eye(3))
 for i in range(1,1001):tick(i)
 checks['fresh_localization_enables_hold']=core.observe()['xy_hold_active']==1
 now=base+2_000_000;obs=(c.c_float*5)(0,0,0,0,0)
 checks['out_of_order_observation_rejected']=not core.dll.fc_horizontal_observation(obs,now-1000)
 obs[0]=float('nan');checks['NaN_revokes_aiding']=not core.dll.fc_horizontal_observation(obs,now+1)
 core.disable_horizontal_aiding=True;tick(1001);checks['invalid_aiding_clears_hold']=core.observe()['xy_hold_active']==0
 obs[0]=0;checks['finite_observation_accepted']=bool(core.dll.fc_horizontal_observation(obs,now+4000))
 for i in range(1002,1007):tick(i)
 checks['fresh_again_recaptures_hold']=core.observe()['xy_hold_active']==1
 for i in range(1007,1060):tick(i)
 st=core.observe();checks['older_than_100ms_expires']=st['xy_aiding_fresh']==0 and st['xy_hold_active']==0
 checks['expiry_returns_to_neutral_angle_commands']=abs(st['roll_command'])<1e-6 and abs(st['pitch_command'])<1e-6
 checks['NULL_revokes_aiding']=not core.dll.fc_horizontal_observation(None,now+200000)
 result=dict(passed=all(checks.values()),checks=checks,scope=__doc__);print(json.dumps(result,indent=2));assert result['passed']
if __name__=='__main__':main()
