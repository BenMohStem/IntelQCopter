"""Real ROS 2 DDS round trips to production flight C, with CPU plant.

Two Python adapter nodes in one process, UDP DDS, Float32MultiArray test envelopes.
This tests neither the unbuilt ROS C++ wrapper nor a physical UART nor Isaac physics.
"""
import ros2_probe  # sets isolated bundled runtime and writable logs
import sys,time,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent/'sitl'))
import numpy as np
import rclpy
from std_msgs.msg import Float32MultiArray
from run import libraries,run_case,ROOT

class DdsFlight:
    def __init__(self,core):
        self.core=core;self.seq=0;self.result=None;self.latencies=[]
        self.plant=rclpy.create_node('qcopter_cpu_plant')
        self.bridge=rclpy.create_node('qcopter_c_flight_bridge')
        self.request=self.plant.create_publisher(Float32MultiArray,'sitl/sensor_command',10)
        self.response=self.bridge.create_publisher(Float32MultiArray,'sitl/motor_estimator',10)
        self.sub=self.bridge.create_subscription(Float32MultiArray,'sitl/sensor_command',self.control,10)
        self.reply=self.plant.create_subscription(Float32MultiArray,'sitl/motor_estimator',self.receive,10)
    def control(self,msg):
        a=np.array(msg.data,np.float32);mot=np.zeros(4,np.float32);diag=np.zeros(16,np.float32)
        self.core.qc_sitl_step(a[1:10].copy(),a[10:14].copy(),float(a[14]),int(a[15]),mot,diag)
        self.response.publish(Float32MultiArray(data=np.r_[a[0],mot,diag].tolist()))
    def receive(self,msg):self.result=np.array(msg.data,np.float32)
    def qc_sitl_reset(self,*args):self.core.qc_sitl_reset(*args)
    def qc_sitl_step(self,sensor,command,dt,flags,motors,diag):
        self.seq+=1;self.result=None;start=time.perf_counter()
        message=Float32MultiArray(data=np.r_[self.seq,sensor,command,dt,flags].tolist())
        # Wait for discovery before first publish; later frames are strictly ordered.
        while self.request.get_subscription_count()==0 or self.response.get_subscription_count()==0:
            rclpy.spin_once(self.plant,timeout_sec=.01);rclpy.spin_once(self.bridge,timeout_sec=.01)
            if time.perf_counter()-start>10:raise TimeoutError('DDS discovery failed')
        self.request.publish(message)
        while self.result is None:
            rclpy.spin_once(self.bridge,timeout_sec=.01);rclpy.spin_once(self.plant,timeout_sec=.01)
            if time.perf_counter()-start>2:raise TimeoutError(f'DDS frame {self.seq} timed out')
        if int(self.result[0])!=self.seq:raise AssertionError('Stale/out-of-order DDS response')
        motors[:]=self.result[1:5];diag[:]=self.result[5:21]
        if not np.isfinite(self.result).all():raise AssertionError('Non-finite result')
        self.latencies.append(time.perf_counter()-start)

def main():
    rclpy.init();fc,nav=libraries();link=DdsFlight(fc)
    out=ROOT/'artifacts/simulation/2026-10-20/ros2';out.mkdir(parents=True,exist_ok=True)
    model=json.loads((ROOT/'sim/model/simulation_parameters.json').read_text())
    selected=json.loads((ROOT/'artifacts/simulation/2026-10-20/summary.json').read_text())
    scale=next(r['gain_scale'] for r in selected['results'] if r['name']==selected['best_nominal_case'])
    result=run_case(link,nav,model,out,'dds-route',scale=scale,duration=30)
    a=np.array(link.latencies)*1000
    result['dds']={'frames':link.seq,'nodes':2,'transport':'Fast DDS UDPv4; same-process Python adapter nodes',
        'latency_median_ms':float(np.median(a)),'latency_p99_ms':float(np.quantile(a,.99)),
        'latency_max_ms':float(a.max()),'timing_claim':'wall-clock round trip; not RP2350 500Hz deadline proof'}
    np.savetxt(out/'dds-latencies-ms.csv',a,delimiter=',',header='round_trip_ms',comments='')
    (out/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result),flush=True)
    link.plant.destroy_node();link.bridge.destroy_node();rclpy.shutdown()
    if not result['success']:raise SystemExit('DDS route failed')

if __name__=='__main__':main()
