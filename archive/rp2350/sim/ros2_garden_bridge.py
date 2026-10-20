"""Python3.12 ROS DDS adapter, reached by Python3.10 simulator over local IPC.

Actual FC calls (including lidar corrections) occur only in the DDS subscriber.
This is a test harness, not the unbuilt ROS C++ mission package or hardware UART.
"""
import ros2_probe
import sys,socket,struct,json,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent/'sitl'))
import numpy as np
import ctypes as ct
from run import libraries,ROOT
import rclpy
from std_msgs.msg import Float32MultiArray,Float64MultiArray,Float32,UInt8
from rclpy.qos import QoSProfile,DurabilityPolicy

def exact(stream,n):
    buf=b''
    while len(buf)<n:
        part=stream.recv(n-len(buf))
        if not part:raise EOFError
        buf+=part
    return buf

def main():
    fc,_=libraries();fc.qc_sitl_height_observation.argtypes=[ct.c_float,ct.c_float]
    rclpy.init();plant=rclpy.create_node('garden_transport_adapter');controller=rclpy.create_node('garden_c_flight_core')
    fc.qc_sitl_set_battery_soc.argtypes=[ct.c_float]
    fc.qc_sitl_battery_flags.restype=ct.c_uint
    battery_pub=controller.create_publisher(UInt8,'qcopter/battery_policy',
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    last_battery_flags=None
    def publish_battery_policy():
        nonlocal last_battery_flags
        flags=int(fc.qc_sitl_battery_flags())
        if flags!=last_battery_flags:
            battery_pub.publish(UInt8(data=flags))
            print(f'Battery policy: warning={bool(flags&1)} disarm_latched={bool(flags&2)}; no automatic landing',flush=True)
            last_battery_flags=flags
    def battery_soc(message):
        if np.isfinite(message.data) and 0<=message.data<=100:
            fc.qc_sitl_set_battery_soc(float(message.data));publish_battery_policy()
    battery_sub=controller.create_subscription(Float32,'qcopter/battery_soc_percent',battery_soc,10)
    response=None;selection=None
    selection_reply=controller.create_publisher(Float64MultiArray,'garden/mission_selection_ack',10)
    def select(m):
        nonlocal selection
        a=np.array(m.data,np.float64)
        if len(a)!=7 or not np.isfinite(a).all() or not(.6<=a[6]<=2.5):return
        if not(-1<=a[0]<=9 and -1<=a[3]<=9 and abs(a[1])<=3 and abs(a[4])<=3):return
        selection=a
        selection_reply.publish(Float64MultiArray(data=a.tolist()))
        out=ROOT/'artifacts/simulation/2026-10-20-missions/ros-selected-points.json'
        out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps({'start':a[:3].tolist(),'finish':a[3:6].tolist(),'cruise_height_m':float(a[6]),'source':'actual ROS2 mission_selection topic; no arming'},indent=2),encoding='utf-8')
    selected_sub=controller.create_subscription(Float64MultiArray,'garden/mission_selection',select,10)
    pub=plant.create_publisher(Float32MultiArray,'garden/sensors_commands',10)
    reply=controller.create_publisher(Float32MultiArray,'garden/control_reply',10)
    def receive(m):
        nonlocal response
        response=np.array(m.data,np.float32)
    def control(m):
        a=np.array(m.data,np.float32);op=int(a[0]);seq=a[1];payload=a[2:]
        if op==1:fc.qc_sitl_reset(float(payload[0]),float(payload[1]),payload[2:].copy());result=[]
        elif op==2:
            motor=np.zeros(4,np.float32);diag=np.zeros(16,np.float32)
            fc.qc_sitl_step(payload[:9].copy(),payload[9:14].copy(),float(payload[14]),int(payload[15]),motor,diag);result=np.r_[motor,diag].tolist()
        elif op==3:fc.qc_sitl_height_observation(float(payload[0]),float(payload[1]));result=[]
        else:raise ValueError('Unknown operation')
        publish_battery_policy()
        reply.publish(Float32MultiArray(data=[float(seq),*result]))
    sub=controller.create_subscription(Float32MultiArray,'garden/sensors_commands',control,10)
    received=plant.create_subscription(Float32MultiArray,'garden/control_reply',receive,10)
    server=socket.socket();server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);server.bind(('127.0.0.1',9241));server.listen(1);server.settimeout(60)
    print('Garden ROS2 bridge listening at localhost:9241',flush=True);connection,_=server.accept();connection.settimeout(30)
    seq=0;latencies=[];counts={1:0,2:0,3:0}
    try:
        while True:
            op,n=struct.unpack('<II',exact(connection,8))
            if op==9:break
            if op==4:
                deadline=time.perf_counter()+30
                while selection is None and time.perf_counter()<deadline:rclpy.spin_once(controller,timeout_sec=.1)
                if selection is None:raise TimeoutError('No ROS mission selection received')
                result=selection.astype('<f4').tobytes();connection.sendall(struct.pack('<I',7)+result);continue
            if n>100:raise ValueError('Oversized IPC frame')
            values=np.frombuffer(exact(connection,4*n),dtype='<f4');seq+=1;response=None;start=time.perf_counter()
            while pub.get_subscription_count()==0 or reply.get_subscription_count()==0:
                rclpy.spin_once(plant,timeout_sec=.01);rclpy.spin_once(controller,timeout_sec=.01)
                if time.perf_counter()-start>10:raise TimeoutError('DDS discovery')
            pub.publish(Float32MultiArray(data=[float(op),float(seq),*map(float,values)]))
            while response is None:
                rclpy.spin_once(controller,timeout_sec=.01);rclpy.spin_once(plant,timeout_sec=.01)
                if time.perf_counter()-start>2:raise TimeoutError(f'DDS request {seq}')
            if int(response[0])!=seq:raise ValueError('Out-of-order DDS response')
            result=response[1:].astype('<f4').tobytes();connection.sendall(struct.pack('<I',len(result)//4)+result)
            counts[op]+=1;latencies.append((time.perf_counter()-start)*1000)
    finally:
        connection.close();server.close();plant.destroy_node();controller.destroy_node();rclpy.shutdown()
    report={'transport':'Two Python ROS2 DDS adapter nodes + localhost TCP IPC to Python3.10 renderer',
      'requests':seq,'operation_counts':counts,'median_ms':float(np.median(latencies)),
      'p99_ms':float(np.quantile(latencies,.99)),'max_ms':float(max(latencies)),
      'scope':'Actual C reset, control and lidar fusion through DDS; not ROS C++ wrapper/hardware/real-time proof'}
    out=ROOT/'artifacts/simulation/2026-10-20-missions/garden-ros2-transport.json'
    out.write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report),flush=True)

if __name__=='__main__':main()
