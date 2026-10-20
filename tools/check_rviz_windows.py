"""Launch actual RViz and robot_state_publisher; save original logs and TF proof."""
from pathlib import Path
import json, os, subprocess, time, yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'artifacts/setup/jazzy-repair/rviz'
OUT.mkdir(parents=True,exist_ok=True)
share=ROOT/'build/ros2-install/share/qcopter_description'
params=ROOT/'build/f450-view-parameters.yaml'
params.write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':
    {'robot_description':(share/'urdf/f450.urdf').read_text()}}}))
ros=Path('C:/dev/jazzy/install')
processes=[];logs=[]
rclpy.init();node=rclpy.create_node('f450_rviz_probe');transforms={}
def receive(message):
    for transform in message.transforms:transforms[transform.child_frame_id]=transform.header.frame_id
subscription=node.create_subscription(TFMessage,'/tf_static',receive,
    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
try:
    for name,args in [('robot_state_publisher',['--ros-args','--params-file',str(params)]),
                      ('rviz2',['-d',str(share/'rviz/f450.rviz')])]:
        log=open(OUT/f'{name}.log','w');logs.append(log)
        processes.append(subprocess.Popen([str(ros/f'lib/{name}/{name}.exe'),*args],stdout=log,stderr=subprocess.STDOUT))
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    alive=[proc.poll() is None for proc in processes]
    for handle in logs:handle.flush()
    rviz_log=(OUT/'rviz2.log').read_text(errors='replace')
    result={'robot_state_publisher_running':alive[0],'rviz_running':alive[1],
        'received_fixed_transforms':transforms,
        'OpenGL_initialized':'OpenGl version' in rviz_log or 'OpenGL version' in rviz_log,
        'scope':'Actual RViz process/logs and static TF; no screenshot or interactive GUI inspection claimed'}
    result['passed']=all(alive) and len(transforms)==14 and result['OpenGL_initialized']
    (OUT/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    assert result['passed'],'RViz/TF startup check failed; original logs retained'
finally:
    for process in processes:
        if process.poll() is None:process.terminate()
        process.wait(timeout=5)
    for log in logs:log.close()
    node.destroy_node();rclpy.shutdown()
