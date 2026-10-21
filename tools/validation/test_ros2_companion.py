"""Native ROS C++ integration test using a recorded Gazebo RGB frame.

This tests compiled nodes over DDS. It does not claim closed-loop flight using
the ROS C++ wrappers, camera calibration on hardware, or measured panel accuracy.
"""
from pathlib import Path
import json, math, os, subprocess, time
import cv2
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from qcopter_autonomy.msg import SemanticDetections, MetricObstacles, AutonomySetpoint
from qcopter_autonomy.srv import RequestMission

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/setup/jazzy-repair'/('cpp-integration-'+time.strftime('%H%M%S'))
OUT.mkdir(parents=True,exist_ok=True)
BIN=ROOT/'build/ros2-install/lib/qcopter_autonomy'
processes=[]; handles=[]
def start(name,*args):
    log=open(OUT/f'{name}-{len(handles)}.log','w');handles.append(log)
    proc=subprocess.Popen([str(BIN/f'{name}.exe'),*args],stdout=log,stderr=subprocess.STDOUT)
    processes.append(proc)
    return proc

rclpy.init();node=rclpy.create_node('qcopter_cpp_integration_probe')
semantics=[];obstacles=[];commands=[]
subscriptions=[
    node.create_subscription(SemanticDetections,'perception/semantics',semantics.append,10),
    node.create_subscription(MetricObstacles,'perception/metric_obstacles',obstacles.append,10),
    node.create_subscription(AutonomySetpoint,'autonomy/setpoint',commands.append,10)]
odom=node.create_publisher(Odometry,'odometry',10)
camera=node.create_publisher(Image,'camera/image',10)
info=node.create_publisher(CameraInfo,'camera/camera_info',10)
contact=node.create_publisher(Bool,'simulation/ground_contact',10)
image_path=ROOT/'artifacts/simulation/2026-10-20-engines/gazebo-garden-ros-camera-6s.png'
frame=cv2.imread(str(image_path));assert frame is not None
height,width=frame.shape[:2];focal=height/(2*math.tan(math.radians(70)/2))
def tick(seconds,publish=True):
    deadline=time.monotonic()+seconds
    next_frame=0.;next_pose=0.
    while time.monotonic()<deadline:
        if publish and time.monotonic()>=next_pose:
            stamp=node.get_clock().now().to_msg()
            m=Odometry();m.header.stamp=stamp;m.header.frame_id='map';m.child_frame_id='map'
            m.pose.pose.position.z=.082;m.pose.pose.orientation.w=1.;odom.publish(m)
            contact.publish(Bool(data=True));next_pose=time.monotonic()+.05
        if publish and time.monotonic()>=next_frame:
            stamp=node.get_clock().now().to_msg()
            im=Image();im.header.stamp=stamp;im.header.frame_id='camera_optical'
            im.width=width;im.height=height;im.encoding='bgr8';im.step=width*3;im.data=frame.tobytes();camera.publish(im)
            ci=CameraInfo();ci.header=im.header;ci.width=width;ci.height=height
            ci.k=[focal,0.,width/2,0.,focal,height/2,0.,0.,1.];ci.d=[0.]*5
            ci.distortion_model='plumb_bob';info.publish(ci)
            next_frame=time.monotonic()+.3
        rclpy.spin_once(node,timeout_sec=.01)
        for proc in processes:
            if proc.poll() is not None and proc.returncode!=0:
                raise RuntimeError(f'Compiled node exited {proc.returncode}; see {OUT}')

def request():
    client=node.create_client(RequestMission,'mission/request')
    assert client.wait_for_service(timeout_sec=5),'Mission service missing'
    req=RequestMission.Request();req.start.z=.082;req.finish.x=8.;req.finish.z=.082;req.cruise_height_m=1.25
    result=client.call_async(req)
    deadline=time.monotonic()+4
    while not result.done() and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.01)
    assert result.done(),'Mission service timeout'
    response=result.result();node.destroy_client(client)
    return response

report={}
try:
    start('yolo_node','--ros-args','-p',f'onnx_model:={ROOT}/build/panel-training/rgb-panel/weights/best.onnx','-p','input_size:=320')
    start('panel_pose_node','--ros-args','-p','known_fixture_model_confirmed:=true')
    dormant=start('mission_node')
    tick(8)
    disabled=request();report['default_mission_rejected']=not disabled.accepted
    report['default_commands_inactive']=bool(commands) and all(not c.valid for c in commands)
    dormant.terminate();dormant.wait(timeout=5);processes.remove(dormant)
    commands.clear()
    start('mission_node','--ros-args','-p','enable_simulation_missions:=true')
    tick(3)
    tick(.03)
    accepted=request();report['enabled_fresh_mission_accepted']=accepted.accepted
    report['enabled_request_reason']=accepted.reason
    tick(2)
    report['active_commands_received']=sum(c.valid for c in commands)
    report['semantic_messages']=len(semantics)
    report['detected_panel_messages']=sum(bool(s.detections) for s in semantics)
    report['processed_metric_messages']=sum(o.image_processed for o in obstacles)
    report['ranged_panel_messages']=sum(bool(o.centers) for o in obstacles)
    report['source_image']=str(image_path.relative_to(ROOT))
    report['scope']='Recorded native RGB input and synthetic pose; DDS C++ node integration, not a flight or distance-accuracy proof'
    report['passed']=all([report['default_mission_rejected'],report['default_commands_inactive'],
        report['enabled_fresh_mission_accepted'],report['active_commands_received']>0,
        report['detected_panel_messages']>0,report['ranged_panel_messages']>0])
    (OUT/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    assert report['passed'],'C++ integration checks failed'
finally:
    for process in processes:
        if process.poll() is None:process.terminate()
        process.wait(timeout=5)
    for handle in handles:handle.close()
    node.destroy_node();rclpy.shutdown()
