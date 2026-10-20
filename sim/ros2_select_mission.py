"""Working bundled ROS2 point selector; this requests a simulation mission only."""
import ros2_probe
import argparse,time,json
import rclpy
from std_msgs.msg import Float64MultiArray

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--points',type=float,nargs=7,metavar='METRES')
    args=parser.parse_args()
    points=args.points or list(map(float,input('Start x y z, finish x y z, cruise height (7 metre values): ').split()))
    if len(points)!=7:raise SystemExit('Exactly seven coordinates/height values required')
    rclpy.init();node=rclpy.create_node('garden_mission_selector');acks=[]
    sub=node.create_subscription(Float64MultiArray,'garden/mission_selection_ack',lambda m:acks.append(list(m.data)),10)
    pub=node.create_publisher(Float64MultiArray,'garden/mission_selection',10);deadline=time.monotonic()+15
    while time.monotonic()<deadline and not acks:
        if pub.get_subscription_count():pub.publish(Float64MultiArray(data=points))
        rclpy.spin_once(node,timeout_sec=.1)
    if not acks:raise SystemExit('No mission acknowledgement; start bridge and waiting simulation first')
    if acks[-1]!=points:raise SystemExit('Mission acknowledgement did not match requested points')
    print(json.dumps({'requested':points,'acknowledged':acks[-1],'physical_arming':False}),flush=True)
    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
