"""Real Windows controller reports only. No navigation or synthetic pilot logic."""
import argparse,csv,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from std_msgs.msg import UInt8MultiArray,String
from xinput_controller import Controller

class Xbox(Node):
    def __init__(self,args):
        super().__init__('real_xbox_ground_station');self.controller=Controller(args.controller);self.last=None
        self.pub=self.create_publisher(UInt8MultiArray,'pilot/xbox_report',10);self.info=self.create_publisher(String,'pilot/state',10)
        self.file=Path(args.log).open('w',newline='',encoding='utf8');self.writer=csv.writer(self.file);self.writer.writerow(['wall_monotonic_s','connected','index','raw_hid_hex'])
        self.create_timer(.02,self.poll,clock=Clock(clock_type=ClockType.STEADY_TIME))
        print('LIVE XBOX READY rmw='+rclpy.get_rmw_implementation_identifier(),flush=True)
    def poll(self):
        raw,info=self.controller.poll();info['wall_time_s']=time.time()
        if raw:self.pub.publish(UInt8MultiArray(data=list(raw)))
        self.info.publish(String(data=json.dumps(info)));self.writer.writerow([time.monotonic(),info['connected'],info['index'],raw.hex() if raw else '']);self.file.flush()
        if self.last!=info['connected']:print('XBOX',json.dumps(info),flush=True);self.last=info['connected']

def main():
    p=argparse.ArgumentParser();p.add_argument('--controller',type=int,choices=range(4));p.add_argument('--log',required=True);args,ros=p.parse_known_args()
    rclpy.init(args=ros);node=Xbox(args)
    try:rclpy.spin(node)
    finally:node.file.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
