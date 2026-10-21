"""Ground PC role. Publishes real-format Xbox reports and sends endpoint requests.

Manual cases are an explicit feedback-based virtual human using the pilot HUD's
simulated pose/velocity. Its ONLY flight-control output is 16-byte Xbox HID reports.
It never calls the FC API or publishes companion flight setpoints.
"""
import argparse,csv,json,math,struct
from pathlib import Path
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray,String,Bool
from qcopter_autonomy.srv import RequestMission

ROOT=Path(__file__).resolve().parents[2]
def raw_axis(value):
    value=max(-1,min(1,value));raw=0 if value==0 else math.copysign(.12+.88*abs(value),value)
    return max(0,min(65535,round(32768+32768*raw)))
def packet(roll=0,pitch=0,yaw=0,vertical=0,lt=0,rt=0,down=False,a=False,b=False):
    return struct.pack('<6H4B',raw_axis(yaw),raw_axis(-vertical),raw_axis(roll),raw_axis(-pitch),round(1023*lt),round(1023*rt),5 if down else 0,(1 if a else 0)|(2 if b else 0),0,0)

class Station(Node):
 def __init__(self,args):
    super().__init__('pc_xbox_and_mission_station');self.args=args
    self.pub=self.create_publisher(UInt8MultiArray,'pilot/xbox_report',10)
    self.done_pub=self.create_publisher(Bool,'pilot/completed',10)
    self.state_pub=self.create_publisher(String,'pilot/state',10)
    self.client=self.create_client(RequestMission,'mission/request')
    self.create_subscription(Odometry,'odometry',self.pose,10)
    self.create_subscription(UInt8MultiArray,'fc/status',self.fc,10)
    self.create_subscription(String,'mission/status',self.mission,10)
    self.create_subscription(Clock,'/clock',self.clock,10)
    self.position=self.velocity=[0,0,0];self.heading=0.;self.mode=0;self.disarmed=False;self.begin=None;self.stage=0;self.stage_time=0;self.last_time=-1
    self.requested=self.accepted=self.mission_done=False;self.disarm_time=None;self.dwell=None;self.row_count=0
    self.spec=next(s for s in json.loads((ROOT/'simulation/gazebo/environments.json').read_text()) if s['id']==args.env)
    out=ROOT/'artifacts/gazebo-suite'/args.run;out.mkdir(parents=True,exist_ok=True)
    self.file=(out/'xbox-reports.csv').open('w',newline='');self.writer=csv.writer(self.file);self.writer.writerow(['time_s','phase','raw_hid_hex','roll_stick','pitch_stick','yaw_stick','vertical_stick','LT','RT'])
    self.pc_log=(out/'pc-mission.json');self.pc_log.write_text(json.dumps({'role':'ground PC','mission_requests':0,'manual':args.manual is not None,'external_mission':args.external_mission}))
    print('PC READY: simulated Xbox packets and endpoint service requests only',flush=True)
 def pose(self,m):
    p=m.pose.pose.position;v=m.twist.twist.linear;q=m.pose.pose.orientation
    self.position=[p.x,p.y,p.z];self.velocity=[v.x,v.y,v.z];self.heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 def fc(self,m):
    if len(m.data)==16:self.mode=m.data[3];self.disarmed=bool(m.data[4])
 def mission(self,m):
    if 'done=1' in m.data:self.mission_done=True
 def clock(self,m):
    now=m.clock.sec+m.clock.nanosec*1e-9
    if now<=self.last_time:return
    self.last_time=now
    if self.begin is None:self.begin=now;self.stage_time=now
    elapsed=now-self.begin;values={};phase='select-stable' if self.args.manual is not None else 'select-navigation'
    # Like a person watching the mode indicator: retain the gesture until the
    # FC acknowledges navigation. A real DDS gap may restart the FC's 5 s timer.
    selecting=elapsed<5.4 or (self.args.manual is None and self.mode!=1 and not self.requested)
    if selecting:values=dict(down=True,a=self.args.manual is not None,b=self.args.manual is None)
    elif self.args.manual is None:
        phase='navigation-neutral-xbox'
        if not self.args.external_mission and not self.requested and self.mode==1 and self.client.service_is_ready():
            req=RequestMission.Request();req.start.x,req.start.y,req.start.z=self.spec['start'];req.finish.x,req.finish.y,req.finish.z=self.spec['finish'];req.cruise_height_m=self.spec['cruise']
            self.requested=True
            future=self.client.call_async(req)
            def response(f):
                reply=f.result();self.accepted=reply.accepted
                self.pc_log.write_text(json.dumps({'role':'ground PC','mission_requests':1,'start':self.spec['start'],'finish':self.spec['finish'],'cruise':self.spec['cruise'],'accepted':reply.accepted,'reason':reply.reason},indent=2))
                print('PC ENDPOINT RESPONSE',reply.accepted,reply.reason,flush=True)
            future.add_done_callback(response)
        if self.mission_done:self.disarm_time=self.disarm_time or now
    else:
        i=self.args.manual;end=[0.,0.,.082] if i%2==0 else [1.1,-.45,.082]
        targets=[[0,0,1.0],[1.1,.35 if i%2==0 else -.3,1.0],[1.6,-.4 if i%2==0 else .4,1.1],[end[0],end[1],.65]]
        phase='manual-'+str(self.stage)
        if self.stage<len(targets):
            target=targets[self.stage];ex=[target[j]-self.position[j] for j in range(3)]
            desired=[max(-.45,min(.45,.7*ex[j])) for j in range(2)]
            accel=[max(-.8,min(.8,1.7*(desired[j]-self.velocity[j]))) for j in range(2)]
            bx=math.cos(self.heading)*accel[0]+math.sin(self.heading)*accel[1];by=-math.sin(self.heading)*accel[0]+math.cos(self.heading)*accel[1]
            yaw_target=.20 if self.stage==2 else 0
            values=dict(roll=-math.atan2(by,9.80665)*180/math.pi/35,pitch=math.atan2(bx,9.80665)*180/math.pi/35,
                yaw=max(-.2,min(.2,1.7*(yaw_target-self.heading)*180/math.pi/180)),vertical=max(-.5,min(.5,ex[2]))/2)
            close=math.hypot(*ex[:2])<.12 and abs(ex[2])<.1 and math.hypot(*self.velocity[:2])<.13
            self.dwell=(self.dwell or now) if close else None
            if self.dwell is not None and now-self.dwell>.7:
                self.stage+=1;self.stage_time=now;self.dwell=None;print('VIRTUAL HUMAN waypoint',self.stage,self.position,flush=True)
        else:
            phase='manual-LT-descent';values=dict(lt=.45)
            # A virtual pilot continues correcting lateral drift with the right
            # stick while lowering height using LT. No hidden FC position input.
            accel=[max(-.8,min(.8,1.7*(max(-.45,min(.45,.7*(end[j]-self.position[j])))-self.velocity[j]))) for j in range(2)]
            bx=math.cos(self.heading)*accel[0]+math.sin(self.heading)*accel[1];by=-math.sin(self.heading)*accel[0]+math.cos(self.heading)*accel[1]
            values.update(roll=-math.atan2(by,9.80665)*180/math.pi/35,pitch=math.atan2(bx,9.80665)*180/math.pi/35)
            if self.position[2]<.105 and abs(self.velocity[2])<.12:self.disarm_time=self.disarm_time or now
    if self.disarm_time is not None:
        phase='hold-Down-LT-disarm';values=dict(lt=1.,down=True)
        if now-self.disarm_time>2.4 and self.disarmed:
            self.done_pub.publish(Bool(data=True));phase='complete'
    raw=packet(**values);self.pub.publish(UInt8MultiArray(data=list(raw)))
    self.writer.writerow([now,phase,raw.hex(),*[values.get(k,0) for k in ('roll','pitch','yaw','vertical','lt','rt')]]);self.file.flush();self.row_count+=1
    self.state_pub.publish(String(data=json.dumps(dict(time=now,phase=phase,mode=self.mode,mission_requests=int(self.requested),accepted=self.accepted,virtual_human=self.args.manual is not None))))

def main():
 p=argparse.ArgumentParser();p.add_argument('--env',required=True);p.add_argument('--run',required=True);p.add_argument('--manual',type=int);p.add_argument('--external-mission',action='store_true');args,ros=p.parse_known_args()
 rclpy.init(args=ros);node=Station(args)
 try:rclpy.spin(node)
 finally:node.file.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
