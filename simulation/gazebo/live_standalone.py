"""ROS-free live flight boundary: real XInput -> active C core in native Gazebo."""
import argparse,csv,json,subprocess,time
from pathlib import Path
from multiprocessing.connection import Client
import cv2,numpy as np
from xinput_controller import Controller
from live_view import Viewer
from runtime_paths import ROOT,ffmpeg

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--controller',type=int,choices=range(4));p.add_argument('--duration',type=float,default=0);args=p.parse_args()
    out=ROOT/'artifacts/gazebo-suite'/args.run;controller=Controller(args.controller);viewer=Viewer();client=Client(('127.0.0.1',9251),authkey=b'intelqcopter-esp32')
    client.send({'op':'state'});state=client.recv();initial=state['sim_time_s'];start=time.monotonic();next_wall=start;last_frame=-99;reports=0;seen=False
    log=(out/'real-xbox.csv').open('w',newline='',encoding='utf8');writer=csv.writer(log);writer.writerow(['wall_monotonic_s','sim_time_s','connected','index','raw_hid_hex'])
    stderr=(out/'ffmpeg.log').open('w');record=subprocess.Popen([ffmpeg(),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','960x540','-r','5','-i','-','-an','-c:v','libx264','-preset','ultrafast','-crf','27','-pix_fmt','yuv420p',str(out/'gazebo.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=stderr)
    print('STANDALONE READY: no ROS2 imported, no companion process; actual XInput only',flush=True)
    try:
        while not viewer.stopped.is_set() and (not args.duration or time.monotonic()-start<args.duration):
            raw,info=controller.poll();reports+=bool(raw);seen|=bool(raw)
            writer.writerow([time.monotonic(),state['sim_time_s'],info['connected'],info['index'],raw.hex() if raw else '']);log.flush()
            client.send({'op':'step','hid':[raw] if raw else [],'companion':[]});state=client.recv();t=state['sim_time_s']
            if t-last_frame>=.199:
                canvas=np.full((540,960,3),[20,28,40],np.uint8)
                for topic,x,y in [('/overview/rgb',0,0),('/drone/rgb',640,0)]:
                    client.send({'op':'camera','topic':topic});w,h,data=client.recv();canvas[y:y+h,x:x+w]=np.frombuffer(data,np.uint8).reshape(h,w,3)
                texts=[f'Native Gazebo + ESP32 C host core | standalone | t={t-initial:.1f}s',f'Xbox connected={info["connected"]} reports={reports} | FC mode={state["mode"]}',f'disarm={state["disarm"]} | physical ESC output disabled | ROS2 absent']
                for i,text in enumerate(texts):cv2.putText(canvas,text,(15,390+35*i),cv2.FONT_HERSHEY_SIMPLEX,.52,(230,240,245),1,cv2.LINE_AA)
                record.stdin.write(canvas.tobytes());last_frame=t
                viewer.update(canvas,dict(sim_time_s=t-initial,real_time_factor=(t-initial)/max(.001,time.monotonic()-start),controller=info,fc_mode=state['mode'],disarmed=state['disarm'],position=state['position'],simulation_actuation=state.get('simulation_actuation'),standalone=True,companion_frames=0))
            next_wall+=.02;time.sleep(max(0,min(.02,next_wall-time.monotonic())))
    except KeyboardInterrupt:pass
    finally:
        log.close();record.stdin.close();record.wait(timeout=30);stderr.close();client.send({'op':'close'});client.recv();client.close();viewer.close()
        (out/'live-summary.json').write_text(json.dumps(dict(engine='Native Gazebo 10.5/OGRE2 + active C host DLL',standalone=True,ROS2_loaded=False,real_controller_seen=seen,real_hid_reports=reports,companion_frames=0,final_state={k:v for k,v in state.items() if k!='fc'},scope='Interactive session, not an autonomous test pass. Synthetic sensors/calibration and assumed model. Physical BLE/I2C/ESC timing unvalidated.'),indent=2)+'\n',encoding='utf8')
if __name__=='__main__':main()
