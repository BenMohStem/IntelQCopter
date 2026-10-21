"""Rebuild the hardening report from original host, QEMU, ROS2 and Gazebo evidence."""
from pathlib import Path
import json,csv,hashlib,re,struct,zlib,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[2]
def read(p):return json.loads((R/p).read_text(encoding='utf-8-sig'))
def main():
 out=R/'artifacts/reports/hardening-20261021';out.mkdir(parents=True,exist_ok=True)
 host=read('artifacts/validation/hardening-final-20261021/summary.json');ros=read('artifacts/validation/hardening-ros-final-20261021/summary.json');emu=read('artifacts/emulation/hardening-final-20261021/summary.json')
 mr='hardening-manual-final-20261021';nr='hardening-navigation-final-20261021';lr='hardening-linkloss-final-20261021';base=Path('artifacts/gazebo-suite')
 manual=read(base/mr/'result.json');nav=read(base/nr/'transport.json');engine=read(base/nr/'engine-summary.json');loss=read(base/lr/'transport.json')
 mission=(R/base/nr/'drone-mission.log').read_text(encoding='utf8');indices=list(map(int,re.findall(r'ROUTE_WAYPOINT_REACHED index=(\d+)',mission)))
 pc=[json.loads(x) for x in (R/base/nr/'proof-pc.jsonl').read_text(encoding='utf8').splitlines()]
 wire=[json.loads(x) for x in (R/base/nr/'wire-inputs.jsonl').read_text(encoding='utf8').splitlines()];frames=[bytes.fromhex(h) for row in wire for h in row['companion']]
 neutral=[r for r in pc if 'hid' in r and not r['selecting']];publishers=[r['publishers'] for r in pc if 'publishers' in r]
 lostwire=[json.loads(x) for x in (R/base/lr/'wire-inputs.jsonl').read_text(encoding='utf8').splitlines()]
 source=hashlib.sha256((R/'firmware/src/flight_controller.c').read_bytes()).hexdigest()
 checks=dict(host_fault_stress=host['passed'],ROS2_process_fault_injection=ros['passed'],QEMU_63_checks=emu['passed'] and len(emu['checks'])==63,manual_angle_brake_hold=manual['passed'],final_C_source_matches=source==manual['source_sha256']==engine['sha256']['firmware/src/flight_controller.c']==emu['hashes']['firmware/src/flight_controller.c'],navigation_lands=nav['completed'] and nav['final_state']['ground_contact'],all_intermediate_nodes=indices==[1,2,3,4,5],zero_panel_contacts=engine['obstacle_contact_steps']==0,PC_sent_one_route=sum('route_request' in r for r in pc)==1,neutral_pilot_after_selection=bool(neutral) and all(r['hid']=='00800080008000800000000000000000' for r in neutral),PC_has_no_pose_feedback=pc[0]['subscriptions']==['/clock','fc/status'],sole_companion_publisher=bool(publishers) and all(x==[{'node':'drone_mission_computer','namespace':'/drone'}] for x in publishers),UART_CRC_and_finite=bool(frames) and all(len(f)==28 and zlib.crc32(f[:24])==int.from_bytes(f[24:28],'little') and all(math.isfinite(v) for v in struct.unpack('<4f',f[8:24])) for f in frames),link_loss_prevents_completion=not loss['completed'] and not loss['final_state']['health']['companion_fresh'] and any(x.get('deliberately_dropped_companion') for x in lostwire) and all(not x['companion'] for x in lostwire if x['time_s']>=25.1))
 profiles={}
 for row in (R/base/nr/'ros2-profile.jsonl').read_text(encoding='utf8').splitlines():
  s=json.loads(row)['profile'];fields=dict(x.split('=',1) for x in s.split(',') if '=' in x);profiles[fields['component']]=fields
 with (R/'artifacts/validation/hardening-final-20261021/host-api-times-us.csv').open() as f:times=np.array([float(r['host_ctypes_api_us']) for r in csv.DictReader(f)])
 with (R/'artifacts/validation/hardening-final-20261021/collective-stress.csv').open() as f:stress=list(csv.DictReader(f))
 with (R/base/nr/'flight.csv').open() as f:flight=list(csv.DictReader(f))
 plt.style.use('dark_background');fig,axs=plt.subplots(2,2,figsize=(15,10));fig.patch.set_facecolor('#102234')
 for name in ('requested','allocated','ceiling'):axs[0,0].plot([float(x['time_s']) for x in stress],[float(x[name]) for x in stress],label=name)
 axs[0,0].axvline(10,color='white',ls=':',label='RT → LT');axs[0,0].set(title='Actual C collective: stationary saturation bench',xlabel='Logical seconds',ylabel='Normalized demand');axs[0,0].legend()
 axs[0,1].hist(times,bins=np.geomspace(max(.1,times.min()),times.max()+1,50));axs[0,1].set(xscale='log',yscale='log',title='30,000 host API timings — NOT ESP32 deadlines',xlabel='Host ctypes call duration (µs)',ylabel='Samples')
 names=['drone_mission_computer','qcopter_yolo','qcopter_panel_pose'];labels=['Mission','YOLO','PnP'];xs=np.arange(3)
 axs[1,0].bar(xs-.2,[float(profiles.get(n,{}).get('mean_us',0))/1000 for n in names],.4,label='Mean')
 axs[1,0].bar(xs+.2,[float(profiles.get(n,{}).get('max_us',0))/1000 for n in names],.4,label='Max');axs[1,0].set_xticks(xs,labels);axs[1,0].axhline(20,color='orange',ls=':',label='Default diagnostic budget');axs[1,0].set(title='Actual C++ callbacks: steady wall clock',ylabel='Milliseconds');axs[1,0].legend()
 axs[1,1].plot([float(x['x']) for x in flight],[float(x['y']) for x in flight],label='Native Gazebo trajectory')
 route=read(base/nr/'approved-route.json')['points'];axs[1,1].plot([p[0] for p in route],[p[1] for p in route],'o--',label='Approved nodes')
 for i,p in enumerate(route):axs[1,1].annotate(str(i+1),p[:2],xytext=(5,5),textcoords='offset points')
 for x,y,yaw in engine['environment']['panels']:axs[1,1].plot([x+.6*math.sin(yaw),x-.6*math.sin(yaw)],[y-.6*math.cos(yaw),y+.6*math.cos(yaw)],color='orange',lw=5)
 axs[1,1].set(title='True panel positions shown only for evaluation',xlabel='World x (m)',ylabel='World y (m)');axs[1,1].axis('equal');axs[1,1].legend()
 fig.tight_layout();fig.savefig(out/'profiling.png',dpi=130);plt.close(fig)
 result=dict(passed=all(checks.values()),checks=checks,host=host,ROS2=ros,QEMU_checks=len(emu['checks']),manual=manual,source_sha256=source,navigation=dict(run=nr,seconds=nav['simulated_seconds'],contacts=engine['obstacle_contact_steps'],indices=indices),loss=dict(run=lr,expected_failure=True,completed=loss['completed'],companion_fresh=loss['final_state']['health']['companion_fresh']),ROS2_profiles=profiles,assumed_parameters=engine['assumed_parameters'],scope='Actual C host DLL + native Gazebo/OGRE2 + actual ROS2 C++ + separate ESP32-S3 QEMU synthetic replay. Perfect localization/synthetic sensors; not physical BLE/I2C/UART/ESC or 500Hz deadline proof. Known-panel YOLO/PnP, not general RGB depth.')
 (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
 entries=''.join('<li>'+('PASS' if v else 'FAIL')+' — '+k.replace('_',' ')+'</li>' for k,v in checks.items())
 html=f'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>FC and ROS2 hardening proof</title><style>body{{max-width:1150px;margin:35px auto;padding:20px;background:#102234;color:#e2edf6;font:17px/1.65 system-ui}}a{{color:#6ce5c9}}video,img{{width:100%;border-radius:12px}}section{{padding:20px;background:#193346;border-radius:14px;margin:20px 0}}li{{margin:7px 0}}.note{{border-left:4px solid #fac276;padding:12px;background:#243748}}h1,h2{{line-height:1.3}}</style></head><body>
 <h1>Gentle controls · preserved PID headroom · observable failures</h1>
 <p>Actual C FC, native Gazebo 10.5/OGRE2 cameras and ROS2 C++ nodes. All original trials, including failures, remain saved.</p>
 <p class="note">Physical motor arming remains disabled. XY hold uses explicit perfect simulated localization. Known-size panel perception is not general monocular obstacle ranging. Host microseconds are not ESP32 timing guarantees.</p>
 <section><h2>{'PASS' if result['passed'] else 'FAIL'} — current hardening checks</h2><p>{len(host['checks'])} C host checks · 20,000 malformed-packet trials · 30,000-step soak + 5,000 mixed-axis steps · {len(ros['checks'])} actual ROS2 fault checks · {len(emu['checks'])} QEMU checks.</p><p>Gentle is the boot default. Full RT permits at most 0.8 collective (1800 µs equivalent), with differential motor range retained for the PIDs. LT decreases climb and collective allowance. This is not an ESC driver.</p><ul>{entries}</ul></section>
 <h2>Measured graphs</h2><img src="profiling.png" alt="Actual collective ceiling, host timing distribution, ROS2 callback durations and native route">
 <p>YOLO's default 20 ms diagnostic-budget overruns are retained. Its wall-clock inference rate differs from 15 fps recording and from the 500 Hz FC control interval.</p>
 <h2>Final seven-node autonomous flight · {nav['simulated_seconds']:.2f} simulated seconds</h2><video controls preload="metadata" src="../../gazebo-suite/{nr}/gazebo.mp4"></video><p>One PC route request, neutral pilot after mode selection; the onboard planner generates UART commands and the C FC generates motor demands. Zero panel contacts.</p>
 <h2>Standalone angle → brake → level/hold</h2><video controls preload="metadata" src="../../gazebo-suite/{mr}/gazebo.mp4"></video><p>Predetermined raw Xbox-format input only; no ROS2 or feedback test pilot. Source hash matches the navigation and QEMU C source.</p>
 <h2>Expected failed trial: companion cable loss</h2><p>After 25 simulated seconds, the transport drops companion frames. The route does not complete and the FC marks commands stale. No fallback script completes the route.</p><video controls preload="metadata" src="../../gazebo-suite/{lr}/gazebo.mp4"></video>
 <p><a href="summary.json">All measured checks, parameters and hashes</a> · <a href="/debug-guide">Diagnostics and real-flight preparation guide</a> · <a href="../../gazebo-suite/{nr}/wire-inputs.jsonl">Original HID/UART</a> · <a href="../../gazebo-suite/{nr}/ros2-profile.jsonl">Original ROS2 profiles</a> · <a href="../../gazebo-suite/{nr}/drone-mission.log">Original onboard mission log</a> · <a href="../../gazebo-suite/{mr}/controller-trace.jsonl">C control and diagnostic trace</a></p>
 </body></html>'''
 (out/'index.html').write_text(html,encoding='utf8');print(json.dumps(checks,indent=2));assert result['passed']
if __name__=='__main__':main()
