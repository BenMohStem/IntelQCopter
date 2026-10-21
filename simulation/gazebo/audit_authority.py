"""Audit original wire inputs and native physics data; no flight-control output."""
import argparse,csv,json,math,re,struct,zlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--loss-run');p.add_argument('--manual-run');a=p.parse_args();d=ROOT/'artifacts/gazebo-suite'/a.run
 t=json.loads((d/'transport.json').read_text());e=json.loads((d/'engine-summary.json').read_text());route=json.loads((d/'approved-route.json').read_text());pc=[json.loads(x) for x in (d/'proof-pc.jsonl').read_text().splitlines()];wire=[json.loads(x) for x in (d/'wire-inputs.jsonl').read_text().splitlines()]
 req=[x for x in pc if 'route_request' in x];neutral=[x for x in pc if 'hid' in x and not x['selecting']];publishers=[x['publishers'] for x in pc if 'publishers' in x];frames=[bytes.fromhex(h) for row in wire for h in row['companion']];seq=[int.from_bytes(f[4:8],'little') for f in frames]
 logs=(d/'drone-mission.log').read_text();indices=list(map(int,re.findall(r'ROUTE_WAYPOINT_REACHED index=(\d+)',logs)));error=math.dist(t['final_state']['position'][:2],route['points'][-1][:2]);checks={
 'one_route_request':len(req)==1 and sum(x.get('accepted',False) for x in pc)==1,
 'neutral_HID_after_mode_selection':bool(neutral) and all(x['hid']=='00800080008000800000000000000000' for x in neutral),
 'no_PC_pose_or_image_subscription':pc[0]['subscriptions']==['/clock','fc/status'],
 'sole_companion_publisher_is_drone':bool(publishers) and all(x==[{'node':'drone_mission_computer','namespace':'/drone'}] for x in publishers),
 'all_intermediate_nodes_acknowledged':indices==list(range(1,len(route['points'])-1)),
 'all_companion_frames_valid':bool(frames) and all(len(f)==28 and f[:3]==b'QN\x01' and f[3] in (1,3) and zlib.crc32(f[:24])==int.from_bytes(f[24:28],'little') and all(math.isfinite(v) for v in struct.unpack('<4f',f[8:24])) for f in frames),
 'companion_setpoints_within_FC_limits':all(all(abs(v)<=limit for v,limit in zip(struct.unpack('<4f',f[8:24]),[35,35,180,3])) for f in frames),
 'companion_sequence_strictly_increases':all(y>x for x,y in zip(seq,seq[1:])),
 'landing_disarm_originated_in_drone_command':any(f[3]==3 for f in frames) and t['final_state']['disarm'],
 'completed_on_ground_near_final_node':t['completed'] and t['final_state']['ground_contact'] and error<.2,
 'zero_native_obstacle_contact_steps':e['obstacle_contact_steps']==0}
 loss=None
 if a.loss_run:
  ld=ROOT/'artifacts/gazebo-suite'/a.loss_run;lt=json.loads((ld/'transport.json').read_text());lw=[json.loads(x) for x in (ld/'wire-inputs.jsonl').read_text().splitlines()];dropped=[x for x in lw if x.get('deliberately_dropped_companion')]
  loss=dict(counter_note='This original trial transport counter counts generated DDS frames before the deliberate drop; delivered frames are in wire-inputs and engine-summary. Future counter is corrected.',delivered_frames=sum(len(x['companion']) for x in lw),run=a.loss_run,completed=lt['completed'],companion_fresh=lt['final_state']['health']['companion_fresh'],dropped_frames=sum(len(x['deliberately_dropped_companion']) for x in dropped),final_position=lt['final_state']['position'])
  checks['UART_loss_prevents_completion']=bool(dropped) and not lt['completed'] and not lt['final_state']['health']['companion_fresh'] and all(not x['companion'] for x in lw if x['time_s']>=25.1)
 manual=None
 if a.manual_run:
  md=ROOT/'artifacts/gazebo-suite'/a.manual_run;manual=json.loads((md/'manual-audit.json').read_text());me=json.loads((md/'engine-summary.json').read_text());checks['manual_input_audit_passed']=manual['passed'];checks['manual_and_auto_use_same_C_core_binary']=me['sha256']['build/sitl/esp32_flight_core.dll']==e['sha256']['build/sitl/esp32_flight_core.dll']
 summary=dict(manual_replay=manual,passed=all(checks.values()),checks=checks,run=a.run,node_count=len(route['points']),acknowledged_intermediate_indices=indices,route_requests=len(req),neutral_reports=len(neutral),UART_frames=len(frames),landing_error_m=error,simulated_seconds=t['simulated_seconds'],obstacle_contact_steps=e['obstacle_contact_steps'],minimum_evaluated_clearance_m=e['min_evaluated_clearance_m'],loss_control=loss,engine=e['engine'],assumed_parameters=e['assumed_parameters'],limitations=['C host DLL, not Xtensa execution in this physics run','Perfect Gazebo localization; synthetic sensors and calibration','YOLO/PnP uses recognizable known-size panels; not general depth','Physical BLE, UART wiring, sensor drivers, ESC outputs and deadlines unvalidated'])
 out=ROOT/'artifacts/reports/authority-20261021';out.mkdir(parents=True,exist_ok=True);(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
 with (d/'flight.csv').open() as f:rows=list(csv.DictReader(f))
 def col(k):return np.array([float(r[k]) for r in rows])
 ts=col('time_s');plt.style.use('dark_background');fig,axs=plt.subplots(2,2,figsize=(14,9));ax=axs[0,0];ax.plot(col('x'),col('y'),label='Gazebo trajectory');pts=np.array(route['points']);ax.plot(pts[:,0],pts[:,1],'o--',alpha=.5,label='Approved nodes')
 for i,pnt in enumerate(pts):ax.annotate(str(i+1),pnt[:2],xytext=(6,6),textcoords='offset points')
 for x,y,yaw in e['environment']['panels']:
  ax.plot([x+.6*math.sin(yaw),x-.6*math.sin(yaw)],[y-.6*math.cos(yaw),y+.6*math.cos(yaw)],color='orange',lw=5)
 ax.set(xlabel='World x (m)',ylabel='World y (m)',title='Native trajectory; orange = true panel positions for evaluation');ax.axis('equal');ax.legend()
 axs[0,1].plot(ts,col('z')-.082,label='True relative height');axs[0,1].plot(ts,col('estimated_z'),label='C estimate');axs[0,1].set(ylabel='Height (m)',xlabel='Simulation seconds');axs[0,1].legend()
 for k in ('roll_deg','pitch_deg'):axs[1,0].plot(ts,col(k),label=k)
 axs[1,0].set(ylabel='Angle (degrees)',xlabel='Simulation seconds');axs[1,0].legend()
 for k in ('FL','FR','RR','RL'):axs[1,1].plot(ts,col(k),label=k,alpha=.7)
 axs[1,1].set(ylabel='Applied motor fraction',xlabel='Simulation seconds');axs[1,1].legend()
 for ax in axs.flat:ax.grid(alpha=.15)
 fig.suptitle('Seven-node onboard ROS2 → C FC → Gazebo proof · synthetic localization / assumed model');fig.tight_layout();fig.savefig(out/'trajectory-and-control.png',dpi=140);plt.close(fig)
 entries=''.join('<li>'+('PASS' if v else 'FAIL')+' · '+k.replace('_',' ')+'</li>' for k,v in checks.items());video='../../gazebo-suite/'+a.run+'/gazebo.mp4'
 html=f'''<!doctype html><html><meta charset="utf-8"><title>IntelQCopter authority evidence</title><style>body{{max-width:1200px;margin:35px auto;font:17px/1.6 system-ui;background:#102234;color:#e1edf5;padding:20px}}a{{color:#63dfc3}}img,video{{width:100%}}pre{{white-space:pre-wrap}}li{{margin:5px}}</style><h1>Who actually flew the route?</h1><p>Seven approved nodes, one PC request, neutral pilot after selection. The onboard ROS2 C++ mission/planner generated the commands consumed by the actual C FC host build.</p><p>Native Gazebo / OGRE2 · perfect simulated localization · synthetic sensors/calibration · assumed mass/inertia/thrust. No physical-flight validation.</p><h2>{'PASS' if summary['passed'] else 'FAIL'} · {t['simulated_seconds']:.2f} simulated seconds · {error*1000:.1f} mm final XY error · {e['obstacle_contact_steps']} obstacle contact steps</h2><video controls preload="metadata" src="{video}"></video><img src="trajectory-and-control.png"><h2>Wire and authority audit</h2><ul>{entries}</ul><p>Negative control: deliberately removing companion frames prevented completion and the FC marked them stale. That is an expected failed flight, retained separately.</p><p><a href="summary.json">Machine-readable audit and assumptions</a> · <a href="../../gazebo-suite/{a.run}/wire-inputs.jsonl">Raw input frames</a> · <a href="../../gazebo-suite/{a.run}/proof-pc.jsonl">PC requests, neutral HID and DDS publisher identity</a> · <a href="../../gazebo-suite/{a.run}/drone-mission.log">Original onboard mission log</a> · <a href="../../gazebo-suite/{a.run}/authority-manifest.json">Source/binary hashes and process commands</a></p><h2>Manual authority check</h2><p><a href="../../gazebo-suite/authority-real-manual-flight-20261021/gazebo.mp4">Original human-input replay video</a> · <a href="../../gazebo-suite/authority-real-manual-flight-20261021/manual-audit.json">Manual wire audit</a>. This replay uses the same C FC binary, no ROS2, no companion frames and no pose-feedback pilot. It is a 45-second open-loop replay, not a new live human flight or a successful landing test. It can drift horizontally; there is no hidden position hold.</p><h2>Limits of this evidence</h2><p>The software structure and recorded wire show which process supplied commands. Synthetic odometry still makes localization easier than real flight; known-panel PnP is not general obstacle ranging. Old automated manual trials used a labelled feedback-based virtual human; they are not presented as human input. Real manual trials retain actual XInput logs and use the same C motor path, with no companion in standalone mode.</p></html>'''
 (out/'index.html').write_text(html,encoding='utf8');print(json.dumps(summary,indent=2));assert summary['passed']
if __name__=='__main__':main()
