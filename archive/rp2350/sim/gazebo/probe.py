"""Real Gazebo gravity/contact test; does not substitute for a flight test."""
import json,sys,time
from pathlib import Path
import gz.math as math
import gz.sim as sim
from build_world import build,ROOT
OUT=ROOT/'artifacts/test-runs/2026-10-20-engines';OUT.mkdir(parents=True,exist_ok=True)
path=build(ROOT/'build/gazebo/drop.sdf',drop_test=True)
fixture=sim.TestFixture(str(path));rows=[];link=None
def pre(info,ecm):
    global link
    if link is None:
        world=sim.World(sim.world_entity(ecm));model=sim.Model(world.model_by_name(ecm,'drone'))
        entity=model.link_by_name(ecm,'body')
        if entity:link=sim.Link(entity);link.enable_velocity_checks(ecm,True)
def post(info,ecm):
    if link is not None:
        pose=link.world_pose(ecm)
        if pose:rows.append([len(rows)*.002,pose.pos().z()])
fixture.on_pre_update(pre);fixture.on_post_update(post);fixture.finalize()
print('Starting native Gazebo gravity test',flush=True);start=time.perf_counter()
ok=fixture.server().run(True,1000,False)
result={'engine':'Gazebo 10.5.0','server_return':ok,'samples':len(rows),'wall_s':time.perf_counter()-start,
        'initial_height_m':rows[0][1] if rows else None,'final_height_m':rows[-1][1] if rows else None,
        'scope':'Gravity/contact only; no flight controller or RGB avoidance proof'}
result['success']=bool(ok and len(rows)>900 and rows[0][1]>1.4 and .06<rows[-1][1]<.12)
(OUT/'gazebo-gravity.json').write_text(json.dumps(result,indent=2));(OUT/'gazebo-gravity-trace.json').write_text(json.dumps(rows))
print(json.dumps(result,indent=2),flush=True)
sys.exit(0 if result['success'] else 2)
