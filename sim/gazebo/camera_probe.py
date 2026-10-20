"""Capture actual Gazebo sensor images; no substitute renderer."""
import json,time
from pathlib import Path
import gz.math
import gz.sim as sim
from gz.transport import Node
from gz.msgs.image_pb2 import Image
from build_world import build,ROOT
OUT=ROOT/'artifacts/test-runs/2026-10-20-engines'
frames={};node=Node()
def capture(topic):
    def receive(message):
        frames[topic]=(message.width,message.height,message.pixel_format_type,bytes(message.data))
    return receive
callbacks=[capture('/drone/rgb'),capture('/overview/rgb')]
for topic,callback in zip(['/drone/rgb','/overview/rgb'],callbacks):
    assert node.subscribe(Image,topic,callback)
fixture=sim.TestFixture(str(build()));fixture.finalize();server=fixture.server()
start=time.perf_counter();iteration=0
while time.perf_counter()-start<45 and len(frames)<2:
    assert server.run(True,100,False);iteration+=100;time.sleep(.05)
for topic,(width,height,fmt,pixels) in frames.items():
    if len(pixels)==width*height*3:
        (OUT/('gazebo-'+topic.split('/')[1]+'.ppm')).write_bytes(f'P6\n{width} {height}\n255\n'.encode()+pixels)
report={'engine':'Gazebo OGRE2 camera','images':{k:{'width':v[0],'height':v[1],'format':v[2],'bytes':len(v[3])} for k,v in frames.items()},
        'wall_s':time.perf_counter()-start,'iterations':iteration,'success':len(frames)==2}
(OUT/'gazebo-camera.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
raise SystemExit(0 if report['success'] else 2)
