"""Native Gazebo physics/sensors, synchronized to a local flight-loop client."""
import json,time
from multiprocessing.connection import Listener
import gz.math as gm
import gz.sim as sim
from gz.transport import Node
from gz.msgs.image_pb2 import Image
from gz.msgs.contacts_pb2 import Contacts
from build_world import build,ROOT

frames={};contacts=[];contact_time=-99.;entity_names={};node=Node();callbacks=[];reported_contact=False
for topic in ['/drone/rgb','/overview/rgb']:
    def receive(message,topic=topic):
        frames[topic]=(message.width,message.height,bytes(message.data))
    callbacks.append(receive);assert node.subscribe(Image,topic,receive)
def receive_contacts(message):
    global contacts,contact_time,reported_contact
    contact_time=message.header.stamp.sec+message.header.stamp.nsec*1e-9
    contacts=[(entity_names.get(c.collision1.id,c.collision1.name),entity_names.get(c.collision2.id,c.collision2.name)) for c in message.contact]
    if message.contact and not reported_contact:
        print('First native contact:',str(message),flush=True);reported_contact=True
callbacks.append(receive_contacts);assert node.subscribe(Contacts,'/drone/contact',receive_contacts)
fixture=sim.TestFixture(str(build()));body=None;state=None
force=[0.,0.,0.];torque=[0.,0.,0.];drag=[0.,0.,0.]
def pre(info,ecm):
    global body
    if body is None:
        world=sim.World(sim.world_entity(ecm));model=sim.Model(world.model_by_name(ecm,'drone'))
        entity=model.link_by_name(ecm,'body')
        if entity:
            body=sim.Link(entity);body.enable_velocity_checks(ecm,True)
            for name in ['ground','panel0','panel1','drone']:
                other=sim.Model(world.model_by_name(ecm,name));other_link=sim.Link(other.link_by_name(ecm,'body'))
                entity_names[other_link.collision_by_name(ecm,'collision')]=name
    if body is not None:
        pose=body.world_pose(ecm)
        if pose:
            f=pose.rot().rotate_vector(gm.Vector3d(*force))+gm.Vector3d(*drag)
            moment=pose.rot().rotate_vector(gm.Vector3d(*torque))
            body.add_world_wrench(ecm,f,moment)
def post(info,ecm):
    global state
    if body is not None:
        pose=body.world_pose(ecm);v=body.world_linear_velocity(ecm);w=body.world_angular_velocity(ecm)
        if pose and v is not None and w is not None:
            p=pose.pos();q=pose.rot()
            state={'sim_time_s':info.sim_time.total_seconds(),'position':[p.x(),p.y(),p.z()],'quaternion':[q.x(),q.y(),q.z(),q.w()],
                   'velocity':[v.x(),v.y(),v.z()],'omega':[w.x(),w.y(),w.z()]}
fixture.on_pre_update(pre);fixture.on_post_update(post);fixture.finalize();server=fixture.server()
# Initialize camera buffers with the aircraft resting on the ground.
for _ in range(100):
    server.run(True,10,False)
    if state and len(frames)==2:break
    time.sleep(.02)
assert state and len(frames)==2,'Gazebo state/cameras did not initialize'
print('Gazebo live engine ready on localhost:9250; actual physics and RGB sensors',flush=True)
with Listener(('127.0.0.1',9250),authkey=b'intelqcopter-local-engine') as listener:
    with listener.accept() as client:
        count=0
        while True:
            message=client.recv();op=message['op']
            if op=='close':client.send({'steps':count});break
            if op=='step':
                force=message['force'];torque=message['torque'];drag=message['drag']
                assert server.run(True,1,False);count+=1
            if op=='camera':client.send(frames[message['topic']]);continue
            reply=dict(state);reply['contacts']=contacts if 0<=state['sim_time_s']-contact_time<.02 else [];client.send(reply)
print('Gazebo live engine stopped after',count,'controlled physics steps',flush=True)
