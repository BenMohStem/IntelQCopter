"""Native PhysX + Isaac RGB products synchronized to the C flight-loop client."""
import asyncio,json,os
from pathlib import Path
from multiprocessing.connection import Listener
import numpy as np
import carb,omni.kit.app,omni.usd

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(os.environ['QCOPTER_ISAAC_PROBE_DIR'])

async def main():
    app=omni.kit.app.get_app();listener=None
    try:
        await omni.usd.get_context().open_stage_async(str(ROOT/'sim/isaac/assets/garden_native.usda'))
        from isaacsim.core.simulation_manager import SimulationManager as SM
        from isaacsim.core.rendering_manager import RenderingManager
        from omni import timeline
        import omni.replicator.core as rep
        SM.switch_physics_engine('physx');SM.setup_simulation(dt=.002,device='cpu');RenderingManager.set_dt(.002)
        timeline.get_timeline_interface().play()
        for _ in range(3):await app.next_update_async()
        SM.enable_fabric(False);SM.initialize_physics()
        tensors=SM.get_physics_sim_view();body=tensors.create_rigid_body_view('/World/drone')
        contact=tensors.create_rigid_contact_view('/World/drone',[
            '/World/ground/body/collision','/World/panel0/body/collision','/World/panel1/body/collision'])
        settings=carb.settings.get_settings();settings.set('/app/player/playSimulations',False)
        settings.set('/app/useFabricSceneDelegate',False)
        settings.set('/rtx/rendermode','MinimalRendering');settings.set('/rtx/minimal/mode',3)
        products=[];annotators={}
        for topic,path,size in [('/drone/rgb','/World/drone/body/rgb',(320,240)),('/overview/rgb','/World/overview/camera/camera',(640,360))]:
            product=rep.create.render_product(path,size);products.append(product)
            annotator=rep.AnnotatorRegistry.get_annotator('rgb');annotator.attach([product]);annotators[topic]=annotator
        async def render():
            # Publish the native solver's transforms to USD before camera
            # capture. Without writeback a nested camera can stay at spawn.
            import omni.physx
            omni.physx.get_physx_interface().update_transformations(False,True,True,False)
            await rep.orchestrator.step_async(rt_subframes=1,delta_time=0.0,pause_timeline=False)
        for _ in range(3):await render()
        def state():
            transform=body.get_transforms()[0].copy();velocity=body.get_velocities()[0].copy()
            matrix=contact.get_contact_force_matrix(.002)
            names=['ground','panel0','panel1'];contacts=[('drone',names[i]) for i in range(3) if np.linalg.norm(matrix[0,i])>.01]
            return {'position':transform[:3].tolist(),'quaternion':transform[3:].tolist(),
                    'velocity':velocity[:3].tolist(),'omega':velocity[3:].tolist(),'contacts':contacts}
        initial=state();print('ISAAC native engine ready',initial,flush=True)
        (OUT/'engine-ready.json').write_text(json.dumps(initial,indent=2))
        listener=Listener(('127.0.0.1',9251),authkey=b'intelqcopter-local-engine')
        # Run physics RPCs synchronously on Kit's main thread. Yield only for
        # requested camera renders, avoiding an automatic render at every 2ms
        # control request. This is a headless, single-client experiment.
        client=listener.accept();count=0;last_capture=-100
        with client:
            while True:
                message=client.recv();op=message['op']
                if op=='close':client.send({'steps':count});break
                if op=='step':
                    q=body.get_transforms()[0,3:].copy();x,y,z,w=q
                    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
                    force=(r@np.array(message['force'])+message['drag']).astype(np.float32).reshape(1,3)
                    torque=(r@np.array(message['torque'])).astype(np.float32).reshape(1,3)
                    body.apply_forces_and_torques_at_position(force,torque,None,np.array([0],np.uint32),True)
                    SM.step();count+=1
                    if count%500==0:
                        progress=state();progress['controlled_steps']=count
                        (OUT/'engine-progress.json').write_text(json.dumps(progress,indent=2))
                if op=='camera':
                    if last_capture!=count:await render();last_capture=count
                    image=annotators[message['topic']].get_data()
                    if image.ndim!=3:raise RuntimeError('Isaac RGB buffer became empty')
                    client.send((image.shape[1],image.shape[0],np.ascontiguousarray(image[:,:,:3]).tobytes()));continue
                client.send(state())
        (OUT/'engine-complete.json').write_text(json.dumps({'controlled_steps':count,'native_physics':'PhysX CPU','native_cameras':'Isaac MinimalRendering','physical_arming':False},indent=2))
    except Exception as error:
        import traceback
        traceback.print_exc();(OUT/'engine-error.txt').write_text(repr(error));print('ISAAC_ENGINE_ERROR',repr(error),flush=True)
    finally:
        if listener:listener.close()
        app.post_quit()

asyncio.ensure_future(main())
