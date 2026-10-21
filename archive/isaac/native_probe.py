"""Native PhysX drop test and low-cost actual camera test in Isaac Kit."""
import asyncio,json,os,struct,zlib
from pathlib import Path
import omni.kit.app,omni.usd
import carb

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(os.environ['QCOPTER_ISAAC_PROBE_DIR'])

async def main():
    app=omni.kit.app.get_app();result={'engine':'Isaac Sim 6.2 / PhysX','physics_success':False,'renderer_success':False}
    await omni.usd.get_context().open_stage_async(str(ROOT/'sim/isaac/assets/qcopter_test.usda'))
    print('NATIVE_PROBE: stage opened',flush=True)
    try:
        from pxr import UsdGeom,Gf
        stage=omni.usd.get_context().get_stage()
        stage.SetStartTimeCode(0);stage.SetEndTimeCode(60000)
        UsdGeom.Xformable(stage.GetPrimAtPath('/World/Drone')).GetOrderedXformOps()[0].Set(Gf.Vec3d(0,0,1.5))
        from isaacsim.core.simulation_manager import SimulationManager as SM
        SM.switch_physics_engine('physx');SM.setup_simulation(dt=.002,device='cpu')
        print('NATIVE_PROBE: initialize physics',flush=True);SM.initialize_physics()
        view=SM.get_physics_sim_view().create_rigid_body_view('/World/Drone')
        start=float(view.get_transforms()[0,2]);samples=[]
        for i in range(1000):
            SM.step();samples.append([i*.002,float(view.get_transforms()[0,2])])
        result.update(initial_height_m=start,final_height_m=samples[-1][1],steps=len(samples))
        # The authored USD collider is 0.08 m high: its COM rests at 0.04 m.
        result['physics_success']=start>1.4 and .03<samples[-1][1]<.06
        (OUT/'physics-trace.json').write_text(json.dumps(samples))
        print('NATIVE_PROBE: physics result',result,flush=True)
    except Exception as error:
        result['physics_error']=repr(error);print('NATIVE_PROBE: physics error',repr(error),flush=True)
    (OUT/'probe-result.json').write_text(json.dumps(result,indent=2))
    try:
        from isaacsim.core.rendering_manager import RenderingManager
        RenderingManager.set_dt(1/60)
        settings=carb.settings.get_settings();settings.set('/rtx/rendermode','MinimalRendering');settings.set('/rtx/minimal/mode',3);settings.set('/rtx/post/aa/op',0)
        import omni.replicator.core as rep
        from omni import timeline
        product=rep.create.render_product('/World/Overview',(320,240));product.hydra_texture.set_updates_enabled(True)
        rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product])
        timeline.get_timeline_interface().play()
        for _ in range(20):await app.next_update_async()
        for i in range(5):
            print('NATIVE_PROBE: capture',i,flush=True)
            await rep.orchestrator.step_async(rt_subframes=1,pause_timeline=False)
            data=rgb.get_data()
            if getattr(data,'size',0):break
            for _ in range(10):await app.next_update_async()
        result['rgb_shape']=list(data.shape)
        if data.ndim!=3 or data.shape[:2]!=(240,320):raise RuntimeError('Empty RGB after native clock setup')
        image=data[:,:,:3].astype('uint8');payload=b''.join(b'\0'+row.tobytes() for row in image)
        def chunk(kind,value):return struct.pack('!I',len(value))+kind+value+struct.pack('!I',zlib.crc32(kind+value)&0xffffffff)
        png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',320,240,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(payload))+chunk(b'IEND',b'')
        (OUT/'isaac-camera.png').write_bytes(png);result['renderer_success']=True
    except Exception as error:
        result['renderer_error']=repr(error);print('NATIVE_PROBE: capture error',repr(error),flush=True)
    (OUT/'probe-result.json').write_text(json.dumps(result,indent=2));print('NATIVE_PROBE:',result,flush=True)
    app.post_quit(0 if result['physics_success'] and result['renderer_success'] else 2)

asyncio.ensure_future(main())
