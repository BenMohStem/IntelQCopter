"""Run with Kit --exec. Check local USD loading and a real RGB frame first."""
import asyncio
import json
import os
from pathlib import Path
import struct
import zlib

import carb
import omni.kit.app
import omni.usd

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(os.environ.get('QCOPTER_ISAAC_PROBE_DIR',str(ROOT / 'artifacts/test-runs/2026-10-20-isaac-retry')))
OUT.mkdir(parents=True, exist_ok=True)

async def probe():
    app = omni.kit.app.get_app()
    result = {'engine': 'NVIDIA Isaac Sim Kit', 'renderer_success': False,
              'vulkan_setting': carb.settings.get_settings().get('/app/vulkan')}
    try:
        print('QCOPTER_PROBE: Python execution reached', flush=True)
        opened = await omni.usd.get_context().open_stage_async(str(ROOT/'sim/isaac/assets/qcopter_test.usda'))
        result['stage_open_result'] = str(opened)
        print('QCOPTER_PROBE: USD stage open returned', opened, flush=True)
        import omni.replicator.core as rep
        carb.settings.get_settings().set('/rtx/rendermode','RaytracedLighting')
        stage = omni.usd.get_context().get_stage()
        cameras = [str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName() == 'Camera']
        if not cameras:
            raise RuntimeError('USD has no camera')
        product = rep.create.render_product(cameras[0], (320, 240))
        rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb.attach([product])
        # A paused stage needs explicit Replicator capture; app ticks alone can
        # leave its render product unpopulated.
        for _ in range(3):
            await rep.orchestrator.step_async(rt_subframes=4,delta_time=0.0)
        data = rgb.get_data()
        result['rgb_shape'] = list(getattr(data, 'shape', []))
        if tuple(result['rgb_shape'][:2]) != (240, 320) or data.size == 0:
            raise RuntimeError('No populated camera RGB frame')
        image = data[:,:,:3].astype('uint8')
        # PNG encoding without adding packages to the shared Isaac installation.
        def chunk(kind, value):
            return struct.pack('!I',len(value))+kind+value+struct.pack('!I',zlib.crc32(kind+value)&0xffffffff)
        payload = b''.join(b'\0'+row.tobytes() for row in image)
        png = b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',320,240,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(payload))+chunk(b'IEND',b'')
        (OUT/'isaac-camera.png').write_bytes(png)
        result['renderer_success'] = True
        print('QCOPTER_PROBE: actual Isaac RGB capture saved', flush=True)
    except Exception as error:
        result['error'] = repr(error)
        print('QCOPTER_PROBE: failed', repr(error), flush=True)
    finally:
        (OUT/'probe-result.json').write_text(json.dumps(result,indent=2))
        app.post_quit(0 if result['renderer_success'] else 2)

asyncio.ensure_future(probe())
