"""Package original setup evidence and the successful migrated Gazebo flight."""
from pathlib import Path
import hashlib, importlib.metadata as metadata, json, sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'sim'))
import ros2_probe
import rclpy
OUT=ROOT/'artifacts/setup/jazzy-repair'
run='jazzy-migration-v4-gazebo'
flight=ROOT/'artifacts/simulation/2026-10-20-engines'
summary=json.loads((flight/f'{run}-summary.json').read_text())
validation=json.loads((flight/f'{run}-validation.json').read_text())
rviz=json.loads((OUT/'rviz/result.json').read_text())
cpp_path=next(p for p in sorted(OUT.glob('cpp-integration-*/result.json'),reverse=True)
              if json.loads(p.read_text()).get('passed'))
cpp=json.loads(cpp_path.read_text())
report={
    'isaac_application_removed':not Path('C:/IsaacSim').exists(),
    'ros_python':sys.executable,'rclpy_module':rclpy.__file__,
    'colcon_versions':{name:metadata.version(name) for name in ['colcon-core','colcon-output']},
    'rviz_passed':rviz['passed'],'cpp_integration_passed':cpp['passed'],
    'cpp_result':str(cpp_path.relative_to(OUT)),
    'native_gazebo_mission_passed':validation['success'],
    'hardware_armed':summary['hardware_armed'],
    'limitations':['Assumed 1.2 kg flight baseline; separate provisional 0.740 kg component URDF',
                   'Simulated odometry and known-size panels; downward floor ray is analytic',
                   'Flight uses shared C++ mission core and Python DDS adapters; ROS C++ wrappers tested separately',
                   'No physical companion, camera, UART or lidar has been configured or validated']}
assert all(report[k] for k in ['isaac_application_removed','rviz_passed','cpp_integration_passed','native_gazebo_mission_passed'])
(OUT/'summary.json').write_text(json.dumps(report,indent=2))
url='../../simulation/2026-10-20-engines/'+run
(OUT/'index.html').write_text(f'''<!doctype html><html lang="en"><meta charset="utf-8">
<title>IntelQcopter — standalone Jazzy and Gazebo proof</title>
<style>body{{font:17px system-ui;max-width:1100px;margin:40px auto;padding:0 22px;background:#111923;color:#e8edf5}}a{{color:#8dcaff}}video,img{{max-width:100%;border-radius:10px}}section{{background:#1c2938;padding:22px;margin:22px 0;border-radius:12px}}code{{color:#a9e6ca}}li{{margin:.6em 0}}</style>
<h1>IntelQcopter · Jazzy + Gazebo</h1><p>Executed on Windows 10 · 20 October 2026. Isaac Sim application removed.</p>
<section><h2>Fresh native Gazebo flight</h2><p>35.998 simulated seconds · 2.12 cm landing error · 0.560 m evaluated minimum panel clearance · no collision.</p>
<video controls preload="metadata" poster="{url}-preview.png" src="{url}.mp4"></video>
<p>Recorded simulation time, not a real-time desktop screen capture. Concurrent builds: 60.619 seconds wall time.</p>
<p><a href="{url}-summary.json">Raw summary</a> · <a href="{url}.csv">CSV</a> · <a href="{url}-transport.json">Actual DDS latency</a> · <a href="{url}-validation.json">Source/contact/video checks</a></p>
<img src="{url}-graphs.png" alt="Original flight, filter and covariance graphs"></section>
<section><h2>Actual software checks</h2><ul>
<li>Native Jazzy DDS: <a href="dds-probe.log">five received messages</a>.</li>
<li>RViz: <a href="rviz/rviz2.log">OpenGL 4.6 original log</a>; <a href="rviz/result.json">14 component transforms and running-process checks</a>.</li>
<li>Compiled ROS C++: <a href="{cpp_path.relative_to(OUT).as_posix()}">42 detected/ranged frame messages, default rejection, accepted enabled mission, 40 active setpoints</a>.</li>
<li><a href="rebuild-third.log">Successful RViz build</a> · <a href="companion-build-final.log">Successful companion build</a> · <a href="rosbag-benchmark-build.log">Repaired rosbag benchmark build</a>.</li>
<li><a href="isaac-uninstall.json">Isaac uninstall receipt</a> · <a href="patch-normalization.json">Patch repair hashes</a> · <a href="original-user-build.log">Original supplied build log</a>.</li></ul></section>
<section><h2>Component URDF</h2><img src="f450-urdf-preview.png" alt="Independent render of the original provisional F450 URDF">
<p>Original approximate geometry, not a downloaded manufacturer CAD model. 9047 props, four orange motors, battery, two red/two white arms. All component masses provisional; estimated sum 740 g. The flight above uses the separate 1.2 kg baseline.</p>
<p><a href="f450-urdf-validation.json">Independent URDF load/mass check</a> · <a href="../../../docs/f450-model-sources.md">Model assumptions and F450 CAD references</a></p></section>
<section><h2>Run it</h2><p><code>powershell -ExecutionPolicy Bypass -File tools/run_native_garden.ps1</code></p>
<p>Double-click <code>tools/view_f450_windows.cmd</code> for RViz.</p>
<p><a href="../../../docs/windows-jazzy-setup.md">Setup, repairs and reproducible commands</a> · <a href="../../../docs/companion-computer-setup.md">Pi/Jetson deployment plan</a></p>
<p>Known-size panel course, simulated local pose, analytical downward ray. C++ ROS wrappers were tested separately from the flight. Physical motor arming remains disabled. Real companion/camera/UART integration is pending.</p></section></html>''',encoding='utf-8')
manifest={str(p.relative_to(OUT)):hashlib.sha256(p.read_bytes()).hexdigest()
          for p in OUT.rglob('*') if p.is_file() and p.name not in {'sha256-manifest.json','packaging.log'}}
(OUT/'sha256-manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(report,indent=2))
