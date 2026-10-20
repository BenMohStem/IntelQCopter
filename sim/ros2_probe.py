"""Probe native Jazzy independently of any simulator."""
import os
import sys
from pathlib import Path

prefix = Path(os.environ.get('QCOPTER_ROS_PREFIX', r'C:\dev\jazzy\install'))
os.environ['PATH'] = str(prefix / 'bin') + os.pathsep + os.environ['PATH']
os.environ['AMENT_PREFIX_PATH'] = str(prefix)
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
os.environ.setdefault('ROS_DOMAIN_ID', '64')
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(Path(__file__).with_name('fastdds_udp.xml'))
log_dir = Path(__file__).resolve().parents[1] / 'artifacts/test-runs/2026-10-20/ros-logs'
log_dir.mkdir(parents=True, exist_ok=True)
os.environ['ROS_LOG_DIR'] = str(log_dir)
sys.path.insert(0, str(prefix / 'Lib/site-packages'))
dll_handles = [os.add_dll_directory(str(p)) for p in
               (prefix / 'bin', Path(sys.prefix) / 'Library/bin') if p.is_dir()]

if __name__ == '__main__':
    import time
    import rclpy
    from std_msgs.msg import Float32
    rclpy.init()
    node = rclpy.create_node('qcopter_runtime_probe')
    received = []
    sub = node.create_subscription(Float32, '/qcopter/probe', lambda m: received.append(m.data), 10)
    pub = node.create_publisher(Float32, '/qcopter/probe', 10)
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline and len(received) < 5:
        pub.publish(Float32(data=3.25))
        rclpy.spin_once(node, timeout_sec=0.1)
    print({'middleware': os.environ['RMW_IMPLEMENTATION'], 'received': received}, flush=True)
    node.destroy_node()
    rclpy.shutdown()
    if len(received) < 5:
        raise SystemExit('ROS 2 transport probe failed')
