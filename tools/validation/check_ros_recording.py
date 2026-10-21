"""Actual ROS message types -> diagnostic recorder; no flight commands published."""
import io,json,sys
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulation/gazebo'))
from ros_bridge import Boundary
from qcopter_autonomy.msg import SemanticDetections,SemanticDetection,MetricObstacles
from geometry_msgs.msg import Point

trace=io.StringIO();node=SimpleNamespace(semantics=0,metric_frames=0,trace=trace)
detection=SemanticDetection();detection.class_id=0;detection.confidence=.9
detection.xyxy_pixels=[1.,2.,100.,120.]
message=SemanticDetections();message.detections=[detection]
Boundary.semantic(node,message)
metric=MetricObstacles();metric.header.frame_id='map';metric.image_processed=True
metric.centers=[Point(x=3.,y=.2,z=.9)];metric.radii_m=[.9]
Boundary.metric(node,metric)
records=[json.loads(x) for x in trace.getvalue().splitlines()]
assert len(records)==2 and records[0]['detections'][0]['xyxy']==[1.,2.,100.,120.]
assert abs(records[1]['radii'][0]-.9)<1e-6
print(json.dumps(dict(passed=True,checks=2,scope='Actual generated ROS2 float-array messages serialize to JSON. No flight or hardware exercised.'),indent=2))
