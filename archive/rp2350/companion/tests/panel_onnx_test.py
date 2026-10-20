"""Execute the ROS node's 320px OpenCV raw-output contract, without ROS."""
from pathlib import Path
import hashlib, json
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
model=ROOT/'build/panel-training/rgb-panel/weights/best.onnx'
frame=cv2.imread(str(ROOT/'artifacts/simulation/2026-10-20-missions/garden-final-camera-12s.png'))
assert frame is not None
side=320; scale=min(side/frame.shape[1],side/frame.shape[0])
w,h=round(frame.shape[1]*scale),round(frame.shape[0]*scale)
left,top=(side-w)//2,(side-h)//2
image=np.full((side,side,3),114,np.uint8)
image[top:top+h,left:left+w]=cv2.resize(frame,(w,h))
net=cv2.dnn.readNetFromONNX(str(model))
net.setInput(cv2.dnn.blobFromImage(image,1/255,(side,side),swapRB=True,crop=False))
raw=net.forward()
assert raw.shape==(1,5,2100) and np.isfinite(raw).all()
rows=raw[0].T; boxes=[]; scores=[]
for x,y,w,h,confidence in rows:
    if confidence<.35 or w<=0 or h<=0: continue
    boxes.append([int(x-w/2),int(y-h/2),int(w),int(h)]);scores.append(float(confidence))
kept=cv2.dnn.NMSBoxes(boxes,scores,.35,.45)
assert len(kept)>0
result={'opencv':cv2.__version__,'shape':list(raw.shape),'retained_detections':len(kept),
        'maximum_confidence':max(scores),'onnx_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),
        'scope':'Actual CPU OpenCV inference on recorded RGB; not a compiled ROS C++ wrapper test.'}
(ROOT/'artifacts/simulation/2026-10-20-missions/panel-onnx-test.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
