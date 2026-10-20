"""Execute OpenCV's ONNX engine and the preprocessing used by the C++ ROS node."""
import cv2,json,time,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'artifacts/simulation/2026-10-20/perception'

def main():
    image=cv2.imread(str(ROOT/'build/perception/bus.jpg'));h,w=image.shape[:2]
    scale=min(640/w,640/h);nw,nh=round(w*scale),round(h*scale);left,top=(640-nw)//2,(640-nh)//2
    padded=np.full((640,640,3),114,np.uint8);padded[top:top+nh,left:left+nw]=cv2.resize(image,(nw,nh))
    model=ROOT/'build/perception/yolo11n.onnx';net=cv2.dnn.readNetFromONNX(str(model))
    blob=cv2.dnn.blobFromImage(padded,1/255.,(640,640),swapRB=True)
    net.setInput(blob);start=time.perf_counter();raw=net.forward();elapsed=time.perf_counter()-start
    assert raw.shape==(1,84,8400)
    boxes=[];scores=[];ids=[]
    for p in raw[0].T:
        cid=int(np.argmax(p[4:]));conf=float(p[4+cid])
        if conf<.35:continue
        x1=int(np.clip((p[0]-p[2]/2-left)/scale,0,w));y1=int(np.clip((p[1]-p[3]/2-top)/scale,0,h))
        x2=int(np.clip((p[0]+p[2]/2-left)/scale,0,w));y2=int(np.clip((p[1]+p[3]/2-top)/scale,0,h))
        if x2<=x1 or y2<=y1:continue
        boxes.append([x1,y1,x2-x1,y2-y1]);scores.append(conf);ids.append(cid)
    indices=cv2.dnn.NMSBoxes(boxes,scores,.35,.45).flatten()
    detections=[{'class_id':ids[i],'confidence':scores[i],'xywh':boxes[i]} for i in indices]
    assert any(d['class_id']==5 and d['confidence']>.8 for d in detections) # COCO bus
    for d in detections:
        x,y,bw,bh=d['xywh'];cv2.rectangle(image,(x,y),(x+bw,y+bh),(0,220,0),2)
        cv2.putText(image,f"class {d['class_id']} {d['confidence']:.2f}",(x,max(20,y)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,100,255),2)
    cv2.imwrite(str(OUT/'opencv-onnx-reference.jpg'),image)
    report={'engine':'actual OpenCV DNN CPU; same letterbox/raw-output/NMS contract as C++ source',
      'onnx_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),'shape':list(raw.shape),
      'inference_s':elapsed,'detections':detections,'ROS_cpp_wrapper_executed':False}
    (OUT/'onnx-result.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report))

if __name__=='__main__':main()
