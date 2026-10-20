"""Real YOLO inference on an external reference image, separate from navigation."""
import hashlib,json,time
from pathlib import Path
from ultralytics import YOLO

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'artifacts/simulation/2026-10-20/perception'
OUT.mkdir(parents=True,exist_ok=True)
if __name__=='__main__':
    import urllib.request
    fixtures=ROOT/'build/perception';fixtures.mkdir(parents=True,exist_ok=True)
    image=fixtures/'bus.jpg';weights=fixtures/'yolo11n.pt'
    if not image.exists():urllib.request.urlretrieve('https://raw.githubusercontent.com/ultralytics/assets/main/im/bus.jpg',image)
    # Ultralytics downloads the official weights into this explicit project path.
    model=YOLO(str(weights));model.predict(str(image),device=0,verbose=False)
    start=time.perf_counter();r=model.predict(str(image),device=0,verbose=False)[0]
    elapsed=time.perf_counter()-start
    r.save(str(OUT/'yolo-reference.jpg'))
    detections=[{'class':r.names[int(b.cls.item())],'confidence':float(b.conf.item()),
                 'xyxy':b.xyxy[0].tolist()} for b in r.boxes]
    report={'test':'actual GPU YOLO inference, not flight perception','weights':'Ultralytics YOLO11n',
      'weights_sha256':hashlib.sha256(weights.read_bytes()).hexdigest(),
      'image_source':'https://raw.githubusercontent.com/ultralytics/assets/main/im/bus.jpg',
      'image_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),
      'elapsed_warm_s':elapsed,'ultralytics_timing_ms':r.speed,'detections':detections}
    (OUT/'yolo-result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report),flush=True)
    if not any(d['class']=='bus' for d in detections):raise SystemExit('Reference bus not detected')
