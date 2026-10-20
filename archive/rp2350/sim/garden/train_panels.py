"""Synthetic panel learning on separate seeds/poses; no runtime segmentation labels."""
import os,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
os.environ['YOLO_CONFIG_DIR']=str(ROOT/'build/yolo-settings')
import cv2,numpy as np
from world import Garden
from ultralytics import YOLO

def main():
    config=json.loads((ROOT/'sim/model/simulation_parameters.json').read_text())
    dataset=ROOT/'build/panel-dataset';dataset.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(9201);world=Garden(config)
    # Label generation may access renderer IDs. No test/navigation code may do so.
    for split,count in [('train',180),('val',40)]:
        for directory in ['images','labels']:(dataset/directory/split).mkdir(parents=True,exist_ok=True)
        for i in range(count):
            pos=np.array([rng.uniform(-1.8,2.2),rng.uniform(-2.4,2.4),rng.uniform(.45,2.4)])
            target=np.array([3.2+rng.uniform(-.5,.5),rng.uniform(-.6,.6),rng.uniform(.7,1.3)])
            front=(target-pos);front/=np.linalg.norm(front);left=np.cross([0,0,1],front);left/=np.linalg.norm(left);up=np.cross(front,left)
            rgb,ids=world.camera(pos,np.column_stack([front,left,up]))
            labels=[]
            for body in world.panels:
                yy,xx=np.where((ids&((1<<24)-1))==body)
                if len(xx)<90:continue
                x1,x2=xx.min(),xx.max();y1,y2=yy.min(),yy.max()
                labels.append(f'0 {(x1+x2)/640:.7f} {(y1+y2)/480:.7f} {(x2-x1)/320:.7f} {(y2-y1)/240:.7f}')
            cv2.imwrite(str(dataset/'images'/split/f'{i:04}.jpg'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            (dataset/'labels'/split/f'{i:04}.txt').write_text('\n'.join(labels),encoding='utf-8')
    world.close()
    spec=dataset/'panels.yaml';spec.write_text(f'path: {dataset.as_posix()}\ntrain: images/train\nval: images/val\nnames:\n  0: panel\n',encoding='utf-8')
    model=YOLO(str(ROOT/'build/perception/yolo11n.pt'))
    model.train(data=str(spec),epochs=12,imgsz=320,batch=8,workers=0,device=0,amp=False,
      project=str(ROOT/'build/panel-training'),name='rgb-panel',exist_ok=True,seed=9201,
      plots=True,verbose=False,cache=False,mosaic=.5,close_mosaic=2)

if __name__=='__main__':main()
