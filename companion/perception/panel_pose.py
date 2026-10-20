"""Single-RGB range for a calibrated, known-size rectangular test fixture.

NOT general monocular obstacle ranging. Reject clipped/uncertain quadrilaterals.
YOLO gates candidate regions; no depth map, renderer IDs or true panel pose used.
"""
import cv2
import numpy as np

def panel_observations(bgr,detections,k,width=1.09,height=1.69):
    result=[];h,w=bgr.shape[:2]
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([95,110,70]),np.array([130,255,255]))
    for bbox,confidence in detections:
        x1,y1,x2,y2=bbox
        roi=np.zeros_like(mask);roi[max(0,int(y1)-5):min(h,int(y2)+6),max(0,int(x1)-5):min(w,int(x2)+6)]=255
        contours,_=cv2.findContours(mask&roi,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in sorted(contours,key=cv2.contourArea,reverse=True)[:2]:
            if cv2.contourArea(contour)<100:continue
            poly=cv2.approxPolyDP(contour,.025*cv2.arcLength(contour,True),True)
            if len(poly)!=4 or not cv2.isContourConvex(poly):continue
            corners=poly.reshape(4,2).astype(np.float64)
            if np.any(corners[:,0]<3)|np.any(corners[:,0]>w-4)|np.any(corners[:,1]<3)|np.any(corners[:,1]>h-4):continue
            ordered=np.zeros((4,2));s=corners.sum(1);d=np.diff(corners,axis=1).ravel()
            ordered[0]=corners[np.argmin(s)];ordered[2]=corners[np.argmax(s)]
            ordered[1]=corners[np.argmin(d)];ordered[3]=corners[np.argmax(d)]
            if len(np.unique(ordered,axis=0))<4:continue
            objects=np.array([[-width/2,-height/2,0],[width/2,-height/2,0],
                              [width/2,height/2,0],[-width/2,height/2,0]],np.float64)
            ok,rvec,tvec=cv2.solvePnP(objects,ordered,k,np.zeros(5),flags=cv2.SOLVEPNP_IPPE)
            if not ok or not np.isfinite(tvec).all() or not np.isfinite(rvec).all() or tvec[2,0]<=.3 or tvec[2,0]>9:continue
            projected,_=cv2.projectPoints(objects,rvec,tvec,k,np.zeros(5))
            error=float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-ordered)**2,axis=1))))
            if error>2.0:continue
            result.append({'position_optical_m':tvec.ravel(),'reprojection_rms_px':error,
                'confidence':confidence,'corners':ordered,'range_sigma_m':float(.04*tvec[2,0]+.03)})
            break
    return result
