"""Metric depth is required: YOLO labels alone do not supply obstacle distance."""
import numpy as np

def obstacle_points(depth_m, intrinsics, stride=8, near=.2, far=8.):
    """All valid depth pixels, including objects YOLO does not recognize.

    Returns optical coordinates x right, y down, z forward. Transform using the
    calibrated camera extrinsic and synchronized local pose before map insertion.
    Caller owns age checks, voxelization, map clearing and sensor health handling.
    """
    fx,fy,cx,cy=intrinsics
    if fx<=0 or fy<=0: raise ValueError('Invalid focal length')
    v,u=np.mgrid[0:depth_m.shape[0]:stride,0:depth_m.shape[1]:stride]
    z=depth_m[::stride,::stride]
    valid=np.isfinite(z)&(z>=near)&(z<=far)
    return np.column_stack((((u-cx)*z/fx)[valid],((v-cy)*z/fy)[valid],z[valid]))

def box_depth(depth_m, xyxy):
    """Median of valid central bbox depths for semantic annotation, not safety."""
    x1,y1,x2,y2=map(int,xyxy);w=x2-x1;h=y2-y1
    if w<=0 or h<=0:return None
    crop=depth_m[max(0,y1+h//4):min(depth_m.shape[0],y2-h//4),
                 max(0,x1+w//4):min(depth_m.shape[1],x2-w//4)]
    valid=crop[np.isfinite(crop)&(crop>.2)&(crop<8)]
    return float(np.median(valid)) if valid.size>=8 else None
