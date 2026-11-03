#!/usr/bin/env python3
"""Export the audit's per-session numerical matrices to ordinary CSV rows."""
import argparse,csv,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('json_path',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
d=json.loads(a.json_path.read_text())
rows=[]
def emit(session,sensor,part,arr,units):
 if arr is None:return
 if isinstance(arr,list) and arr and isinstance(arr[0],list):
  for i,row in enumerate(arr):
   for j,v in enumerate(row):rows.append([session,sensor,part,f'{i}{j}',v,units])
 else:
  for i,v in enumerate(arr):rows.append([session,sensor,part,str(i),v,units])
for session,detail in d['runs'].items():
 gg=detail.get('rest',{});aa=detail.get('accelerometer_six_face',{});mm=detail.get('magnetometer',{})
 emit(session,'gyro','bias',gg.get('gyro_bias_dps'),'deg/s')
 emit(session,'gyro','sample_std',gg.get('gyro_std_dps'),'deg/s')
 emit(session,'gyro','covariance',gg.get('gyro_covariance_dps2'),'deg²/s²')
 emit(session,'accelerometer','bias',aa.get('recommended_diagonal_bias_mps2'),'m/s²')
 emit(session,'accelerometer','matrix',aa.get('recommended_diagonal_matrix'),'dimensionless')
 emit(session,'accelerometer','full3x3_experimental',aa.get('full3x3_experimental_matrix'),'dimensionless')
 emit(session,'magnetometer','hard_iron_center',mm.get('hard_iron_offset_raw_counts'),'ADC counts')
 emit(session,'magnetometer','symmetric_matrix',mm.get('dimensionless_symmetric_soft_iron_matrix'),'dimensionless')
 emit(session,'magnetometer','unit_sphere_matrix',mm.get('unit_sphere_matrix_inverse_counts'),'1/ADC count')
pm=d.get('magnetometer_pooled_raw_counts',{})
emit('POOLED_MAG','magnetometer','hard_iron_center',pm.get('hard_iron_offset_raw_counts'),'ADC counts')
emit('POOLED_MAG','magnetometer','symmetric_matrix',pm.get('dimensionless_symmetric_soft_iron_matrix'),'dimensionless')
a.output.parent.mkdir(parents=True,exist_ok=True)
with a.output.open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['session','sensor','quantity','index_0_based','value','unit']);w.writerows(rows)
print('Wrote',len(rows),'calibration coefficients:',a.output)
