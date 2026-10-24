"""Analyze a saved stationary IMU serial log; never open a serial port."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('log', type=Path)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
rows = []
for line in args.log.read_text(encoding='utf-8', errors='replace').splitlines():
    if not line.startswith('IMU,'):
        continue
    cells = next(csv.reader([line]))
    if len(cells) != 8:
        raise SystemExit('Malformed IMU line: preserve the original log and check capture.')
    values = [float(value) for value in cells[1:]]
    if not all(math.isfinite(value) for value in values):
        raise SystemExit('Nonfinite measurement; no noise estimate produced.')
    rows.append(values)
if len(rows) < 2:
    raise SystemExit('Need at least two actual IMU samples; no statistics invented.')
intervals = []
for previous, current in zip(rows, rows[1:]):
    elapsed = (int(current[0]) - int(previous[0])) & 0xffffffff
    if elapsed == 0 or elapsed > 5000000:
        raise SystemExit('Reset, duplicate time, or gap over 5 seconds: select one continuous stationary run.')
    intervals.append(elapsed / 1e6)
channels = {name: [row[i+1] for row in rows] for i,name in enumerate(
    ('ax_drone_m_s2','ay_drone_m_s2','az_drone_m_s2','gx_sensor_rad_s','gy_sensor_rad_s','gz_sensor_rad_s'))}
channels['roll_accel_rad'] = [math.atan2(row[2],row[3]) for row in rows]
channels['pitch_accel_rad'] = [math.atan2(-row[1],math.hypot(row[2],row[3])) for row in rows]
if max(channels['roll_accel_rad']) - min(channels['roll_accel_rad']) > math.pi:
    raise SystemExit('Roll wraps across pi; repeat upright instead of using linear angle statistics.')
stats = {name: dict(mean=statistics.mean(values), variance=statistics.variance(values),
                   standard_deviation=statistics.stdev(values)) for name,values in channels.items()}
roll = channels['roll_accel_rad']; pitch = channels['pitch_accel_rad']
roll_mean = statistics.mean(roll)
pitch_mean = statistics.mean(pitch)
covariance = sum((x-roll_mean)*(y-pitch_mean) for x,y in zip(roll,pitch))/(len(rows)-1)
report = dict(source_sha256=hashlib.sha256(args.log.read_bytes()).hexdigest(), samples=len(rows),
    observed_rate_hz=1/statistics.mean(intervals), min_interval_s=min(intervals),
    max_interval_s=max(intervals), statistics=stats,
    stationary_angle_R_rad2=[[stats['roll_accel_rad']['variance'],covariance],
                           [covariance,stats['pitch_accel_rad']['variance']]],
    status='Candidate statistics from user-selected stationary interval; stationarity not automatically verified',
    caveats=[f'Observed report rate {1/statistics.mean(intervals):.3f} Hz; not verified fresh 500 Hz acquisition',
             'Gyro values remain in sensor coordinates, not drone coordinates',
             'R is stationary observation covariance, not dynamic flight error',
             'No process Q or final Kalman gain calculated',
             'Filtered sample correlation and thermal drift not characterized'])
if args.output.exists():
    raise SystemExit('Output exists; choose a new filename to preserve previous results.')
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
for name,value in stats.items():
    print(f'{name}: mean={value["mean"]:.9g}, stddev={value["standard_deviation"]:.9g}, variance={value["variance"]:.9g}')
print(f'Samples={len(rows)}; observed report rate={report["observed_rate_hz"]:.3f} Hz')
print('Report:',args.output)
