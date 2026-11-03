"""Smoke-test the numeric 3D magnetometer coordinate grid without serial hardware.

Run `py -3 tools/test_mag_3d_grid.py` from project root.
"""
import math
from pathlib import Path
import numpy as np
from matplotlib.figure import Figure
from live_calibration_gui import magnetic_axis_limits, configure_magnetic_axes
from calibrate import magnetic_orientation_coverage

points=[(-91.5,-73.8,-57.2),(19.5,32.5,2.1),(-36,-23,-28)]
limits=magnetic_axis_limits(points)
span=[hi-lo for lo,hi in limits]
assert np.allclose(span,span[0]), limits
fig=Figure(figsize=(6,5),dpi=100)
ax=fig.add_subplot(111,projection='3d')
assert len(configure_magnetic_axes(ax,points,enlarged=True))==6
fig.canvas.draw()
assert ax.get_xlabel().startswith('X') and ax.get_ylabel().startswith('Y')
assert any(axis.pane.get_facecolor()[-1] for axis in (ax.xaxis,ax.yaxis,ax.zaxis))
# Nearly uniform directional grid should have high coverage.
phi=np.linspace(0,2*math.pi,45,endpoint=False)
theta=np.linspace(0.01,math.pi-.01,25)
uv=np.asarray([(math.sin(t)*math.cos(p),math.sin(t)*math.sin(p),math.cos(t))
               for t in theta for p in phi])
report=magnetic_orientation_coverage(uv)
assert report['coverage_fraction_within_18_degrees']>=.98, report
assert sum(report['octant_counts_sensor_frame'].values())==len(uv)
print('PASS: equal 3D axis scale, readable grid panes/ticks, directional triad and coverage metrics')
