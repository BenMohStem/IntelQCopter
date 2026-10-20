# Original 3D garden evidence — 20 October 2026

Open index.html for the selected video, graphs and links. garden-final is the
current conservative-gain run with final source hashes, corrected panel dimensions,
ROS-selected start/finish and actuator wind-down. Other trials are development
iterations with different layouts/policies; they are not a fair gain ranking.
The failed initial-landing-drift trial is retained (48 cm landing error).

Final: 35.448 s simulated; 8.14 mm XY finish error; 0.490 m minimum evaluated panel
clearance; 17.9 mm cruise height RMSE; no panel collision; ground contact and virtual
motors stopped. One of two panels mapped. 178 actual YOLO frames, 37 accepted PnP
ranges, 1652 downward lidar corrections, 19,378 actual DDS requests.

This is **PyBullet CPU rendering, not Isaac**. Mass/thrust/inertia are assumed.
RGB YOLO + known-size blue fixture PnP is online, with no oracle obstacle map/depth
fed to navigation. Position/orientation and landing contact remain simulated.
No physical flight, general monocular autonomy, full ROS C++ build or 500 Hz hardware
deadline is proven. Hardware arming stays disabled.

Files: final CSV/summary/events/observations are original outputs; metrics/graphs
are derived from them. linear-analysis.json contains analytic/local and finite-grid
checks, not flight trials. Source hashes identify the final experiment; validation
and SHA256 manifest identify the delivered package. Original logs are under
../../test-runs/2026-10-20-missions. See ../../../sim/garden/README.md for reruns.
