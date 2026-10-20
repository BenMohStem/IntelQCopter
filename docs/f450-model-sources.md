# F450 chassis model and component assumptions

Search found actual F450 CAD assets:

- https://github.com/sinanp1x/f450-drone-cad-stl — F450 STL/CAD components.
- https://github.com/beomsu7/px4-quadrotor-HW-parts — F450/custom assemblies and mounts.

Neither repository declared a license in the checked root/API metadata. Their
meshes have not been copied into this public repository. They are references for
finding a closer CAD asset, not verified dimensional or inertial ground truth.
We have not found and validated an exact, licensed, ready-to-use URDF matching the
user's particular F450 clone and orange 1400KV motors.

An original editable primitive model now lives in:
`companion/ros2_ws/src/qcopter_description/urdf/f450.urdf`.
It includes 450 mm motor diagonal, four explicit motor masses, four 9 inch swept
propeller disks with two-blade visuals, frame/arms, and a separate battery link.
Red arms are drawn forward; confirm the real orientation. A 9047 label supplies
diameter/pitch, not mass, thrust or airfoil shape. CW/CCW propeller pairs must be
confirmed physically; matching diameter/pitch does not mean all four spin the same way.

Edit `sim/model/f450_components.json`, then run
`tools/build_f450_urdf.py` with the simulation Python. The generator uses box and
cylinder inertia, rotates each tensor into the body frame, finds the combined
centre of mass and applies the parallel-axis theorem. The report is
`urdf/mass-properties.json`; it is not a measured inertia tensor.

| Component | Assumed mass |
|---|---:|
| Frame including arms | 330 g |
| Each motor | 55 g |
| Each propeller | 10 g |
| Battery | 150 g (user estimate) |
| Total excluding other parts | 740 g |

Battery dimensions/placement and all masses except the approximate battery mass
are unverified choices. ESC/electronics mass is omitted per request; their real
mass will affect the eventual identified model. Prop inertia is approximated as
a uniform disk. No guessed error bars are presented as measured distributions.

The earlier garden flight's 1.2 kg mass and diagonal inertia (0.012,0.012,0.022)
remain a separate conservative baseline. The new component URDF has not been
used for the recorded flight or its PID tuning. Replace estimates with weighed
parts, measured centres and thrust/current data before treating either model as
the real airframe. The independent URDF load/mass-sum check and rendered model
are under `artifacts/setup/jazzy-repair/f450-urdf-*`.
