# Isaac Sim 6.2.0 shared install record

This is an install record, not a simulator run log.

- Source: NVIDIA's official Windows standalone package, [`isaac-sim-standalone-6.2.0-windows-x86_64.zip`](https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.2.0-windows-x86_64.zip).
- Official release page checksum: `86dabd127e9e8c225c239b41a8e3909b` (MD5). `certutil -hashfile ... MD5` returned the same value; download and extraction commands exited successfully.
- Shared extracted directory: `C:\Users\Public\Documents\IsaacSim\app`.
- Package VERSION: `6.2.0-rc.14+release.52805.02189a84.gl`.
- `isaac-sim.bat` and `post_install.bat` are present. NVIDIA's post-install script could not create its optional symbolic link because this account lacks symbolic-link privilege; the same examples directory was linked with a Windows junction.
- Isaac Sim was **not launched**. The workstation is Windows 10 with an RTX 3050 Laptop GPU and 4 GB VRAM; Isaac Sim 6.2.0 lists Windows 11 and an RTX 4080 / 16 GB VRAM minimum. See NVIDIA's [requirements](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html).
- The downloaded ZIP was removed after checksum verification and extraction to recover disk space. The extracted shared application remains installed.

No quadcopter stage was loaded and no physics simulation, ROS 2 session, graph, or video was produced by Isaac Sim.
