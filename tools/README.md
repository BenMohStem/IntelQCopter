# Tools by purpose

| Folder | Responsibility |
|---|---|
| build/ | Pico firmware, host shared libraries and ROS C++ package builds |
| simulation/ | Native garden launch, RViz model viewer and viewer diagnostics |
| model/ | F450 model authoring, linear design and uncertainty calculations |
| docs/ | Offline mathematical guide renderer |
| reports/ | Graph/dashboard generation and native run reports |
| ros2/ | Windows Jazzy environment entry point |
| validation/ | Existing firmware, companion and saved-result checks |
| setup/ | Reusable Pico SDK/Pixi setup |

Use `qcopter.ps1 help` from the repository root to discover current commands.
Arguments following a command are passed to the underlying script. Historical
one-off scripts are indexed in archive/README.md. Build products stay in build/;
original evidence stays in artifacts/. Python environments are local dependencies
and are not moved as part of cache cleanup because their launchers embed paths.
