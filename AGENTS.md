# IntelQcopter working agreement

- Keep each Pico firmware runtime in its single C file: flight_controller.c and
  handler_controller.c. ROS 2 retains normal package organization.
- Physical motor arming remains disabled until the user changes that instruction.
- Preserve original simulation/emulator logs and failed trials. Identify the actual
  engine, assumed parameters and unvalidated hardware interfaces in every report.
- The user requests a local commit and GitHub push after every prompt that changes
  the project. Use descriptive commits and preserve existing history. Do not make
  empty commits for conversation-only prompts, rewrite history or force-push.
- Before publishing, exclude credentials, local environments, downloaded SDKs and
  unrelated personal/coursework files. Never put authentication tokens in URLs.
- If authentication/network access blocks a push, retain local commits and report
  exactly what remains unpublished. Do not describe local commits as pushed.

- Use project.json and qcopter.ps1 for current command paths. Group tools by purpose.
- Source checkouts must stay outside build/ and .cache/. Preserve original evidence.

- User revision 2026-10-28: motor policy is 10% armed idle, 45% final per-motor normal cap, 65% boost cap. Physical arming is authorized only through a concrete known adapter; unknown RP GPIO/protocol and incomplete adapters stay planned until supplied. Existing sensor/driver validity and stop paths take priority. No flash or physical motor test was requested. See docs/motor-limits-2026-10-28.md.
- User revision 2026-10-28: prefer shared system tools; venvs only for genuine compatibility/isolation needs. Shared native Zig is C:/WinLibs/zig/zig.exe. Preserve custom Gazebo, ROS source, RViz/URDF and original evidence during cleanup.
