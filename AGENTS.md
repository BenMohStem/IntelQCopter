# IntelQCopter - ESP32/Xbox branch

- Work on esp32-xbox. Main remains RP2350 flight controller/RP2040 handheld.
- Per the user request of 2026-10-21, keep the ESP32 firmware implementation in
  firmware/src/flight_controller.cpp: C-style control/calibration/estimation and
  only the minimal C++ boundary needed for Arduino/Xbox/sensor libraries.
  fc_api.h contains declarations only; ROS2 keeps its normal package structure.
- Keep centered left-stick vertical-speed/height-hold commands; RT is boost.
- Current facts and assumptions live in knowledge/project.json and docs/status.md.
  Append dated revisions with sources; do not silently replace earlier claims.
  Run tools/knowledge/update_knowledge.py build after changes. SQLite lives in ignored build/knowledge/.
- Distinguish hardware build, QEMU input replay, LTI calculation, flight simulation,
  and physical measurements. New evidence must name its actual scope.
- archive/ is historical evidence; archived launch scripts are not ESP32 tools.
- Do not modify the original Drone folder. Changes to main require a project-wide user request.
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
