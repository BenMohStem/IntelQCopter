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
