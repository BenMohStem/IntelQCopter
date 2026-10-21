# ESP32 architecture migration evidence

- The Git worktree moved from QCopter/build/esp32-xbox to sibling QCopter-ESP32-Xbox.
- QEMU code moved into simulation/emulation; tools are grouped by purpose.
- The active C/C++ firmware sources are unchanged; see firmware-preservation.json.
- The hardware build and emulator build succeed from the new paths; original build logs are retained here.
- `artifacts/emulation/architecture-2026-10-21/` retains the new original QEMU UART log and all 19 passing checks.
- `archive-restore.log` and `.json`: all 755 historical files (117,930,376 bytes) reconstructed from Git and SHA-256 verified.
- `layout-check.json`: current command paths, domains and active script syntax.
- Knowledge schema/history validates after tool moves and immutable Git source references.

The duplicate old snapshot is now a recoverable local cache rather than 755 repeated
tracked payload files. Its inventory and immutable Git reference remain versioned.
The reference Drone source and all original branch logs remain versioned.

Automatic approval review blocked recursive removal of the redundant local
`.cache/archive/rp2350-preserved/` recovery-check copy. It remains ignored locally.
No hardware execution, real Bluetooth pairing or new closed-loop flight is claimed.
