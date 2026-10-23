# IntelQCopter — stepwise ESP32-S3 rebuild

This branch starts with hardware diagnostics. **There is no flight controller or motor-output driver yet.**

Target: ESP32-S3, Arduino framework / PlatformIO, MPU6050 and BMP280 first; Xbox next. The RP2350/RP2040 project stays on `main`. The earlier ESP32 project and PCB stay on `esp32-xbox`, with baseline commit `c194112` in this branch's ancestry.

Start with [the plan](docs/PLAN.md), then [the bench checklist](docs/bench-tests.md). Current acceptance state lives in [the stage catalogue](validation/stages.json) and immutable validation events, not in conversational claims.

## Working method

- Implement one small capability, commit it, compile/check it, retain original output.
- Real hardware acceptance is a separate action with an operator, hardware identity, measurements and evidence.
- Builds and simulations never stand in for a physical sensor test.
- Later code may be prepared offline. It remains unvalidated until its prerequisites pass.
- Failure blocks dependent steps. Fix the earliest failure; revalidate it and each affected descendant in order.
- Preserve failed attempts and old commits. Remove rejected later code from the active implementation with normal new commits or start a repair branch from the last accepted checkpoint. Do not rewrite or force-push shared history.

## Folder map

| Folder | Purpose |
|---|---|
| `firmware/` | One Arduino-style `main.cpp` for the current bench step |
| `docs/` | Plan and procedures |
| `config/` | Explicit hardware assumptions and bench identity |
| `validation/` | Stage dependencies and immutable acceptance history |
| `artifacts/` | Original build logs, serial captures and measurements |
| `tools/` | Small build, capture and validation tools |
| `important_sources/` | Links to canonical current files, no duplicate implementations |

Use `project.json` and `qcopter.ps1` for commands. Do not flash automatically; select the actual COM port deliberately. Physical motor arming remains disabled.
