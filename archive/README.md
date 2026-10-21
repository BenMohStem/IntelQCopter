# Historical reference index

The earlier RP2350 application is an immutable Git snapshot. Its 755 files
(117,930,376 bytes) are cached locally at `.cache/archive/rp2350/`, verified
against `rp2350.snapshot.json`. The source tree is recoverable from commit
8432cf33300fc1f92aeab97f01548aac2b1d2e48 at `archive/rp2350`.

```powershell
.\qcopter.ps1 archive status
.\qcopter.ps1 archive verify
.\qcopter.ps1 archive restore
```

Restore reconstructs a missing cache from Git, checks the entire file set and
SHA-256 values, then installs it. It does not overwrite a different existing cache.
The current Pico code remains on main. Historical scripts retain old path
assumptions; use their original commit to reproduce the historical layout.

`reference-drone/` remains versioned: it is the original inspected Drone sketch,
PlatformIO configuration and provenance. The user's separate Drone folder is untouched.
Current ESP32 and QEMU logs remain versioned in artifacts/. They are evidence,
not cache. Earlier commit history also retains the full original archive payload.
