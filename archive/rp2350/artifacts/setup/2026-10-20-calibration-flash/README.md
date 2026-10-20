# Build and document evidence — 20 October 2026

## Executed

| Operation | Result | Original output |
|---|---|---|
| Pico2 RP2350 production firmware compile/link | Exit 0 | rp2350-build.log |
| Desktop production C and common C++ library build | Exit 0 | host-build.log |
| Standalone Tectonic mathematical paper compilation | Exit 0; final PDF 36 pages | latex-build.log, pdf-info.txt |
| PDF page rasterization and visual inspection | All page contact sheets, selected full pages; overflow repaired | Review images retained locally under build/pdf-review |

Firmware command: CMake v4.3.4 --build build/flight-rp2350.
Host command: powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_sitl.ps1.
Paper command: build/tools/tectonic.exe -X compile --outdir build/pdf-review --outfmt pdf --keep-logs docs/model-guide/main.tex.
TECTONIC_CACHE_DIR points to build/tectonic-cache.

The raw linker excerpt shows FLASH length 0x003FE000 and firmware end
0x1000A600. The two 4 KiB calibration sectors begin at XIP 0x103FE000,
so this image does not overlap them. See reserved-flash-region.ld.
The size utility reports text 46592, data 0, BSS 5740 bytes.
The manifest records actual working-file and binary hashes; Git may normalize text newlines.
Generated binaries remain local build artifacts, not versioned downloads.

## Failed attempts retained

- latex-failed-alignment.log: a boxed expression spanned an alignment separator; corrected in source.
- latex-failed-fonts.log: standard font data were not yet available in the local cache.
- Standard LaTeX font/packages were then fetched; final compilation succeeded.
- The final standalone compiler still prints a Fontconfig configuration diagnostic,
  but returns success and embeds the fonts; rendered mathematics was inspected.
- The app's separate live-preview compiler could not start: Windows CreateProcessAsUserW
  error 5, access denied. The delivered PDF was compiled by standalone Tectonic.

## Not executed or established

No implementation test suite, new ROS/Gazebo flight, physical sensor capture,
flash power-loss/endurance check, ESC operation or actual RP2350 write.
Existing battery test expectations were updated to the new enum/policy but were not run.
No old video/log is presented as evidence for the new flash or battery behavior.
The manually supplied simulation SOC topic has not been exercised in a new flight.
Physical arming remains disabled. Missing drivers and measured parameters are documented.
