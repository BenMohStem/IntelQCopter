# Original KiCad carrier evidence — 21 October 2026

Engine: KiCad 10.0.6 native CLI ERC, DRC with schematic parity, XML netlist export; independent Python comparison against generated pad-net assignments.

Final: `erc-bundled.json`, `drc-release-candidate.json`, `schematic-netlist-final.xml`, `verification-summary.json`. The summary pins source hashes. Zero ERC/DRC/unrouted/parity issues; all 35 connected nets agree for 37 components, including 13 insulated wire links.

Earlier `erc-01`, `erc-final`, `drc-01`, `drc-02`, `drc-parity`, `drc-bundled` and the pre-normalization comparison are preserved failures/intermediate trials. Do not read them as final success reports. The JSON reports contain the actual native diagnostics; the logs contain their original CLI summaries.

Assumptions: reference DevKit header geometry, mapped 3.3 V-compatible sensor breakouts, external regulated 5 V, optional companion UART, no ESC power paths. Mechanical fit, supply/current-sensor choice, ADC/HMC/lidar drivers and physical interfaces are unvalidated. No physical flight or electrical bench measurements were performed for this step. Physical motor arming remains disabled.
