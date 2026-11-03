# v4 package import manifest

Source: user-provided `C:\Users\Mohamed\Downloads\IntelQCopter_100Hz_UART_Mag_Recovery_v4`.
The corresponding ZIP SHA-256 is
`3F1BE14B469130D0A72A18F7A204BB19F85C218F2EA6C965F1A0347F0502736A`.
Cached `.pio` builds and machine-specific `.vscode` settings were not imported.
The GUI, analysis/calibration tools, firmware sources and capture evidence were
copied; the main firmware and launcher have the documented local changes.

SHA-256 values below identify the checked-in files and retained evidence. The
original-source hashes distinguish unchanged inputs from the two edited files.

| File | SHA-256 | Provenance |
|---|---|---|
| `src/main.cpp` | `96A695A7AF9CDD6FC64BBAC0F7004D63A91696E7FC7154C172DB8D3972758EA4` | Local STOP+START QMC read change; original source was `FF0E833233666498DE07D9F8B23CE69773FED3704D6DB837E951BADC62EE5171` |
| `src/mag_probe.cpp` | `7698FA20891C1CF5C6C48EE705CCADDAE3B52216B6F69BA534246DD840FEED82` | Unchanged isolated v13 MAG-only probe |
| `tools/live_calibration_gui.py` | `6A35AB19F7146B02B023855B8AECD5864BB6C631A797AB22B7EEE25FED666130` | Unchanged analyzed guided GUI |
| `tools/start_calibration.ps1` | `774C7E6A26F50426472181A68F13A1079A2642250A16050106B4DCAF685E7D84` | Python interpreter fallback added |
| `captures/usb_preflight_20261103_213127.csv` | `845EB0B1FBD5060BA73BDB7CB2529F04A8432C729DC09AA659A6A92E29C3F452` | Original failed preflight, retained unchanged |
| `captures/usb_preflight_20261103_213351.csv` | `44E0D14C704EF5034D0A92034639BFE3EFEACF83CFFC4D107BCEA70658BD5318` | Original empty failed attempt, retained unchanged |
| `captures/usb_preflight_20261103_214043.csv` | `F75BB0CA29B28DEEE9175F0CDB3E87AC7CC812A59764BE27FCB9E09837732863` | New preflight after local firmware upload; zero MAG samples |
| `captures/mag_probe_v13_100hz_20261103_213504.log` | `26F204E4EEDBAF9D8A3B127134B7E1B545F450C6EC4C53E4B417DE8618FDCEBE` | Earlier isolated probe: 1,683 fresh samples at 89.64 Hz |
| `captures/mag_probe_v13_100hz_20261103_213504.json` | `68BB22D7EDEF1D591D044968B505F5607120D1BA18E968E8DEE17DA8C35FB0BE` | Summary of the earlier isolated probe |
| `captures/mag_probe_v13_100hz_20261103_214259.log` | `2A0F359005436FC180BE2F3228ADBA95167083966634D439CC8C4FD2ED0AF874` | Invalid attempt; board stayed in ROM download mode |
| `captures/mag_probe_v13_100hz_20261103_214259.json` | `BAA3AFB1250E95E73CFAE63356C7D34BB79C86A7057081C63D3A0354FC915959` | Confirms no probe CSV samples; not a sensor result |
