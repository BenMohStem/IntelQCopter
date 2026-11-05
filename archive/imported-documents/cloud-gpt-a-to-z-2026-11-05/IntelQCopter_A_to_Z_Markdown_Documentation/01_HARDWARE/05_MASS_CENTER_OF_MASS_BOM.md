# Mass budget, BOM, COM and uncertainty

## Described physical inventory

| Item | Quantity | Notes |
|---|---:|---|
| F450-like frame, hardware and spacers | 1 | Two red/two white arms |
| 3S 4200 mAh LiPo | 1 | Lower-center area |
| A2212/10T motors | 4 | 1400 KV described; model not independently confirmed |
| 40 A ESCs | 4 | Mid-arm/below arm |
| 75×100 mm electronics PCB | 1 | ESP32-S3, MPU6050, QMC5883P, BMP280, regulators, connectors |
| TOFSense-F2 Mini | 1 | Downward near center |
| Samsung N150 companion motherboard | optional | Screen/battery removed in proposed variant |

## Current *assumption-based* profile estimates

| Property | Basic | N150 variant |
|---|---:|---:|
| Total mass | 1.104 ± 0.051 kg | 1.319 ± 0.083 kg |
| COM z relative to source origin | +0.01866 m | +0.01185 m |
| Jxx | 0.00978 kg·m² | 0.01030 kg·m² |
| Jyy | 0.01039 kg·m² | 0.01091 kg·m² |
| Jzz | 0.01917 kg·m² | 0.01956 kg·m² |

## Error propagation

The standard deviations came from component and geometry priors, *not* repeated independent aircraft measurements. Independence between masses and position errors may be unjustified if hardware is mechanically linked. Mass uncertainty matters to collective feedforward; inertia uncertainty propagates into rate-loop torque gains; COM position affects arm leverage and static balance. Off-diagonal inertia terms should be estimated or bounded if asymmetric components are installed.

## Measurements needed

Weigh the final frame *including cable ties, bolts, wiring, battery and actual props*. Weigh or estimate individual subassemblies; photograph their measured body coordinates and vertical heights; repeat the weighing. Do the same after adding the N150 motherboard. Update the model and only then claim physical posterior uncertainty.
