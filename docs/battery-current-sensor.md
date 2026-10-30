# Battery current sensing - selection proposal, 30 October 2026

Use ONE sensor in the main battery feed before the power distribution branches, so it measures all ESCs AND avionics. Four ESC-current sensors add weight, wiring and ADC channels; useful later for motor diagnostics, not necessary for initial charge accounting. Do not put a single sensor after only one ESC branch and call it battery current.

```text
3S LiPo positive -> current-sensor primary conductor -> distribution -> 4 ESCs + regulators
3S LiPo negative ----------------------------------> common power return
Sensor signal output -> filtered/protected ADC input (pin not assigned yet)
```

The main high-current wiring is separate from the home-etched signal PCB. No motor phase current is being measured here.

## Range and references

Four40A ESC labels sum to160A, but this is neither measured consumption nor a guarantee that wiring/battery/motors safely deliver160A. Include avionics and transient headroom; a200A-class sensing range is a conservative starting candidate until propulsion tests establish demand. A chip measuring200A does not automatically make a cheap module or connector safe at200A continuously.

Preferred component candidate: **Allegro ACS773ECB-200B-PFF-T**, bidirectional±200A, nominal3.3V supply, ratiometric output; nominal6.6mV/A and1.65V zero-current output at3.3V. This is a component reference, not a reviewed ready-to-wire breakout. Confirm exact suffix, genuine component, PCB/conductor thermal rating, decoupling, output conditioning and actual module output range before buying/wiring. Supply/output/ADC limits need validation; do not assume every module labelled ACS773 accepts the same wiring.

Practical ready-board alternative: **Matek HCS-150A**, rated0-150A DC, advertised0-3.3V current/voltage outputs, sensitivity20mV/A, approximately7g. It senses the magnetic field around the designated battery conductor. Use it only if measured/design peak current stays below its150A range with margin; it is not a160A-range solution. Manufacturer specifies5% current-output error, so do not call its charge estimate laboratory-accurate. Keep magnetometer placement away from power-current fields and the current sensor's magnetic assembly.

ACS758200A modules are an older possible option, but Allegro marks ACS758 NOT FOR NEW DESIGN and directs new designs to ACS772/ACS773. A5V-powered module may output above the ESP32 ADC limit: inspect its datasheet/circuit before connecting. No sensor has been purchased/installed/accepted by this recommendation.

## Shortlist: all Hall-effect sensors

| Exact reference | Measurement range | Typical supply / nominal output | Intended use |
|---|---|---|---|
| ACS773ECB-200B-PFF-T | -200 to +200 A | 3.3V; 1.65V zero; 6.6mV/A | Preferred whole-battery component candidate |
| Matek HCS-150A | 0 to150A DC | 5V recommended board supply (manufacturer permits4-13V); current output0-3.3V | Ready-board option if total peak has margin below150A |
| ACS758ECB-200B-PFF-T | -200 to +200A | Chip permits3.0-5.5V; at5V:2.5V zero,10mV/A | Older fallback; module may require ADC divider; not for new design |
| ACS773LCB-050B-PFF-T | -50 to +50A | 3.3V;1.65V zero;26.4mV/A | Optional per-ESC input-current diagnostics; four needed |

All are magnetic/Hall sensing, not shunt-amplifier measurement. Integrated Hall ICs still have a resistive primary conductor, so Hall does not mean zero power loss. ACS773 specifies100micro-ohms: conductor loss is I^2 R, approximately1W at100A or4W at200A; check thermal design and continuous-current derating, not just the measurement range. The Matek device uses the battery conductor through its magnetic sensing opening; follow the manufacturer's threading/direction instructions and do not put both supply and return through the opening, which would cancel the field.

For charge estimation, choose one total-current sensor. Four per-ESC sensors omit avionics unless separately measured. Do not choose a30A ACS712 module for a40A ESC's full range or the whole battery. A lower-range sensor gives more voltage per amp but less peak headroom; actual current must be measured before optimizing range.

## Measurement and charge accounting

For the ratiometric200B candidate at nominal supply:

I[A] = (Vout[V] - Vzero[V]) / 0.0066[V/A]

Measure the actual zero offset and slope with a known current/reference meter. Use calibrated ADC voltage, check range/noise/temperature, and sample battery current in a slow task rather than blocking the1kHz IMU task. Small persistent zero bias accumulates into charge error.

Used capacity[mAh] = sum(I[A] * delta_t[s]) / 3.6

Remaining fraction approximately = initial_fraction - used_capacity / usable_capacity

Usable capacity is measured, not automatically the4200mAh label. Full battery voltage, load sag, temperature, charge history and current-sensor bias also matter. Kalman filtering cannot create missing reference information. Battery voltage alone cannot accurately identify20% charge under arbitrary load. This document does not implement a motor-disarm threshold or alter the disabled-output policy.

Sources checked30October2026:
- https://www.allegromicro.com/~/media/files/datasheets/acs773-datasheet.pdf
- https://www.allegromicro.com/en/products/sense/current-sensor-ics/integrated-current-sensors/acs758
- https://www.mateksys.com/?portfolio=hcs-150a
