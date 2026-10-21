"""F450 known-subassembly uncertainty model; standard library only."""
from __future__ import annotations

import csv
import json
import math
import random
import statistics
import sys
import time
from pathlib import Path


def stats(values: list[float]) -> dict[str, float]:
    ordered = sorted(values)
    def quantile(p: float) -> float:
        position = p * (len(ordered) - 1)
        low = int(position)
        high = min(low + 1, len(ordered) - 1)
        fraction = position - low
        return ordered[low] * (1.0 - fraction) + ordered[high] * fraction
    return {
        "mean": statistics.fmean(values),
        "standard_deviation": statistics.stdev(values),
        "p2_5": quantile(0.025),
        "median": quantile(0.5),
        "p97_5": quantile(0.975),
    }


def positive_gauss(rng: random.Random, mean: float, sigma: float) -> float:
    return max(0.001, rng.gauss(mean, sigma))


def sample_mass_properties(rng: random.Random) -> tuple[float, float, float, float, float, float, float]:
    # Declared assumptions: these priors are deliberately broad and replaceable.
    frame_mass = positive_gauss(rng, 0.282, 0.0282)  # clone variation: assumed 10% 1-sigma
    battery_mass = positive_gauss(rng, 0.150, 0.015)  # user's approximate mass: assumed 10% 1-sigma
    diagonal = positive_gauss(rng, 0.450, 0.005)  # tape-measure uncertainty: assumed 5 mm 1-sigma
    arm_fraction = min(0.85, max(0.25, rng.gauss(0.55, 0.10)))  # mass distribution prior
    x_batt = rng.gauss(-0.018, 0.010)  # provisional SCAD placement +/-10 mm 1-sigma
    y_batt = rng.gauss(0.0, 0.005)
    z_batt = rng.gauss(-0.008, 0.005)
    batt_x = positive_gauss(rng, 0.105, 0.0105)  # provisional box dimension, +/-10%
    batt_y = positive_gauss(rng, 0.038, 0.0038)
    batt_z = positive_gauss(rng, 0.028, 0.0028)

    total = frame_mass + battery_mass
    cg = (battery_mass*x_batt/total, battery_mass*y_batt/total,
          battery_mass*z_batt/total)
    # The frame's arm share is represented by four slender rods from center
    # to motor locations. Remaining frame mass is concentrated at center.
    arm_mass_each = frame_mass * arm_fraction / 4.0
    arm_length = diagonal / 2.0
    ixx = iyy = izz = 0.0
    for sx, sy in ((1,1),(1,-1),(-1,-1),(-1,1)):
        cx = sx * arm_length / (2.0 * math.sqrt(2.0))
        cy = sy * arm_length / (2.0 * math.sqrt(2.0))
        dx, dy, dz = cx-cg[0], cy-cg[1], -cg[2]
        ixx += arm_mass_each * (arm_length**2/24.0 + dy*dy + dz*dz)
        iyy += arm_mass_each * (arm_length**2/24.0 + dx*dx + dz*dz)
        izz += arm_mass_each * (arm_length**2/12.0 + dx*dx + dy*dy)
    plate_mass = frame_mass * (1.0-arm_fraction)
    ixx += plate_mass * (cg[1]**2 + cg[2]**2)
    iyy += plate_mass * (cg[0]**2 + cg[2]**2)
    izz += plate_mass * (cg[0]**2 + cg[1]**2)
    # Rectangular battery inertia plus parallel-axis shift to the assembly CG.
    dx, dy, dz = x_batt-cg[0], y_batt-cg[1], z_batt-cg[2]
    ixx += battery_mass*((batt_y*batt_y+batt_z*batt_z)/12.0 + dy*dy + dz*dz)
    iyy += battery_mass*((batt_x*batt_x+batt_z*batt_z)/12.0 + dx*dx + dz*dz)
    izz += battery_mass*((batt_x*batt_x+batt_y*batt_y)/12.0 + dx*dx + dy*dy)
    return total, cg[0], cg[1], cg[2], ixx, iyy, izz


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python tools/model/model_uncertainty.py <output-directory>")
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    rng = random.Random(2350)
    samples = [sample_mass_properties(rng) for _ in range(20_000)]
    fields = ("known_subassembly_mass_kg", "cg_x_m", "cg_y_m", "cg_z_m",
              "Ixx_kg_m2", "Iyy_kg_m2", "Izz_kg_m2")
    with (out / "mass-inertia-monte-carlo.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        writer.writerows(samples)
    summary = {
        "model": "F450 frame plus battery only; motors, ESCs, controller, sensors, payload excluded",
        "seed": 2350,
        "trials": len(samples),
        "priors": {
            "F450 frame mass": {"mean_kg": 0.282, "sigma_kg": 0.0282, "basis": "DJI manual value; 10% clone-variation sigma is an assumption"},
            "battery mass": {"mean_kg": 0.150, "sigma_kg": 0.015, "basis": "user approximation; 10% sigma is an assumption"},
            "motor diagonal": {"mean_m": 0.450, "sigma_m": 0.005, "basis": "F450 nominal; measurement tolerance is assumed"},
            "arm mass fraction": {"mean": 0.55, "sigma": 0.10, "basis": "placeholder prior; truncate to [0.25, 0.85]"},
            "battery center xyz": {"mean_m": [-0.018, 0.0, -0.008], "sigma_m": [0.010, 0.005, 0.005], "basis": "provisional SCAD placement and assumed tolerances"},
            "battery box xyz": {"mean_m": [0.105, 0.038, 0.028], "sigma_fraction": 0.10, "basis": "provisional SCAD dimensions, not measured"},
        },
        "assumptions": [
            "Frame arms are four uniform slender diagonal rods; remaining frame mass is a point mass at the origin.",
            "The covariance model is conditional on the stated Gaussian priors and is not a measured confidence interval.",
            "Unmodeled aircraft components make this a known-subassembly lower bound, not takeoff mass or flight inertia.",
            "No thrust, torque, stability margin, or flight time is inferred from KV/current rating without motor/prop data.",
        ],
        "properties": {name: stats([row[i] for row in samples]) for i, name in enumerate(fields)},
        "runtime_s": time.perf_counter()-started,
        "runtime_note": "Desktop Monte Carlo execution time; not RP2350 loop timing.",
    }
    (out / "mass-inertia-summary.json").write_text(json.dumps(summary, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
