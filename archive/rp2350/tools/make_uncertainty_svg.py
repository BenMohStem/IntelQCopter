"""Render small, dependency-free SVG proof plots from recorded Monte Carlo CSVs."""
from __future__ import annotations

import csv
import html
import math
import sys
from pathlib import Path


if len(sys.argv) != 4:
    raise SystemExit("Usage: python make_uncertainty_svg.py <kalman.csv> <mass.csv> <output-dir>")

kalman_path, mass_path, output_dir = map(Path, sys.argv[1:])
with kalman_path.open(newline="", encoding="utf-8-sig") as f:
    kf = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
with mass_path.open(newline="", encoding="utf-8-sig") as f:
    mass_rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
if not kf or not mass_rows:
    raise SystemExit("Input Monte Carlo CSV is empty")

W, H = 1100, 620
INK, MUTED, GRID = "#172033", "#52637a", "#d7e0ea"
BLUE, ORANGE, GREEN, PURPLE = "#2563eb", "#ea580c", "#059669", "#7c3aed"


def svg_start(title: str, subtitle: str) -> list[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" role="img">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="48" y="48" font-family="Segoe UI,Arial,sans-serif" font-size="27" font-weight="700" fill="{INK}">{html.escape(title)}</text>',
        f'<text x="48" y="78" font-family="Segoe UI,Arial,sans-serif" font-size="15" fill="{MUTED}">{html.escape(subtitle)}</text>',
    ]


def text(x: float, y: float, value: str, size: int = 14, color: str = INK,
         anchor: str = "middle") -> str:
    return (f'<text x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}" '
            f'font-family="Segoe UI,Arial,sans-serif" font-size="{size}" fill="{color}">'
            f'{html.escape(value)}</text>')


def line_path(points: list[tuple[float, float]], color: str, width: int = 3) -> str:
    d = " ".join(("M" if i == 0 else "L") + f"{x:.2f},{y:.2f}" for i, (x, y) in enumerate(points))
    return f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round"/>'


def save(name: str, parts: list[str]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / name
    path.write_text("\n".join(parts + ["</svg>"]), encoding="utf-8", newline="\n")
    print(f"Wrote {path} ({path.stat().st_size} bytes)")


def kalman_plot() -> None:
    p = svg_start("Kalman uncertainty: empirical error vs predicted sigma",
                  f"Production C roll filter · {int(500)} seeded synthetic trials · 2 ms steps · not a flight simulation")
    left, right, top, bottom = 105, 1040, 130, 490
    ymax = max(max(r["empirical_error_std_deg"], r["kalman_std_deg"]) for r in kf) * 1.12
    for tick in range(6):
        val = ymax * tick / 5
        y = bottom - (bottom-top) * val / ymax
        p.append(f'<path d="M{left},{y:.2f} H{right}" stroke="{GRID}"/>')
        p.append(text(left-14, y+5, f"{val:.2f}", 13, MUTED, "end"))
    for tick in range(6):
        val = float(kf[-1]["time_s"]) * tick / 5
        x = left + (right-left) * tick / 5
        p.append(f'<path d="M{x:.2f},{top} V{bottom}" stroke="{GRID}"/>')
        p.append(text(x, bottom+24, f"{val:.1f}", 13, MUTED))
    def coords(key: str) -> list[tuple[float, float]]:
        return [(left+(right-left)*r["time_s"]/kf[-1]["time_s"],
                 bottom-(bottom-top)*r[key]/ymax) for r in kf]
    p.append(line_path(coords("empirical_error_std_deg"), BLUE))
    p.append(line_path(coords("kalman_std_deg"), ORANGE))
    p.append(text((left+right)/2, bottom+53, "Time (s)", 15))
    p.append(f'<text x="28" y="{(top+bottom)/2}" transform="rotate(-90 28 {(top+bottom)/2})" text-anchor="middle" font-family="Segoe UI,Arial" font-size="15" fill="{INK}">Roll standard deviation (degrees)</text>')
    ly = 552
    for i, (label, color) in enumerate((("Empirical estimation error σ", BLUE), ("Filter predicted √P00", ORANGE))):
        x = 280 + i*340
        p.append(f'<path d="M{x},{ly-5} h36" stroke="{color}" stroke-width="4"/>')
        p.append(text(x+46, ly, label, 15, INK, "start"))
    end = kf[-1]
    footer = (f"At 5 s: empirical σ {end['empirical_error_std_deg']:.4f}° · predicted σ {end['kalman_std_deg']:.4f}° · "
              f"RMSE {end['rmse_deg']:.4f}° · ±1σ coverage {end['coverage_1sigma']*100:.1f}% · ±2σ {end['coverage_2sigma']*100:.1f}%")
    p.append(text(W/2, 600, footer, 14, MUTED))
    save("kalman-uncertainty.svg", p)


def histogram_plot(values: list[float], bins: int = 48) -> list[tuple[float, float]]:
    lo, hi = min(values), max(values)
    width = (hi-lo)/bins
    counts = [0]*bins
    for v in values:
        counts[min(bins-1, int((v-lo)/width))] += 1
    return [(lo+(i+0.5)*width, count/(len(values)*width)) for i, count in enumerate(counts)]


def density_chart(filename: str, title: str, subtitle: str,
                  series: list[tuple[str, str, list[tuple[float, float]]]], xlabel: str,
                  value_format) -> None:
    p = svg_start(title, subtitle)
    left, right, top, bottom = 110, 1040, 135, 485
    all_x = [x for _, _, points in series for x, _ in points]
    all_y = [y for _, _, points in series for _, y in points]
    xmin, xmax = min(all_x), max(all_x)
    xpad = (xmax-xmin)*0.04
    ymin, ymax = 0.0, max(all_y)*1.1
    for tick in range(6):
        xv = xmin-xpad+(xmax-xmin+2*xpad)*tick/5
        x = left+(right-left)*tick/5
        p.append(f'<path d="M{x:.2f},{top} V{bottom}" stroke="{GRID}"/>')
        p.append(text(x, bottom+24, value_format(xv), 13, MUTED))
        yv = ymax*tick/5
        y = bottom-(bottom-top)*tick/5
        p.append(f'<path d="M{left},{y:.2f} H{right}" stroke="{GRID}"/>')
        p.append(text(left-12, y+5, f"{yv:.2g}", 13, MUTED, "end"))
    for name, color, points in series:
        mapped = [(left+(right-left)*(x-(xmin-xpad))/(xmax-xmin+2*xpad),
                   bottom-(bottom-top)*y/ymax) for x, y in points]
        p.append(line_path(mapped, color, 2))
    p.append(text((left+right)/2, bottom+53, xlabel, 15))
    p.append(f'<text x="28" y="{(top+bottom)/2}" transform="rotate(-90 28 {(top+bottom)/2})" text-anchor="middle" font-family="Segoe UI,Arial" font-size="15" fill="{INK}">Probability density</text>')
    ly = 550
    spacing = 250
    start = (W-(len(series)-1)*spacing)/2
    for i, (name, color, _) in enumerate(series):
        x = start+i*spacing
        p.append(f'<path d="M{x},{ly-5} h34" stroke="{color}" stroke-width="4"/>')
        p.append(text(x+44, ly, name, 15, INK, "start"))
    p.append(text(W/2, 600, f"Seed 2350 · {len(mass_rows):,} Monte Carlo draws · Gaussian/prior assumptions; not measured component distributions", 14, MUTED))
    save(filename, p)


kalman_plot()
mass_values = [r["known_subassembly_mass_kg"] for r in mass_rows]
mass_hist = histogram_plot(mass_values)
density_chart("mass-density.svg", "Known-subassembly mass distribution",
              "F450 reference frame + approximate 150 g battery only; motors/electronics/payload excluded",
              [("Frame + battery", BLUE, mass_hist)], "Combined mass (kg)", lambda x: f"{x:.2f}")
inertias = (("Ixx roll", "#7c3aed", "Ixx_kg_m2"),
            ("Iyy pitch", "#db2777", "Iyy_kg_m2"),
            ("Izz yaw", "#0891b2", "Izz_kg_m2"))
density_chart("inertia-density.svg", "Provisional mass moment of inertia distributions",
              "Simplified frame-rod + battery-box model; incomplete airframe mass properties",
              [(name, color, histogram_plot([r[key] for r in mass_rows])) for name, color, key in inertias],
              "Moment of inertia (kg·m²)", lambda x: f"{x*1e3:.2f}e-3")
