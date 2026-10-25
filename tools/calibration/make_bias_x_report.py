"""Generate an offline, self-contained report tracing measured X bias to the MPU trim prediction."""
from pathlib import Path
import hashlib
import html
import io
import json
import math
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
FIT = ROOT / "measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json"


def equation_svg(tex):
    fig = plt.figure(figsize=(10, 0.65))
    fig.text(0.02, 0.45, "$" + tex + "$", fontsize=16, va="center")
    output = io.StringIO()
    fig.savefig(output, format="svg", bbox_inches="tight", pad_inches=0.12, transparent=True)
    plt.close(fig)
    svg = output.getvalue()
    return svg[svg.index("<svg"):]


def build():
    fit_bytes = FIT.read_bytes()
    fit = json.loads(fit_bytes)
    records = {row["pose"]: row for row in fit["records"]}
    rows = []
    pairs = {}
    for axis in "XYZ":
        values = []
        for suffix in ("plus", "minus"):
            measurements = [records[f"repeat-{repeat}_{axis}{suffix}"]["sensor_accel_mean_m_s2"][0] for repeat in (1, 2)]
            assert all(records[f"repeat-{repeat}_{axis}{suffix}"]["samples"] == 20000 for repeat in (1, 2))
            mean = sum(measurements) / 2
            values.append(mean)
            rows.append([axis + ("+" if suffix == "plus" else "−"), *[f"{value:.12f}" for value in measurements], f"{mean:.12f}"])
        pairs[axis] = sum(values) / 2
    reconstructed = sum(pairs.values()) / 3
    bias = fit["bias_sensor_m_s2"][0]
    assert abs(reconstructed - bias) < 1e-12
    g = 9.80665
    step = g / 1024
    units = bias / step
    steps = math.floor(units + 0.5)
    decrement = 2 * steps
    predicted = decrement * g / 2048
    baseline = -2074
    target = baseline - decrement
    residual = bias - predicted
    source = (ROOT / "firmware/main.cpp").read_text(encoding="utf-8")
    literal = float(re.search(r"accel_bias_x\s*=\s*([0-9.]+)f", source)[1])
    assert abs(literal - bias) < 1e-8
    assert int(re.search(r"expected_baseline_x\s*=\s*(-?\d+)", source)[1]) == baseline
    assert decrement == 274 and target == -2348 and (baseline & 1) == (target & 1)
    proof = {"engine": "Python arithmetic using published means from real MPU6050 recordings; SVG equations rendered by Matplotlib",
        "fit_sha256": hashlib.sha256(fit_bytes).hexdigest(), "fit_training_repetitions": [1, 2],
        "training_samples": 240000, "training_pose_means": 12,
        "pair_midpoints_sensor_x_m_s2": pairs, "reconstructed_bias_m_s2": reconstructed,
        "stored_fitted_bias_m_s2": bias, "firmware_decimal_literal": literal,
        "assumed_g_m_s2": g, "nominal_offset_word_units_per_g": 2048,
        "allowed_step_m_s2": step, "desired_steps": units, "rounded_steps": steps,
        "offset_decrement": decrement, "baseline": baseline, "target": target,
        "predicted_acceleration_decrease_m_s2": predicted,
        "rounding_residual_m_s2": residual, "rounding_error_bound_m_s2": step / 2,
        "predicted_adc_decrease_at_8g": decrement * 4096 / 2048,
        "firmware_sha256": hashlib.sha256((ROOT / "firmware/main.cpp").read_bytes()).hexdigest(),
        "firmware_changed": False, "uploaded": False, "new_hardware_measurements": False,
        "warning": "Register response is a library-based nominal model, not a measured hardware result. Pose-reference uncertainty is not quantified by displayed decimal precision."}
    blocks = []
    def add(kind, *value):
        blocks.append((kind, value))
    add("p", "This report explains why the expected X-reading decrease is approximately 1.312 m/s². It traces that number from your original six-pose measurements to a fitted bias, then to the integer MPU6050 offset word. The bias estimate and the predicted register effect are different quantities.")
    add("note", "STATUS: calculated prediction. The register's physical response and reset/power-cycle behavior still need bench verification. This report performs no new sensor measurement, upload or firmware change.")
    add("h2", "1. Which measurements produced the X bias?")
    add("p", "The completed secured-mount session contains 18 records × 20,000 samples = 360,000 samples. Only repetitions 1 and 2 were used to fit the candidate: 12 pose means from 240,000 samples. Repetition 3 and the later four tilted poses were kept out of this fit. The four tilted records were reviewed using your clarified raised-side order and approximate ±1° hand-held reference; they did not change the bias.")
    add("p", "Each row below shows the mean SENSOR-X reading while the DRONE axis named in the first column pointed upward. All numbers are m/s². These are not three different axes of one sample: every numeric entry is the X channel measured in a different physical pose.")
    add("table", ["Drone pose", "Repetition 1: sensor X", "Repetition 2: sensor X", "Mean of the two"], rows)
    add("eq", r"\bar r_{X+}=\frac{10.594872901279+10.608765615543}{2}\approx10.601819258411")
    add("h2", "2. Why do opposite poses reveal the bias?")
    add("p", "A mounted sensor can mix all three drone axes. For its X output, use the forward model below. The A coefficients describe mounting and sensor response; e denotes the remaining measurement/model error. We assume the labelled reference directions are accurate enough for a provisional fit.")
    add("eq", r"r_x=b_x+A_{11}a_x+A_{12}a_y+A_{13}a_z+e_x")
    add("p", "In the +X and −X poses, the target body vectors are (+g,0,0) and (−g,0,0). Ignoring mean error for this derivation, the measured X values satisfy:")
    add("eq", r"\bar r_{X+}=b_x+gA_{11},\qquad \bar r_{X-}=b_x-gA_{11}")
    add("p", "Adding the pair cancels the gravity response, even when the sensor is tilted relative to the drone:")
    add("eq", r"c_X=\frac{\bar r_{X+}+\bar r_{X-}}{2}\approx b_x")
    add("eq", r"c_X=\frac{10.601819258411-8.093833822761}{2}\approx1.253992717825")
    add("p", "Repeat this operation using the Y and Z opposite-pose pairs. In ideal data the three answers agree. Your measurements give:")
    add("table", ["Opposite-pose pair", "Sensor-X midpoint (m/s²)"], [[axis, f"{pairs[axis]:.12f}"] for axis in "XYZ"])
    add("p", "With equal weights for these balanced poses, average the three midpoint estimates:")
    add("eq", r"b_x=\frac{c_X+c_Y+c_Z}{3}")
    add("eq", r"b_x\approx\frac{1.253992717825+1.422572526859+1.249484974794}{3}")
    add("eq", r"b_x\approx1.308683406493\ \mathrm{m/s^2}")
    add("p", "This reproduces the stored forward least-squares fit. To see why, minimize the sum of squared residuals r_i − b − A a_i over the 12 training means. Differentiating with respect to b gives the following normal equation:")
    add("eq", r"0=-2\sum_{i=1}^{12}(r_{x,i}-b_x-A_xa_i)")
    add("p", "The symmetric target vectors sum to zero. Therefore the fitted intercept equals the mean of the 12 measured X means:")
    add("eq", r"\sum_i a_i=0\quad\Longrightarrow\quad b_x=\frac{1}{12}\sum_{i=1}^{12}r_{x,i}")
    add("note", "The midpoint estimates differ by about 0.173 m/s². That disagreement can reflect pose-reference error, repositioning, temperature or model limitations. Many decimal places describe reproducible arithmetic, not equally precise knowledge of the true bias. No Gaussian or independent-sample assumption is needed for this algebra.")
    add("h2", "3. Convert the bias to an MPU offset word")
    add("p", "The established MPU6050 library convention uses a nominal accelerometer offset-word scale of 2048 units/g. This is separate from the measurement ADC scale, which is 4096 counts/g at our ±8g setting. The offset conversion and preservation of compensation bit 0 are implemented in established MPU6050 drivers; the public register map does not fully specify these trim semantics. Applying this convention to our particular module is a prediction that the bench test must verify.")
    add("eq", r"1\ \mathrm{offset\ word\ unit}\ \longleftrightarrow\ \frac{g}{2048}\ \mathrm{m/s^2}")
    add("p", "We preserve compensation bit 0 by changing the whole word by an even integer. Hence the correction grid is two word units per step:")
    add("eq", r"q=\frac{2g}{2048}=\frac{g}{1024}=0.009576806640625\ \mathrm{m/s^2}")
    add("p", "Calculate how many of those steps would remove the fitted bias:")
    add("eq", r"n_{ideal}=\frac{b_x}{q}=\frac{1024b_x}{g}\approx136.651334375")
    add("eq", r"n=\mathrm{round}(136.651334375)=137,\qquad d=2n=274")
    add("p", "The neighbouring allowed corrections show why 137 steps were selected:")
    add("table", ["Steps", "Word decrement", "Predicted decrease (m/s²)", "Remaining bias (m/s²)"],
        [[str(n), str(2*n), f"{n*step:.12f}", f"{bias-n*step:+.12f}"] for n in (136, 137)])
    add("h2", "4. Why write −2348 rather than −274?")
    add("p", "Your original recording headers reported an X offset word of −2074 after initialization. That existing baseline is retained and adjusted; it is not replaced by the decrement alone. It is a register value, not an acceleration reading.")
    add("eq", r"o_{new}=o_{baseline}-d=-2074-274=-2348")
    add("p", "The positive fitted bias calls for a negative offset-word change under the nominal convention. The firmware verifies the known baseline, writes the absolute target and reads it back. Repeating a command sets the same target; it does not subtract another 274. An unexpected baseline stops the lesson instead of accumulating a correction.")
    add("code", "static const float accel_bias_x = 1.308683406f;\nstatic const int16_t expected_baseline_x = -2074;\nstatic const int32_t offset_decrement = 2 * lroundf(accel_bias_x * 1024.0f / 9.80665f);\nstatic const int32_t corrected_offset_x = expected_baseline_x - offset_decrement;")
    add("h2", "5. The origin of 1.312022510 m/s²")
    add("p", "Convert the selected word decrement back into its nominal physical effect:")
    add("eq", r"\Delta a_x=d\frac{g}{2048}=274\frac{9.80665}{2048}")
    add("eq", r"\Delta a_x=1.312022509765625\ \mathrm{m/s^2}\approx1.312\ \mathrm{m/s^2}")
    add("p", "The fitted bias is 1.308683406 m/s²; the allowed register grid makes the selected predicted correction slightly larger. The modeled residual is:")
    add("eq", r"b_{after}=b_x-\Delta a_x\approx-0.003339103273\ \mathrm{m/s^2}")
    add("p", "Nearest-step rounding alone gives a maximum magnitude q/2, assuming the nominal conversion. This is only a quantization bound; it does not bound the true calibration, reference or sensor error.")
    add("eq", r"|e_{round}|\leq\frac{q}{2}=0.0047884033203125\ \mathrm{m/s^2}")
    add("p", "At ±8g, the measurement ADC has 4096 counts/g. The same predicted decrease therefore corresponds to:")
    add("eq", r"\Delta\mathrm{ADC}=\frac{\Delta a_x}{g}\,4096=\frac{274}{2048}\,4096=548\ \mathrm{counts}")
    add("h2", "6. What must the real bench test demonstrate?")
    add("p", "Keep the drone fixed in one orientation. The baseline and corrected X readings both contain the same real gravity component. Subtracting their means should cancel that common component, leaving the register's effect:")
    add("eq", r"\bar r_{x,baseline}-\bar r_{x,corrected}\approx1.312022510\ \mathrm{m/s^2}")
    add("p", "The final X reading need not be zero. For example, with a hypothetical constant real X gravity component of 2.8 m/s², the model predicts baseline X = 2.8 + 1.308683406 = 4.108683406, and corrected X = 4.108683406 − 1.312022510 = 2.796660896 m/s². These example readings are invented to illustrate the arithmetic; they are not recorded results.")
    add("p", "The prepared PC test measures baseline, corrected and baseline again, interpolating between the two baseline means to reduce linear drift. Readback verifies the bits; the observed change checks actual sensor response. Neither test alone proves complete three-axis calibration or flight readiness. Reset and full power-cycle checks remain separate.")
    add("table", ["Quantity", "Origin", "Evidence status"], [
        ["1.308683406 m/s²", "Fit of real training-pose means", "Provisional estimate; reference uncertainty remains"],
        ["−2074", "Recorded initial offset word", "Seen in original raw-session metadata"],
        ["274 and −2348", "Rounded decrement and absolute target", "Computed and compiled"],
        ["1.312022510 m/s² / 548 ADC counts", "Nominal register-response model", "Prediction; not a measured response"],
        ["Actual decrease and reboot behavior", "Physical bench comparison", "Not established by this report"]])
    add("h2", "Sources and reproduction")
    add("p", "Data: the secured-mount analysis JSON and the recorded post-reset baseline. Method: balanced forward affine least squares. Assumed standard gravity is 9.80665 m/s²; no local-gravity measurement is claimed. Processing: Python arithmetic and Matplotlib SVG equation rendering; no simulator or new sensor reading. Original raw logs remain local. Numerical output is saved alongside this report for independent checking.")
    return blocks, proof


def main():
    blocks, proof = build()
    title = "Where does the 1.312 m/s² correction come from?"
    markdown = ["# " + title, "", "25 October 2026 · IntelQCopter · X-bias provenance report", ""]
    body = []
    for kind, content in blocks:
        if kind == "eq":
            markdown += ["$$", content[0], "$$", ""]
            body.append('<div class="equation">' + equation_svg(content[0]) + '</div>')
        elif kind == "table":
            headers, rows = content
            markdown += ["| " + " | ".join(headers) + " |", "| " + " | ".join("---" for _ in headers) + " |"]
            markdown += ["| " + " | ".join(row) + " |" for row in rows]
            markdown += [""]
            body.append('<div class="table-wrap"><table><thead><tr>' + ''.join('<th>'+html.escape(h)+'</th>' for h in headers) + '</tr></thead><tbody>' + ''.join('<tr>'+''.join('<td>'+html.escape(cell)+'</td>' for cell in row)+'</tr>' for row in rows) + '</tbody></table></div>')
        elif kind == "h2":
            markdown += ["## " + content[0], ""]
            body.append('<h2>' + html.escape(content[0]) + '</h2>')
        elif kind == "code":
            markdown += ["```cpp", content[0], "```", ""]
            body.append('<pre><code>' + html.escape(content[0]) + '</code></pre>')
        else:
            markdown += [("**" + content[0] + "**") if kind == "note" else content[0], ""]
            body.append('<p class="'+('note' if kind=='note' else 'text')+'">'+html.escape(content[0])+'</p>')
    links = [
        ("Measured-data fit JSON", "../measurements/mpu6050/2026-10-25-secured-mount-analysis-v2/analysis.json"),
        ("Tilt validation review", "../measurements/mpu6050/2026-10-25-tilt-validation/report.md"),
        ("Numerical provenance", "../measurements/mpu6050/2026-10-25-bias-x-provenance/calculation.json"),
        ("Bench procedure", "bench-x-bias.md"), ("Current firmware", "../firmware/main.cpp"),
        ("TDK register map: ADC sensitivities", "https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf"),
        ("MPU6050 library: word trims and bit-zero preservation", "https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp"),
        ("MPU6050 example: nominal 2048 offset units/g", "https://github.com/kriswiner/MPU6050HMC5883AHRS/blob/master/MPU6050HMC5883AHRS.ino")]
    markdown += ["- ["+name+"]("+url+")" for name,url in links]
    body.append('<ul>'+''.join('<li><a href="'+html.escape(url)+'">'+html.escape(name)+'</a></li>' for name,url in links)+'</ul>')
    style = """body{margin:0;background:#eef2f6;color:#183149;font:17px/1.7 'Segoe UI',Arial,sans-serif}main{max-width:1060px;margin:36px auto;background:white;padding:48px 56px;border-radius:16px;box-shadow:0 8px 35px #17334912}h1{font-size:36px;line-height:1.2;margin:12px 0 22px}h2{margin-top:44px;color:#174b65;border-top:1px solid #dce6ed;padding-top:24px;font-size:24px}.eyebrow{font-size:13px;letter-spacing:.1em;text-transform:uppercase;color:#587385}.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin:24px 0}.card{padding:18px;background:#f1f7fa;border:1px solid #dbe8ef;border-radius:10px}.number{display:block;font-size:25px;font-weight:650;color:#125c76}.label{font-size:14px}.note{background:#fff7e7;border-left:4px solid #c98c35;padding:16px 20px}.equation{background:#f7fafc;border-radius:8px;margin:16px 0;padding:10px 16px;overflow-x:auto}.equation svg{display:block;max-width:100%;height:auto}table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;font-size:14px}th{background:#e9f2f6;text-align:left}td,th{padding:10px;border-bottom:1px solid #dbe5ec;white-space:nowrap}.table-wrap{overflow-x:auto}pre{overflow-x:auto;background:#142b3c;color:#e9f5ff;padding:20px;border-radius:8px;font-size:14px}a{color:#006f93}footer{font-size:13px;color:#587385;margin-top:36px}@media(max-width:760px){main{margin:0;padding:24px 18px;border-radius:0}.cards{grid-template-columns:1fr}h1{font-size:29px}}@media print{body{background:white}main{box-shadow:none;margin:0;padding:0;max-width:none}.equation,table,.card{break-inside:avoid}h2{break-after:avoid}a{color:inherit}.number{font-size:21px}}"""
    cards = '<div class="cards"><div class="card"><span class="number">1.308683406</span><span class="label">m/s² · fitted X bias</span></div><div class="card"><span class="number">274</span><span class="label">offset-word decrement</span></div><div class="card"><span class="number">1.312022510</span><span class="label">m/s² · predicted reading decrease</span></div></div>'
    document = '<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+html.escape(title)+'</title><style>'+style+'</style></head><body><main><div class="eyebrow">IntelQCopter · 25 October 2026 · Calculation trace</div><h1>'+html.escape(title)+'</h1>'+cards+''.join(body)+'<footer>All equations are embedded SVG. This report needs no internet connection or external math-rendering library.</footer></main></body></html>'
    output = ROOT / "measurements/mpu6050/2026-10-25-bias-x-provenance"
    output.mkdir(parents=True, exist_ok=False)
    (output / "calculation.json").write_text(json.dumps(proof, indent=2) + "\n", encoding="utf-8")
    (ROOT / "docs/bias-x-provenance.md").write_text("\n".join(markdown) + "\n", encoding="utf-8")
    (ROOT / "docs/bias-x-provenance.html").write_text(document, encoding="utf-8")
    print("Report: docs/bias-x-provenance.html (offline SVG math); source: docs/bias-x-provenance.md")


if __name__ == "__main__":
    main()
