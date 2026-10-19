# Mathematical report

Open `main.pdf` to read the compiled 30-page report, or open this directory
in the LaTeX editor. Main source: `main.tex`; derivations: `derivations.tex`;
data-derived numerical results: `results.tex`. The three figures in `images/`
are byte-for-byte copies of the original recorded evidence graphs.

The source is self-contained in this directory. Compile with Tectonic or a
LaTeX distribution supporting XeLaTeX and standard amsmath packages. The built-in
editor preview could not launch its protected WindowsApps compiler on this PC;
a workspace copy of the same Tectonic executable successfully compiled it.

`tools/write_modelling_results.py` regenerates the results chapter and copies
figures from actual saved simulation reports. It does not fabricate numbers.
The source describes exactly six PID objects, their degree-based units, baseline
and selected provisional gain sets, time-varying Kalman gains, expanded matrix
products/exponentials, uncertainty statistics, numerical refinement and the
unvalidated parts of the hardware/autonomy/Isaac pipeline.
