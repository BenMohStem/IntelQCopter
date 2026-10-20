# Mathematical report — 47 pages

Read main.pdf. The report now derives the actual force/torque differential
equations, hover linearization, actuator lag, second-order approximations, full
third/fourth-order feedback models and sixth-order horizontal guidance model.
It derives PID coefficients, Routh/Hurwitz conditions, sampled/multirate models,
Kalman prediction/correction/covariance and parameter uncertainty step by step.
The generic matrix multiplication/exponential lessons have been removed.

main.tex includes derivations.tex (and flight_dynamics_tutorial.tex), historical
results.tex, then mission_results.tex for the new experiments. Earlier numerical
results remain explicitly historical. The current conservative provisional gain
scale is 0.7. All airframe parameters are assumed pending measurement.

Regenerate numerical chapters with tools/write_modelling_results.py and
tools/write_mission_report.py. They read saved evidence, not fabricated values.
Compile with Tectonic/XeLaTeX. The compiled report was checked for layout and
compiler warnings. Five saved evidence figures are included in images/.
The complete editable source bundle is:
../../artifacts/simulation/2026-10-20-missions/mathematical-report-source.zip
