# Preserved first attempts

The initial corrected-board native DRC is retained as `drc-01.json`: zero unconnected/parity errors, with four annotation warnings (three small text heights and one overlap). Header annotations were moved to Dwgs.User at0.8mm, then the board was regenerated and final native checks repeated.

The first PDF command used PlatformIO Python and failed before authoring with `ModuleNotFoundError: No module named reportlab`. Its first audit command failed with `ModuleNotFoundError: No module named scipy`. No success result was inferred from those attempts. The final PDF used the bundled Python/reportlab runtime. SciPy1.18.1 was installed without dependencies into the ignored task-local `QCopter/build/pcb-audit-python` folder for the final rendered-mask audits. No dependency or personal screenshot was added to the Git package.
