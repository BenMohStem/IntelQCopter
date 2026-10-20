# Synthetic panel detector

YOLO11n fine-tuned for 12 epochs, 180 training / 40 validation images, 320 pixels,
seed 9201. Synthetic validation: precision 0.934, recall 0.976, mAP50 0.980,
mAP50-95 0.853. These are not real-world accuracy estimates. The validation views
share the procedural scene style. No real panel or hard-negative evaluation yet.

best.pt SHA256: f5295557192e89d805b2d9c2b7cc07359604ab864672ae66fcc54ffa19f74b4e

Weights are local under build/panel-training/rgb-panel/weights (ignored in Git).
Recreate with sim/garden/train_panels.py. Original training/export logs are in
../../test-runs/2026-10-20-missions. The first plotting attempt reported a missing
Polars runtime; version 2.0 then failed a CPU feature check. Pinning the package
and runtime to 1.35.1 repaired imports/CSV plotting. Original failures and repair
logs are retained. Training results are retained
unchanged in panel-training-results.csv. The model only recognizes this synthetic
fixture style. Range also assumes its known metric size; see sim/garden/README.md.
