# `yolo-detector` — YOLO11n cuboid detector

The learned replacement for the HSV cube detectors on `calibration+detection`. A single-class YOLO11n detector (`0: cuboid`), trained on the Mac from labelled iPhone footage, with synthetic renders tried as extra training data.

## Layout
- `training/` — the training project (originally the `Making Data` folder)
  - `train.py`, `train_mix.py`, `train_clean.py`, `train_two_stage.py` — one script per experiment
  - `run_training_local.sh`, `overnight_pipeline.sh`, `make_bundle.sh`, `extract_frames.sh` — helpers for frame extraction, overnight runs and bundling
  - `mix_data.yaml`, `synth_pretrain.yaml` — dataset configs for the mixed and synth-pretrain runs
  - `dataset/`, `dataset_clean/` — YOLO labels and `data.yaml` (original labels and the cleaned version)
  - `synth/` — synthetic data generator (Blender scene and room renders), label conversion and label-gap tools, the converted room labels and the generation logs
  - `runs/` — **every** training and validation run, kept as-is, including the ones that did worse
- `robot_eval/` — scripts and detection logs from running the models live on the robot (Jetson): `baseline/` is the first on-robot test of `cuboid_v1`, `compare/` runs all three models on the same camera feed side by side (the test that showed `v3_clean` failing)
- `models/` — the candidate models picked for the robot, with a model card

## Runs
Peak validation mAP50 from each run's `results.csv`:

| Run | Data | Epochs run | Peak mAP50 | Outcome |
|---|---|---|---|---|
| `cuboid_v1` | real only | 35 | 0.566 | Baseline. **Most reliable on the robot** (`cuboid_best_baseline.pt`); weak on dark surfaces |
| `cuboid_v2_mix` | real + ~2000 synth, trained together | 49 | 0.572 peak, 0.542 for the saved best | Did **not** beat real-only; synth swamped the real data |
| `cuboid_v3_clean` | real only, cleaned labels | 42 | 0.661 | **Failed on the robot**: highest offline score, but barely detected anything live. The val labels were model-assisted, so the score was circular |
| `cuboid_v3_synthpre` | synth rooms pretraining | 5 | 0.535 | Stage 1 of two-stage; the fine-tuned result is `cuboid_twostage_rooms.pt`: better recall on the robot but jittery. |

`runs/detect/` holds the side-by-side validation runs (`cmp_twostage`, `cmp_v3clean`, `v2_val`, `val*`).

On-robot testing (2026-07-03, `PROJECT_PLAN.md` "Model Evaluation") overrides the offline numbers: use the baseline for stability or the two-stage model for recall, not `v3_clean`. `models/README.md` was written the day before that test and still lists `v3_clean` as a candidate.

## Not included
Images, renders, videos and 3D assets (~6.5 GB) stay local; see `DATA_MANIFEST.md`. Scripts still use absolute paths to the original `Making Data` folder.
