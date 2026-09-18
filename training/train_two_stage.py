#!/usr/bin/env python3
"""Two-stage training: synth-pretrain -> real-only fine-tune.

Rationale (see PROJECT_PLAN §7b): joint synth+real training at ~8:1 lets the
synthetic distribution dominate and HURT real performance (v2_mix: 0.542 vs
0.566 real-only). Pretraining on synth then fine-tuning on REAL ONLY re-anchors
the weights on real pixels while keeping synth-learned shape priors.

Val is real held-out throughout, so metrics stay comparable to runs/cuboid_v1
(real-only baseline: best mAP50 = 0.566).

  scratchpad/train-venv/bin/python train_two_stage.py
"""
from pathlib import Path
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent


def main():
    # ---- Stage 1: pretrain on ALL synthetic (val on real to pick best transfer) ----
    m = YOLO("yolo11n.pt")                      # COCO-pretrained
    r1 = m.train(
        data=str(ROOT / "synth_pretrain.yaml"),
        epochs=80, patience=20, imgsz=640, batch=16, device="mps",
        project=str(ROOT / "runs"), name="cuboid_v3_synthpre", plots=True,
    )
    stage1 = Path(r1.save_dir) / "weights" / "best.pt"
    print(f"[two-stage] stage-1 best: {stage1}")

    # ---- Stage 2: fine-tune on REAL ONLY, lower LR ----
    m2 = YOLO(str(stage1))
    m2.train(
        data=str(ROOT / "dataset" / "data.yaml"),
        epochs=60, patience=20, imgsz=640, batch=8, device="mps",
        lr0=0.001,                              # gentle fine-tune, don't wipe priors
        project=str(ROOT / "runs"), name="cuboid_v3_finetune", plots=True,
    )
    print("[two-stage] done -> compare runs/cuboid_v3_finetune vs cuboid_v1 (0.566)")


if __name__ == "__main__":
    main()
