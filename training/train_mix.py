#!/usr/bin/env python3
"""Train YOLO cuboid detector on SYNTHETIC + REAL (mixed) data.

  scratchpad/train-venv/bin/python train_mix.py

Val stays real-only (held-out videos) so metrics remain honest and comparable
to the baseline (runs/cuboid_v1).
"""
from pathlib import Path
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "mix_data.yaml"

def main():
    model = YOLO("yolo11n.pt")            # COCO-pretrained -> fine-tune
    model.train(
        data=str(DATA),
        epochs=100,
        time=6,           # hard cap: stop after ~6 h regardless
        patience=20,      # early-stop if val plateaus
        imgsz=640,
        batch=8,          # lowered for memory safety on the larger mixed set
        device="mps",
        project=str(ROOT / "runs"),
        name="cuboid_v2_mix",
        plots=True,
    )

if __name__ == "__main__":
    main()
