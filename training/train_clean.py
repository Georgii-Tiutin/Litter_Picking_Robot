#!/usr/bin/env python3
"""Real-only training on the label-cleaned dataset (no synthetic).

  scratchpad/train-venv/bin/python train_clean.py
"""
from pathlib import Path
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "dataset_clean" / "data.yaml"

def main():
    YOLO("yolo11n.pt").train(
        data=str(DATA), epochs=100, patience=20, imgsz=640, batch=16,
        device="mps", time=3, project=str(ROOT/"runs"), name="cuboid_v3_clean", plots=True)

if __name__ == "__main__":
    main()
