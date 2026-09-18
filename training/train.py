#!/usr/bin/env python3
"""Train a YOLO cuboid detector (baseline) on the CVAT-labelled dataset.

Run with the training venv's python:
  scratchpad/train-venv/bin/python train.py
"""
from pathlib import Path
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "dataset" / "data.yaml"

def main():
    # COCO-pretrained nano model -> fine-tune (transfer learning).
    model = YOLO("yolo11n.pt")
    model.train(
        data=str(DATA),
        epochs=100,
        imgsz=640,
        batch=16,            # lower to 8 if the Mac runs out of memory
        device="mps",        # Apple Silicon GPU
        patience=20,         # early stop if no val improvement for 20 epochs
        project=str(ROOT / "runs"),
        name="cuboid_v1",
        plots=True,
    )

if __name__ == "__main__":
    main()
