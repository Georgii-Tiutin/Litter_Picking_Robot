#!/usr/bin/env python3
"""Build dataset_clean/: original GT + size-filtered high-confidence pseudo-labels.

Adds detections that are (a) unmatched by GT (IoU<0.4) and (b) SMALL
(max side < SIZE_GATE, which excludes the tall black bin) -> fills the missing
cream-cube labels while keeping large clutter (bin) unlabelled.

  scratchpad/train-venv/bin/python synth/pseudo_complete_labels.py
"""
from pathlib import Path
from ultralytics import YOLO

ROOT = Path("/Users/georgiitiutin/ProjectsRoot/Making Data")
SRC = ROOT / "dataset"
DST = ROOT / "dataset_clean"
MODEL = ROOT / "runs/cuboid_v1/weights/best.pt"
CONF, IOU_MATCH, SIZE_GATE = 0.50, 0.40, 0.30   # size gate excludes the big bin

def to_xyxy(xc, yc, w, h): return (xc-w/2, yc-h/2, xc+w/2, yc+h/2)
def iou(a, b):
    ix0,iy0,ix1,iy1 = max(a[0],b[0]),max(a[1],b[1]),min(a[2],b[2]),min(a[3],b[3])
    iw,ih = max(0,ix1-ix0),max(0,iy1-iy0); inter=iw*ih
    if inter<=0: return 0.0
    return inter/((a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter)

model = YOLO(str(MODEL))
added = kept = 0
for split in ("train", "val"):
    (DST/"images").mkdir(parents=True, exist_ok=True)
    (DST/"labels"/split).mkdir(parents=True, exist_ok=True)
    # symlink the images dir (Ultralytics derives labels via /images/->/labels/)
    link = DST/"images"/split
    if not link.exists():
        link.symlink_to(SRC/"images"/split)
    for img in sorted((SRC/"images"/split).glob("*.*")):
        gt_lines, gt_boxes = [], []
        lf = SRC/"labels"/split/(img.stem+".txt")
        if lf.exists():
            for ln in lf.read_text().splitlines():
                p = ln.split()
                if len(p) == 5:
                    gt_lines.append(ln.strip()); gt_boxes.append(to_xyxy(*map(float,p[1:])))
        kept += len(gt_lines)
        r = model.predict(str(img), conf=CONF, verbose=False)[0]
        H, W = r.orig_shape
        new = []
        for x0,y0,x1,y1 in r.boxes.xyxy.tolist():
            d = (x0/W, y0/H, x1/W, y1/H)
            w, h = d[2]-d[0], d[3]-d[1]
            if max(w, h) >= SIZE_GATE:            # too big -> likely the bin, skip
                continue
            if all(iou(d, g) < IOU_MATCH for g in gt_boxes):
                new.append(f"0 {(d[0]+d[2])/2:.6f} {(d[1]+d[3])/2:.6f} {w:.6f} {h:.6f}")
        added += len(new)
        out = gt_lines + new
        if out:
            (DST/"labels"/split/(img.stem+".txt")).write_text("\n".join(out)+"\n")

(DST/"data.yaml").write_text(
    f"path: {DST}\ntrain: images/train\nval: images/val\nnc: 1\nnames:\n  0: cuboid\n")
print(f"kept original GT: {kept}  |  added pseudo-labels: {added}")
print(f"clean dataset -> {DST}")
