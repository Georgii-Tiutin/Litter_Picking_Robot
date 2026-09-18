#!/usr/bin/env python3
"""Find labelling gaps: detections with no matching GT (candidate MISSING labels)
and GT with no detection (possible over-/mis-labels). Saves a review contact sheet.

  scratchpad/train-venv/bin/python synth/scan_label_gaps.py
"""
from pathlib import Path
from ultralytics import YOLO
from PIL import Image, ImageDraw

ROOT = Path("/Users/georgiitiutin/ProjectsRoot/Making Data")
MODEL = ROOT / "runs/cuboid_v1/weights/best.pt"     # real-trained, cleaner recall
OUT = ROOT / "synth/label_review"
CONF, IOU_MATCH = 0.40, 0.40
OUT.mkdir(parents=True, exist_ok=True)


def to_xyxy(xc, yc, w, h):
    return (xc - w / 2, yc - h / 2, xc + w / 2, yc + h / 2)


def iou(a, b):
    ix0, iy0 = max(a[0], b[0]), max(a[1], b[1])
    ix1, iy1 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix1 - ix0), max(0, iy1 - iy0)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua


model = YOLO(str(MODEL))
rows, flagged = [], []
tot_missing = tot_unmatched_gt = tot_gt = tot_det = 0

for split in ("train", "val"):
    for img in sorted((ROOT / f"dataset/images/{split}").glob("*.*")):
        lf = ROOT / f"dataset/labels/{split}" / (img.stem + ".txt")
        gt = []
        if lf.exists():
            for line in lf.read_text().splitlines():
                p = line.split()
                if len(p) == 5:
                    gt.append(to_xyxy(*map(float, p[1:])))
        r = model.predict(str(img), conf=CONF, verbose=False)[0]
        H, W = r.orig_shape
        dets = [(float(x0)/W, float(y0)/H, float(x1)/W, float(y1)/H)
                for x0, y0, x1, y1 in r.boxes.xyxy.tolist()]
        gt_hit = [False] * len(gt)
        missing = []                       # detections with no GT match
        for d in dets:
            best, bi = 0.0, -1
            for i, g in enumerate(gt):
                v = iou(d, g)
                if v > best:
                    best, bi = v, i
            if best >= IOU_MATCH:
                gt_hit[bi] = True
            else:
                missing.append(d)
        unmatched_gt = gt_hit.count(False)
        tot_missing += len(missing); tot_unmatched_gt += unmatched_gt
        tot_gt += len(gt); tot_det += len(dets)
        if missing or unmatched_gt:
            rows.append((split, img.name, len(gt), len(dets), len(missing), unmatched_gt))
        if missing:                        # save review image for candidate missing labels
            im = Image.open(img).convert("RGB"); dr = ImageDraw.Draw(im)
            for g in gt:                    # GT = green
                dr.rectangle([g[0]*W, g[1]*H, g[2]*W, g[3]*H], outline=(0, 220, 0), width=3)
            for d in missing:               # candidate missing = red
                dr.rectangle([d[0]*W, d[1]*H, d[2]*W, d[3]*H], outline=(255, 0, 0), width=4)
            im.save(OUT / f"{split}__{img.name}")
            flagged.append((len(missing), OUT / f"{split}__{img.name}"))

# contact sheet of worst offenders
flagged.sort(reverse=True)
sel = [Image.open(p).convert("RGB") for _, p in flagged[:36]]
if sel:
    cell, cols = 240, 6
    rowsn = (len(sel) + cols - 1) // cols
    sheet = Image.new("RGB", (cols*cell, rowsn*cell), (25, 25, 25))
    for i, im in enumerate(sel):
        sheet.paste(im.resize((cell, cell)), ((i % cols)*cell, (i//cols)*cell))
    sheet.save(OUT / "GAPS_contact_sheet.png")

print(f"GT boxes total: {tot_gt} | detections total: {tot_det}")
print(f"Candidate MISSING labels (det, no GT): {tot_missing}  across {len(flagged)} images")
print(f"Unmatched GT (GT, no det): {tot_unmatched_gt}")
print(f"Images flagged: {len(rows)}  |  review images + GAPS_contact_sheet.png in {OUT}")
print("\nWorst images (split, name, #gt, #det, #missing, #unmatched_gt):")
for r in sorted(rows, key=lambda x: -x[4])[:15]:
    print("  ", r)
