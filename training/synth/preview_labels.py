#!/usr/bin/env python3
"""Draw YOLO labels onto renders + build a contact sheet.

Run: scratchpad/train-venv/bin/python synth/preview_labels.py
"""
from pathlib import Path
from PIL import Image, ImageDraw

OUT = Path("/Users/georgiitiutin/ProjectsRoot/Making Data/synth/output")
PREVIEW = Path("/Users/georgiitiutin/ProjectsRoot/Making Data/synth/preview")
PREVIEW.mkdir(parents=True, exist_ok=True)

previews = []
imgs = sorted((OUT / "images").glob("*.png"))
total_boxes = 0
for img_path in imgs:
    lbl = OUT / "labels" / (img_path.stem + ".txt")
    im = Image.open(img_path).convert("RGB"); W, H = im.size
    d = ImageDraw.Draw(im)
    if lbl.exists():
        for line in lbl.read_text().splitlines():
            if not line.strip():
                continue
            _, xc, yc, w, h = (float(v) for v in line.split())
            d.rectangle([(xc - w / 2) * W, (yc - h / 2) * H, (xc + w / 2) * W, (yc + h / 2) * H],
                        outline=(255, 0, 0), width=3)
            total_boxes += 1
    im.save(PREVIEW / img_path.name)
    previews.append(im)

# contact sheet (up to 36 in a grid)
cell, cols = 200, 6
sel = previews[:36]
rows = (len(sel) + cols - 1) // cols
sheet = Image.new("RGB", (cols * cell, rows * cell), (30, 30, 30))
for idx, im in enumerate(sel):
    th = im.resize((cell, cell))
    sheet.paste(th, ((idx % cols) * cell, (idx // cols) * cell))
sheet.save(PREVIEW / "contact_sheet.png")
print(f"{len(imgs)} images, {total_boxes} boxes. Contact sheet -> {PREVIEW/'contact_sheet.png'}")
