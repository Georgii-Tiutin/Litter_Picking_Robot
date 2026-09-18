#!/usr/bin/env python3
"""Convert the room generator's YOLO-OBB labels (8 coords) to axis-aligned YOLO
detection labels (class cx cy w h) for apples-to-apples comparison with the
existing axis-aligned real data + baseline. OBB + vertex data is preserved in
synth/rooms_output/labels and /meta; this only produces a parallel AABB view.

Creates:
  synth/rooms_aabb/images  -> symlink to ../rooms_output/images
  synth/rooms_aabb/labels  -> converted axis-aligned .txt (Ultralytics maps
                              images/ -> labels/ within rooms_aabb)

  python3 synth/obb_to_aabb.py
"""
from pathlib import Path

HERE = Path(__file__).parent
SRC_IMG = HERE / "rooms_output" / "images"
SRC_LBL = HERE / "rooms_output" / "labels"
DST = HERE / "rooms_aabb"


def obb_line_to_aabb(line):
    p = line.split()
    if len(p) != 9:
        return None
    cls = p[0]
    xs = [float(p[i]) for i in (1, 3, 5, 7)]
    ys = [float(p[i]) for i in (2, 4, 6, 8)]
    x0, x1 = min(xs), max(xs); y0, y1 = min(ys), max(ys)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    w, h = x1 - x0, y1 - y0
    return f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"


def main():
    (DST / "labels").mkdir(parents=True, exist_ok=True)
    img_link = DST / "images"
    if not img_link.exists():
        img_link.symlink_to(Path("..") / "rooms_output" / "images")
    n = 0
    for lbl in sorted(SRC_LBL.glob("*.txt")):
        out = []
        for line in lbl.read_text().splitlines():
            a = obb_line_to_aabb(line)
            if a:
                out.append(a)
        (DST / "labels" / lbl.name).write_text("\n".join(out) + ("\n" if out else ""))
        n += 1
    print(f"[obb->aabb] converted {n} label files into {DST}")


if __name__ == "__main__":
    main()
