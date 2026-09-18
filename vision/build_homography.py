#!/usr/bin/env python
"""Fit image->world homography from the four corner tags.

World frame: origin at tag 2 (top-left), X along the top edge, Y down the left
edge, millimetres, in the plane of the tag faces (30 mm above the floor).
"""
import sys, json, numpy as np, cv2
from pupil_apriltags import Detector

img_path = sys.argv[1]
W = float(sys.argv[2]) if len(sys.argv) > 2 else 800.0   # centre-to-centre, mm
H_ = float(sys.argv[3]) if len(sys.argv) > 3 else 450.0

WORLD = {2: (0.0, 0.0), 1: (W, 0.0), 3: (0.0, H_), 4: (W, H_)}

gray = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_BGR2GRAY)
tags = {t.tag_id: t for t in Detector(families="tag36h11", nthreads=4,
        quad_decimate=1.0, refine_edges=1).detect(gray)}

missing = [i for i in WORLD if i not in tags]
if missing:
    sys.exit(f"missing tags {missing} - cannot fit")

src = np.array([tags[i].center for i in (2, 1, 3, 4)], dtype=np.float32)
dst = np.array([WORLD[i]       for i in (2, 1, 3, 4)], dtype=np.float32)
Himg2world = cv2.getPerspectiveTransform(src, dst)

def to_world(pts):
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, Himg2world).reshape(-1, 2)

print(f"world rectangle: {W:.0f} x {H_:.0f} mm (centre-to-centre)\n")
print("tag size implied by the fit (real check of consistency):")
sizes = []
for i in sorted(tags):
    c = to_world(tags[i].corners)
    s = [float(np.linalg.norm(c[k] - c[(k+1) % 4])) for k in range(4)]
    sizes.extend(s)
    print(f"  id{i}: sides {s[0]:5.1f} {s[1]:5.1f} {s[2]:5.1f} {s[3]:5.1f} mm"
          f"   mean {np.mean(s):5.1f}")
print(f"\n  overall mean {np.mean(sizes):.2f} mm, sd {np.std(sizes):.2f} mm "
      f"({100*np.std(sizes)/np.mean(sizes):.1f}%)")

json.dump({"H_img2world": Himg2world.tolist(),
           "world_w_mm": W, "world_h_mm": H_,
           "source_image": img_path,
           "tag_centres_px": {str(i): list(map(float, tags[i].center)) for i in sorted(tags)}},
          open("homography.json", "w"), indent=2)
print("\nsaved homography.json")
