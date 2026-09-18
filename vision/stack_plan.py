#!/usr/bin/env python
"""Stacking planner: order objects, choose grip angle, emit pick/place poses.

Design rules taken from the course findings:
  - grip across the SHORTER side of minAreaRect (section 6: vendor takes an
    arbitrary contour edge and fails on elongated objects)
  - reject if the short side exceeds the claw opening
  - approach and retract vertically, never diagonally across the stack
  - biggest footprint at the bottom
"""
import numpy as np, cv2

CLAW_MAX_MM   = 55.0     # usable jaw opening (measured 3x6cm block fits, 6cm does not)
CLAW_MIN_MM   = 12.0     # below this the jaws close on nothing
PX_PER_MM     = 1.35     # floor scale, overhead camera

def object_geometry(mask_component):
    """minAreaRect -> (centre_px, short_mm, long_mm, angle_deg_of_short_axis)"""
    pts = cv2.findNonZero(mask_component.astype(np.uint8))
    (cx, cy), (w, h), ang = cv2.minAreaRect(pts)
    w_mm, h_mm = w / PX_PER_MM, h / PX_PER_MM
    if w_mm <= h_mm:
        short, long_, short_ang = w_mm, h_mm, ang
    else:
        short, long_, short_ang = h_mm, w_mm, ang + 90.0
    return (cx, cy), short, long_, short_ang

def graspable(short_mm):
    return CLAW_MIN_MM <= short_mm <= CLAW_MAX_MM

def plan_order(objs):
    """Largest footprint first: a wide base is what makes a tall tower survive."""
    return sorted(objs, key=lambda o: -(o["short_mm"] * o["long_mm"]))

def stack_heights(objs_in_order, heights_mm):
    """Cumulative z of the TOP of the stack after each placement."""
    tops, z = [], 0.0
    for h in heights_mm:
        z += h; tops.append(z)
    return tops

if __name__ == "__main__":
    print("rules:")
    print(f"  claw window        {CLAW_MIN_MM:.0f} - {CLAW_MAX_MM:.0f} mm (short side)")
    print(f"  floor scale        {PX_PER_MM} px/mm")
    print("  order              largest footprint first")
    print("  grip axis          across minAreaRect short side")
