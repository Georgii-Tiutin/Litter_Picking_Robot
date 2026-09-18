#!/usr/bin/env python
"""Segment the map into rooms; for each: centre, placement zone, boustrophedon sweep."""
import sys, json, yaml, numpy as np, cv2

MAP_YAML = sys.argv[1]
ROBOT_RADIUS_M   = 0.18     # M3 Pro half-diagonal, generous
WALL_CLEARANCE_M = 0.25     # keep the base this far from obstacles
SWEEP_PITCH_M    = 0.60     # lateral spacing between sweep lanes
DOOR_MAX_M       = 0.90     # openings narrower than this separate rooms
MIN_ROOM_M2      = 2.0

meta = yaml.safe_load(open(MAP_YAML))
res  = float(meta["resolution"]); ox, oy = meta["origin"][0], meta["origin"][1]
img  = cv2.imread(MAP_YAML.rsplit("/",1)[0] + "/" + meta["image"], cv2.IMREAD_GRAYSCALE)
H, W = img.shape
print(f"map {W}x{H} px @ {res} m/px  ->  {W*res:.1f} x {H*res:.1f} m")

free = (img > 250).astype(np.uint8)          # trinary: 254 free, 0 occupied, 205 unknown
occ  = (img < 100).astype(np.uint8)
print(f"free {free.sum()*res*res:6.1f} m2   occupied {occ.sum()*res*res:5.1f} m2")

def px2world(c, r):  return (ox + c*res, oy + (H-1-r)*res)
def world2px(x, y):  return (int(round((x-ox)/res)), int(round(H-1-(y-oy)/res)))

# navigable = free, eroded by the clearance radius
k = int(round(WALL_CLEARANCE_M/res))
nav = cv2.erode(free, np.ones((2*k+1, 2*k+1), np.uint8))
print(f"navigable after {WALL_CLEARANCE_M} m clearance: {nav.sum()*res*res:.1f} m2")

# rooms: open with a kernel wider than a doorway, then label
d = int(round(DOOR_MAX_M/res))
seeds = cv2.morphologyEx(free, cv2.MORPH_OPEN, np.ones((d, d), np.uint8))
ns, seed_lab, seed_st, _ = cv2.connectedComponentsWithStats(seeds, 8)
keep_ids = [i for i in range(1, ns) if seed_st[i, cv2.CC_STAT_AREA]*res*res >= MIN_ROOM_M2]
markers = np.zeros(free.shape, np.int32)
for j, i in enumerate(keep_ids, start=1):
    markers[seed_lab == i] = j
markers[free == 0] = len(keep_ids) + 1                 # everything not free = background
cv2.watershed(cv2.cvtColor(free*255, cv2.COLOR_GRAY2BGR), markers)
lab = np.where((markers > 0) & (markers <= len(keep_ids)), markers, 0)
n = len(keep_ids) + 1
st = np.zeros((n, 5), int)
for j in range(1, n):
    st[j, cv2.CC_STAT_AREA] = int((lab == j).sum())
rooms = []
for i in range(1, n):
    area = st[i, cv2.CC_STAT_AREA]*res*res
    if area < MIN_ROOM_M2: continue
    comp_free = (lab == i).astype(np.uint8)
    comp = cv2.bitwise_and(comp_free, nav)
    if comp.sum()*res*res < 0.5: continue
    # centre = deepest point inside the room (max distance from its own boundary)
    dist = cv2.distanceTransform(comp_free, cv2.DIST_L2, 5)
    r_c, c_c = np.unravel_index(np.argmax(dist), dist.shape)
    cx, cy = px2world(c_c, r_c)
    rooms.append(dict(id=len(rooms)+1, area_m2=round(area,2),
                      centre=[round(cx,3), round(cy,3)],
                      clearance_m=round(float(dist[r_c, c_c])*res,2),
                      comp=comp))
rooms.sort(key=lambda r:-r["area_m2"])
for j,r in enumerate(rooms,1): r["id"]=j

def sweep(comp):
    """Boustrophedon lanes across the room, in world coords."""
    ys, xs = np.where(comp > 0)
    if len(xs)==0: return []
    pitch = max(1, int(round(SWEEP_PITCH_M/res)))
    lanes, flip = [], False
    for r in range(ys.min(), ys.max()+1, pitch):
        cols = np.where(comp[r] > 0)[0]
        if len(cols) < 3: continue
        # split the row into contiguous runs (a room can be non-convex)
        runs, start = [], cols[0]
        for a,b in zip(cols, cols[1:]):
            if b-a > 1: runs.append((start,a)); start=b
        runs.append((start, cols[-1]))
        for c0,c1 in runs:
            if (c1-c0)*res < 0.4: continue
            p0, p1 = px2world(c0, r), px2world(c1, r)
            lanes.append([p1,p0] if flip else [p0,p1]); flip = not flip
    return lanes

out = []
print(f"\n{'room':>5}{'area m2':>9}{'centre (x,y)':>20}{'clear m':>9}{'lanes':>7}{'path m':>8}")
for r in rooms:
    lanes = sweep(r["comp"])
    L = sum(float(np.hypot(b[0]-a[0], b[1]-a[1])) for a,b in lanes)
    print(f"{r['id']:>5}{r['area_m2']:>9.1f}   ({r['centre'][0]:+6.2f},{r['centre'][1]:+6.2f}){r['clearance_m']:>9.2f}{len(lanes):>7}{L:>8.1f}")
    out.append(dict(id=r["id"], area_m2=r["area_m2"], centre=r["centre"],
                    clearance_m=r["clearance_m"], lanes=[[list(a),list(b)] for a,b in lanes],
                    path_len_m=round(L,1)))
json.dump(dict(map=meta["image"], resolution=res, origin=[ox,oy],
               place_zone_m=0.10, rooms=out), open("rooms.json","w"), indent=2)

vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
cols = [(0,180,0),(220,120,0),(0,120,220),(180,0,180),(0,180,180),(120,120,0)]
for r,o in zip(rooms,out):
    c = cols[(r["id"]-1) % len(cols)]
    vis[r["comp"]>0] = (0.75*vis[r["comp"]>0] + 0.25*np.array(c)).astype(np.uint8)
    for a,b in o["lanes"]:
        cv2.line(vis, world2px(*a), world2px(*b), c, 1)
    pc = world2px(*r["centre"])
    half = int(round(0.05/res))
    cv2.rectangle(vis, (pc[0]-half,pc[1]-half), (pc[0]+half,pc[1]+half), (0,0,255), 2)
    cv2.putText(vis, str(r["id"]), (pc[0]+8,pc[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
cv2.imwrite("rooms_plan.png", cv2.resize(vis, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
print("\nsaved rooms.json and rooms_plan.png")
