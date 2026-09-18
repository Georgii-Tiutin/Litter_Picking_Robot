#!/usr/bin/env python3
"""Realistic-room synthetic cuboid generator (v2) — structured domain randomisation.

Composes scenes from Poly Haven CC0 assets (see fetch_assets.py):
  random indoor HDRI (lighting+background) + PBR floor + furniture/clutter,
  with graspable TARGET cubes physics-settled onto the floor and rendered from
  the robot's low eye-in-hand SEARCH view (camera 0.10-0.50 m off the ground,
  16:9 @ ~86 deg H FOV to match the Orbbec DCW2).

Everything is in REAL METRES (Poly Haven furniture is metric; cubes are 2-5.5 cm).

Labels (auto):
  - YOLO-OBB  : class + 4 oriented-box corners (min-area rect of the amodal
                silhouette = convex hull of the 8 projected cube vertices).
  - amodal    : box drawn through occluders; kept only if >=30% of the cube's
                front-facing silhouette is visible (dense ray-sampling), which
                also drops heavily-truncated cubes.
  - sidecar   : 8 projected vertices + visibility + settled dims/yaw (JSON),
                so corner/6-DoF experiments need no re-render.
Distractors (big boxes, cylinders, spheres, cones, furniture) are NEVER labelled.

Run headless:
  blender --background --python synth/generate_rooms.py -- --n 2000
"""
import bpy, bmesh, math, random, colorsys, json, sys, argparse
import numpy as np
from pathlib import Path
from mathutils import Vector, Matrix
from bpy_extras.object_utils import world_to_camera_view

# ---------------- config ----------------
HERE = Path(__file__).parent
ASSETS = HERE / "assets"
OUT = HERE / "rooms_output"
RES_X, RES_Y = 1280, 720          # 16:9, matches real 1920x1080 after letterbox
FOV_H_DEG = 86.0                  # DCW2 default 16:9 horizontal FOV
SAMPLES = 16                      # EEVEE TAA
CAM_H = (0.10, 0.50)             # camera height band (arm reach: 0.30 m sphere @ 0.20 m)
CAM_DIST = (0.4, 2.5)           # camera distance from the cube cluster (m): near->big, far->small-in-scene
CUBE_SPREAD = 0.7               # cubes scattered within this radius of origin (m): spread + frame-edge cases
MIN_BOX = 0.008                 # drop labels smaller than ~10 px (degenerate far cubes)
TARGET_NARROW = (0.02, 0.055)   # narrowest edge (graspable) in m
TARGET_OTHER = (0.02, 0.09)     # other edges in m
VIS_KEEP = 0.30                 # keep target if >=30% of silhouette visible
SETTLE_FRAMES = 55              # physics steps for cubes to come to rest
ABSTRACT_PROB = 0.20            # fraction rendered plain (no furniture / solid floor)


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--start", type=int, default=0, help="first image index (for resuming)")
    return ap.parse_args(argv)


# ---------------- asset discovery ----------------
def discover():
    hdris = sorted(ASSETS.glob("hdris/*.hdr")) + sorted(ASSETS.glob("hdris/*.exr"))
    floors = [d for d in sorted((ASSETS / "floors").glob("*")) if d.is_dir()]
    models = []
    for d in sorted((ASSETS / "models").glob("*")):
        g = list(d.glob("*.gltf")) + list(d.glob("*.glb"))
        if g:
            models.append(g[0])
    return hdris, floors, models


# ---------------- colour helpers ----------------
def rand_color():
    r = random.random()
    if r < 0.7:
        return colorsys.hsv_to_rgb(random.random(), random.uniform(0.5, 1), random.uniform(0.5, 1))
    if r < 0.85:
        return colorsys.hsv_to_rgb(random.random(), random.uniform(0.1, 0.4), random.uniform(0.6, 1))
    g = random.uniform(0.0, 1.0); return (g, g, g)


def make_mat(color, rough=None):
    m = bpy.data.materials.new("m"); m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = random.uniform(0.2, 0.9) if rough is None else rough
    return m


# ---------------- scene setup ----------------
def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    s = bpy.context.scene
    s.render.engine = 'BLENDER_EEVEE'
    s.render.resolution_x = RES_X; s.render.resolution_y = RES_Y
    s.render.image_settings.file_format = 'PNG'
    if hasattr(s, "eevee"):
        try: s.eevee.taa_render_samples = SAMPLES
        except Exception: pass
    return s


def setup_world(s, hdris, abstract):
    w = bpy.data.worlds.new("W"); s.world = w; w.use_nodes = True
    nt = w.node_tree; bg = nt.nodes["Background"]
    bg.inputs[1].default_value = random.uniform(0.4, 1.6)
    if hdris and (not abstract or random.random() < 0.7):
        env = nt.nodes.new("ShaderNodeTexEnvironment")
        env.image = bpy.data.images.load(str(random.choice(hdris)))
        mp = nt.nodes.new("ShaderNodeMapping"); tc = nt.nodes.new("ShaderNodeTexCoord")
        mp.inputs["Rotation"].default_value[2] = random.uniform(0, 2 * math.pi)
        nt.links.new(tc.outputs["Generated"], mp.inputs["Vector"])
        nt.links.new(mp.outputs["Vector"], env.inputs["Vector"])
        nt.links.new(env.outputs["Color"], bg.inputs[0])
    else:
        c = colorsys.hsv_to_rgb(random.random(), random.uniform(0, 0.4), random.uniform(0.5, 1))
        bg.inputs[0].default_value = (*c, 1)


def setup_floor(floors, abstract):
    bpy.ops.mesh.primitive_plane_add(size=12, location=(0, 0, 0))
    g = bpy.context.active_object
    m = bpy.data.materials.new("floor"); m.use_nodes = True
    nt = m.node_tree; bsdf = nt.nodes["Principled BSDF"]
    fdir = None if abstract else (random.choice(floors) if floors else None)
    if fdir:
        tc = nt.nodes.new("ShaderNodeTexCoord"); mp = nt.nodes.new("ShaderNodeMapping")
        scl = random.uniform(2.0, 8.0)                      # UV tiles across the 12 m plane
        for i in range(3): mp.inputs["Scale"].default_value[i] = scl
        mp.inputs["Rotation"].default_value[2] = random.uniform(0, 2 * math.pi)
        nt.links.new(tc.outputs["UV"], mp.inputs["Vector"])
        def load(name, non_color=False):
            f = list(fdir.glob(f"{name}.*"))
            if not f: return None
            img = bpy.data.images.load(str(f[0]))
            if non_color: img.colorspace_settings.name = 'Non-Color'
            t = nt.nodes.new("ShaderNodeTexImage"); t.image = img
            nt.links.new(mp.outputs["Vector"], t.inputs["Vector"])
            return t
        d = load("Diffuse")
        if d: nt.links.new(d.outputs["Color"], bsdf.inputs["Base Color"])
        r = load("Rough", non_color=True)
        if r: nt.links.new(r.outputs["Color"], bsdf.inputs["Roughness"])
        n = load("nor_gl", non_color=True)
        if n:
            nm = nt.nodes.new("ShaderNodeNormalMap")
            nt.links.new(n.outputs["Color"], nm.inputs["Color"])
            nt.links.new(nm.outputs["Normal"], bsdf.inputs["Normal"])
    else:
        bsdf.inputs["Base Color"].default_value = (*rand_color(), 1)
        bsdf.inputs["Roughness"].default_value = random.uniform(0.5, 1.0)
    g.data.materials.append(m)
    return g


def place_furniture(models, k, cam_az):
    """Import k glTF models, rest each on the floor (never labelled).
    Placed in the hemisphere AWAY from the camera so they form background context
    behind/beside the cubes rather than occluding the low foreground camera view."""
    for _ in range(k):
        path = random.choice(models)
        before = set(bpy.data.objects)
        try:
            bpy.ops.import_scene.gltf(filepath=str(path))
        except Exception as e:
            print(f"  [furniture] import failed {path.name}: {e}"); continue
        new = [o for o in bpy.data.objects if o not in before]
        meshes = [o for o in new if o.type == 'MESH']
        if not meshes: continue
        empty = bpy.data.objects.new("furn", None)
        bpy.context.scene.collection.objects.link(empty)
        for o in new:
            if o.parent is None: o.parent = empty
        bpy.context.view_layer.update()
        zmin = min((o.matrix_world @ Vector(c)).z for o in meshes for c in o.bound_box)
        ang = cam_az + math.radians(random.uniform(90, 270))   # avoid the camera->cube wedge
        r = random.uniform(1.0, 3.0)
        empty.location = (r * math.cos(ang), r * math.sin(ang), -zmin)
        empty.rotation_euler = (0, 0, random.uniform(0, 2 * math.pi))


# ---------------- cubes & distractors ----------------
def add_target():
    lo = random.uniform(*TARGET_NARROW)
    dims = [lo, random.uniform(lo, TARGET_OTHER[1]), random.uniform(lo, TARGET_OTHER[1])]
    random.shuffle(dims)
    ang = random.uniform(0, 2 * math.pi); r = random.uniform(0, CUBE_SPREAD)
    z = random.uniform(0.05, 0.20) + max(dims)   # drop from a small height
    bpy.ops.mesh.primitive_cube_add(size=1, location=(r * math.cos(ang), r * math.sin(ang), z))
    o = bpy.context.active_object
    o.scale = dims
    o.rotation_euler = (random.uniform(0, math.pi), random.uniform(0, math.pi), random.uniform(0, math.pi))
    # tiny random bevel -> kills razor-sharp CG edges (a sim-to-real tell)
    bev = o.modifiers.new("bev", 'BEVEL')
    bev.width = random.uniform(0.0005, 0.002); bev.segments = 2
    # colour: solid or per-face
    o.data.materials.clear()
    if random.random() < 0.4:
        cols = [rand_color() for _ in range(6)]
        for c in cols: o.data.materials.append(make_mat(c))
        for poly in o.data.polygons: poly.material_index = random.randrange(6)
    else:
        o.data.materials.append(make_mat(rand_color()))
    o["is_target"] = 1
    return o


def add_distractor():
    kind = random.choice(['box', 'box', 'cylinder', 'sphere', 'cone'])
    ang = random.uniform(0, 2 * math.pi); r = random.uniform(0, 0.6)
    loc = (r * math.cos(ang), r * math.sin(ang), 0.3)
    if kind == 'box':
        dims = [random.uniform(0.09, 0.4) for _ in range(3)]
        bpy.ops.mesh.primitive_cube_add(size=1, location=loc); o = bpy.context.active_object
        o.scale = dims; o.rotation_euler = (0, 0, random.uniform(0, math.pi))
    elif kind == 'cylinder':
        bpy.ops.mesh.primitive_cylinder_add(radius=random.uniform(0.02, 0.08),
                                            depth=random.uniform(0.05, 0.25), location=loc)
        o = bpy.context.active_object
    elif kind == 'sphere':
        bpy.ops.mesh.primitive_uv_sphere_add(radius=random.uniform(0.03, 0.1), location=loc)
        o = bpy.context.active_object
    else:
        bpy.ops.mesh.primitive_cone_add(radius1=random.uniform(0.03, 0.09),
                                        depth=random.uniform(0.06, 0.22), location=loc)
        o = bpy.context.active_object
    o.data.materials.clear(); o.data.materials.append(make_mat(rand_color()))
    o["is_target"] = 0
    return o


# ---------------- physics ----------------
def settle(scene, actives, floor):
    bpy.ops.rigidbody.world_add()
    scene.rigidbody_world.point_cache.frame_start = 1
    scene.rigidbody_world.point_cache.frame_end = SETTLE_FRAMES
    def add_rb(obj, kind, shape):
        bpy.context.view_layer.objects.active = obj
        bpy.ops.rigidbody.object_add()
        obj.rigid_body.type = kind
        obj.rigid_body.collision_shape = shape
    add_rb(floor, 'PASSIVE', 'MESH')
    for o in actives:
        add_rb(o, 'ACTIVE', 'CONVEX_HULL')
    for f in range(1, SETTLE_FRAMES + 1):
        scene.frame_set(f)


# ---------------- camera ----------------
def add_camera(scene):
    cd = bpy.data.cameras.new("Cam"); co = bpy.data.objects.new("Cam", cd)
    scene.collection.objects.link(co); scene.camera = co
    cd.sensor_fit = 'HORIZONTAL'
    cd.angle = math.radians(FOV_H_DEG + random.uniform(-3, 3))
    ang = random.uniform(0, 2 * math.pi); dist = random.uniform(*CAM_DIST)
    h = random.uniform(*CAM_H)
    co.location = (dist * math.cos(ang), dist * math.sin(ang), h)
    look = Vector((random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1), random.uniform(0.0, 0.08)))
    d = look - co.location
    co.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
    co.rotation_euler.rotate_axis('Z', math.radians(random.uniform(-5, 5)))
    return co, ang


# ---------------- visibility & projection ----------------
def face_samples(mesh, mw, grid=5):
    """Dense world-space sample points + world normals over each quad/tri face."""
    out = []
    for poly in mesh.polygons:
        vs = [mw @ mesh.vertices[vi].co for vi in poly.vertices]
        n = (mw.to_3x3() @ poly.normal).normalized()
        if len(vs) == 4:
            for i in range(grid):
                for j in range(grid):
                    u = (i + 0.5) / grid; v = (j + 0.5) / grid
                    top = vs[0].lerp(vs[1], u); bot = vs[3].lerp(vs[2], u)
                    out.append((top.lerp(bot, v), n))
        else:
            c = sum(vs, Vector()) / len(vs)
            out.append((c, n))
    return out


def visibility(scene, cam, obj, dg):
    obj_eval = obj.evaluated_get(dg); mesh = obj_eval.to_mesh()
    camloc = cam.matrix_world.translation
    front = vis = 0
    for p, n in face_samples(mesh, obj.matrix_world):
        view = (p - camloc)
        dv = view.normalized()
        if n.dot(dv) >= 0:            # back-facing -> self-occluded, not part of silhouette
            continue
        front += 1
        cc = world_to_camera_view(scene, cam, p)
        if not (0 <= cc.x <= 1 and 0 <= cc.y <= 1 and cc.z > 0):
            continue                 # off-frame -> truncated, counts as not visible
        hit, loc, _, _, hobj, _ = scene.ray_cast(dg, camloc, dv)
        if hit and hobj is not None and hobj.original == obj and (loc - p).length < 0.01:
            vis += 1
    obj_eval.to_mesh_clear()
    return (vis / front) if front else 0.0


def projected_verts(scene, cam, obj):
    """8 cube corners -> normalised image coords (y down). None if fully behind camera."""
    pts = []
    for c in obj.bound_box:
        cc = world_to_camera_view(scene, cam, obj.matrix_world @ Vector(c))
        if cc.z <= 0:
            continue
        pts.append((cc.x, 1.0 - cc.y))
    return np.array(pts) if len(pts) >= 4 else None


def convex_hull(pts):
    pts = sorted(map(tuple, pts))
    if len(pts) < 3: return np.array(pts)
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0: lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0: upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def min_area_rect(pts):
    """4 corners of the minimum-area rectangle around pts (Nx2)."""
    hull = convex_hull(pts)
    if len(hull) < 3:
        x0, y0 = pts.min(0); x1, y1 = pts.max(0)
        return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    best = None
    for i in range(len(hull)):
        e = hull[(i + 1) % len(hull)] - hull[i]
        ang = math.atan2(e[1], e[0])
        c, s = math.cos(-ang), math.sin(-ang)
        R = np.array([[c, -s], [s, c]])
        rp = pts @ R.T
        mn = rp.min(0); mx = rp.max(0)
        area = (mx[0] - mn[0]) * (mx[1] - mn[1])
        if best is None or area < best[0]:
            corners = np.array([[mn[0], mn[1]], [mx[0], mn[1]], [mx[0], mx[1]], [mn[0], mx[1]]])
            Rinv = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
            best = (area, corners @ Rinv.T)
    return best[1]


# ---------------- sensor realism ----------------
def sensor_realism(path):
    img = bpy.data.images.load(str(path))
    w, h = img.size
    px = np.array(img.pixels[:]).reshape(h, w, 4)
    rgb = px[..., :3]
    rgb *= random.uniform(0.85, 1.15)                              # exposure jitter
    rgb += np.random.normal(0, random.uniform(0.005, 0.03), rgb.shape)  # sensor noise
    px[..., :3] = np.clip(rgb, 0, 1)
    img.pixels[:] = px.reshape(-1)
    img.filepath_raw = str(path); img.file_format = 'PNG'; img.save()
    bpy.data.images.remove(img)


# ---------------- main ----------------
def main():
    a = parse_args()
    random.seed(a.seed); np.random.seed(a.seed)
    hdris, floors, models = discover()
    print(f"[assets] {len(hdris)} hdris, {len(floors)} floors, {len(models)} models")
    img_dir = OUT / "images"; lbl_dir = OUT / "labels"; meta_dir = OUT / "meta"
    for d in (img_dir, lbl_dir, meta_dir): d.mkdir(parents=True, exist_ok=True)

    for i in range(a.start, a.n):
        try:
            abstract = random.random() < ABSTRACT_PROB
            s = reset_scene()
            setup_world(s, hdris, abstract)
            floor = setup_floor(floors, abstract)
            cam, cam_az = add_camera(s)          # camera first, so furniture avoids its sightline
            if not abstract and models:
                place_furniture(models, random.randint(2, 5), cam_az)
            n_t = random.choices([0, 1, 2, 3, 4, 5], weights=[1, 3, 4, 3, 2, 1])[0]
            targets = [add_target() for _ in range(n_t)]
            distract = [add_distractor() for _ in range(random.randint(0, 4))]
            settle(s, targets + distract, floor)
            s.frame_set(SETTLE_FRAMES)
            dg = bpy.context.evaluated_depsgraph_get()
            lines, meta = [], []
            for t in targets:
                pv = projected_verts(s, cam, t)
                if pv is None:
                    continue
                if max(pv[:, 0].max() - pv[:, 0].min(), pv[:, 1].max() - pv[:, 1].min()) < MIN_BOX:
                    continue                 # too tiny to be a useful label
                vis = visibility(s, cam, t, dg)
                if vis < VIS_KEEP:
                    continue
                rect = np.clip(min_area_rect(pv), 0.0, 1.0)
                coords = " ".join(f"{v:.6f}" for v in rect.reshape(-1))
                lines.append(f"0 {coords}")
                meta.append({"vis": round(vis, 3),
                             "verts": np.clip(pv, 0, 1).round(5).tolist(),
                             "dims_m": [round(d, 4) for d in t.dimensions]})

            stem = f"room_{i:05d}"
            s.render.filepath = str(img_dir / f"{stem}.png")
            bpy.ops.render.render(write_still=True)
            sensor_realism(img_dir / f"{stem}.png")
            (lbl_dir / f"{stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
            (meta_dir / f"{stem}.json").write_text(json.dumps(meta))
            print(f"[gen] {stem}: {len(lines)}/{n_t} target(s) kept  (abstract={abstract})")
        except Exception as e:
            import traceback; traceback.print_exc()
            print(f"[gen] image {i} FAILED: {e}")


if __name__ == "__main__":
    main()
