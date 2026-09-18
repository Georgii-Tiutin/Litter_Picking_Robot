#!/usr/bin/env python3
"""Synthetic cuboid generator — v1 (core randomization, procedural).

Run headless:
  blender --background --python synth/generate.py

Per image: random ground/lighting/camera/background; K graspable-size TARGET
cuboids (labelled) + big-box / cylinder / sphere / cone DISTRACTORS (unlabelled).
YOLO boxes are projected from geometry; targets fully occluded (center hidden)
are dropped via a camera ray-cast visibility check. Class 0 = cuboid.
"""
import bpy, math, random, colorsys
from pathlib import Path
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view

# ---------------- config ----------------
OUT = Path("/Users/georgiitiutin/ProjectsRoot/Making Data/synth/output")
N = 2000
RES = 640
SAMPLES = 16   # low + denoiser (see reset_scene) -> clean & fast
SEED = 7

# target = graspable cuboid: narrowest edge <= 5.5 (scene units ~ cm)
TARGET_MIN_EDGE = (1.5, 5.5)     # narrowest edge range
TARGET_MAX_EDGE = (1.5, 9.0)     # other edges range
DISTRACTOR_BOX_EDGE = (9.0, 16.0)  # all edges big -> ungraspable, also scale anchors

random.seed(SEED)
IMG_DIR = OUT / "images"; LBL_DIR = OUT / "labels"
IMG_DIR.mkdir(parents=True, exist_ok=True)
LBL_DIR.mkdir(parents=True, exist_ok=True)


def rand_color():
    r = random.random()
    if r < 0.7:      # saturated
        return colorsys.hsv_to_rgb(random.random(), random.uniform(0.5, 1), random.uniform(0.5, 1))
    if r < 0.85:     # pastel
        return colorsys.hsv_to_rgb(random.random(), random.uniform(0.1, 0.4), random.uniform(0.6, 1))
    g = random.uniform(0.0, 1.0); return (g, g, g)   # greyscale incl white/black


def ground_color():
    r = random.random()
    if r < 0.4:  g = random.uniform(0.6, 1.0); return (g, g, g)   # light / white
    if r < 0.7:  g = random.uniform(0.1, 0.5); return (g, g, g)   # dark grey
    return rand_color()


def make_mat(color):
    m = bpy.data.materials.new("m"); m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = random.uniform(0.2, 0.9)
    return m


def color_object(obj, per_face=False):
    obj.data.materials.clear()
    colors = [rand_color() for _ in range(6)] if per_face else [rand_color()]
    for c in colors:
        obj.data.materials.append(make_mat(c))
    if per_face:
        for poly in obj.data.polygons:
            poly.material_index = random.randrange(len(colors))


def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    s = bpy.context.scene
    s.render.resolution_x = RES; s.render.resolution_y = RES
    s.render.image_settings.file_format = 'PNG'
    s.render.engine = 'CYCLES'; s.cycles.samples = SAMPLES
    s.cycles.use_denoising = True   # keep low-sample renders clean
    # NOTE: Cycles Metal GPU crashes in headless (--background) mode on macOS
    # (MetalKernelPipeline::compile -> nil NSURL abort). CPU is reliable headless.
    s.cycles.device = 'CPU'
    # world / ambient
    w = bpy.data.worlds.new("W"); s.world = w; w.use_nodes = True
    bg = w.node_tree.nodes["Background"]
    wc = colorsys.hsv_to_rgb(random.random(), random.uniform(0.0, 0.5), random.uniform(0.4, 1.0))
    bg.inputs[0].default_value = (*wc, 1)
    bg.inputs[1].default_value = random.uniform(0.4, 1.3)   # ambient floor -> no black frames
    return s


def add_ground():
    bpy.ops.mesh.primitive_plane_add(size=800, location=(0, 0, 0))
    g = bpy.context.active_object
    m = bpy.data.materials.new("ground"); m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*ground_color(), 1)
    b.inputs["Roughness"].default_value = random.uniform(0.5, 1.0)
    g.data.materials.append(m)


def add_lights():
    ld = bpy.data.lights.new("Sun", 'SUN')
    ld.energy = random.uniform(3.0, 7.0)
    ld.color = colorsys.hsv_to_rgb(random.uniform(0.05, 0.15), random.uniform(0, 0.2), 1)  # warm-ish
    ld.angle = random.uniform(0.01, 0.3)  # soft vs hard shadows
    lo = bpy.data.objects.new("Sun", ld); bpy.context.scene.collection.objects.link(lo)
    lo.rotation_euler = (math.radians(random.uniform(15, 70)), 0, math.radians(random.uniform(0, 360)))


def add_camera(s):
    cd = bpy.data.cameras.new("Cam"); co = bpy.data.objects.new("Cam", cd)
    s.collection.objects.link(co); s.camera = co
    cd.angle = math.radians(random.uniform(40, 65))          # FOV
    dist = random.uniform(22, 45)
    elev = math.radians(random.uniform(20, 88))              # oblique -> near top-down
    az = random.uniform(0, 2 * math.pi)
    co.location = (dist * math.cos(elev) * math.cos(az),
                   dist * math.cos(elev) * math.sin(az),
                   dist * math.sin(elev))
    d = Vector((0, 0, 0)) - co.location
    co.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
    co.rotation_euler.rotate_axis('Z', math.radians(random.uniform(-8, 8)))  # small roll
    return co


def place(dims_z, radius):
    return (random.uniform(-radius, radius), random.uniform(-radius, radius), dims_z / 2)


def add_target(radius=6):
    lo = random.uniform(*TARGET_MIN_EDGE)
    dims = [lo, random.uniform(lo, TARGET_MAX_EDGE[1]), random.uniform(lo, TARGET_MAX_EDGE[1])]
    random.shuffle(dims)
    bpy.ops.mesh.primitive_cube_add(size=1, location=place(dims[2], radius))
    o = bpy.context.active_object
    o.scale = dims
    o.rotation_euler = (0, 0, random.uniform(0, math.pi))
    color_object(o, per_face=(random.random() < 0.4))
    return o


def add_distractor(radius=6):
    kind = random.choice(['box', 'box', 'cylinder', 'sphere', 'cone'])
    if kind == 'box':
        dims = [random.uniform(*DISTRACTOR_BOX_EDGE) for _ in range(3)]
        bpy.ops.mesh.primitive_cube_add(size=1, location=place(dims[2], radius))
        o = bpy.context.active_object; o.scale = dims
        o.rotation_euler = (0, 0, random.uniform(0, math.pi))
    elif kind == 'cylinder':
        r = random.uniform(1.5, 6); h = random.uniform(3, 15)
        bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=h, location=place(h, radius))
        o = bpy.context.active_object
    elif kind == 'sphere':
        r = random.uniform(2, 7)
        bpy.ops.mesh.primitive_uv_sphere_add(radius=r, location=place(2 * r, radius))
        o = bpy.context.active_object
    else:
        r = random.uniform(2, 6); h = random.uniform(4, 14)
        bpy.ops.mesh.primitive_cone_add(radius1=r, depth=h, location=place(h, radius))
        o = bpy.context.active_object
    color_object(o, per_face=False)
    return o


def visible(s, cam, obj, depsgraph):
    """Ray-cast from camera to object centre; visible if the target is hit first."""
    center = obj.matrix_world @ (0.125 * sum((Vector(c) for c in obj.bound_box), Vector()))
    origin = cam.matrix_world.translation
    direction = (center - origin).normalized()
    hit, _, _, _, hit_obj, _ = s.ray_cast(depsgraph, origin, direction)
    return hit and hit_obj is not None and hit_obj.name == obj.name


def yolo_bbox(s, cam, obj):
    # centre must be in front of the camera (z>0) and inside the frame
    center = obj.matrix_world @ (0.125 * sum((Vector(c) for c in obj.bound_box), Vector()))
    cn = world_to_camera_view(s, cam, center)
    if cn.z <= 0 or not (0.0 <= cn.x <= 1.0 and 0.0 <= cn.y <= 1.0):
        return None
    xs, ys = [], []
    for c in obj.bound_box:
        n = world_to_camera_view(s, cam, obj.matrix_world @ Vector(c))
        if n.z <= 0:            # object spans the camera plane -> unreliable projection
            return None
        xs.append(n.x); ys.append(n.y)
    x0, x1 = max(0.0, min(xs)), min(1.0, max(xs))
    y0, y1 = max(0.0, min(ys)), min(1.0, max(ys))
    if x1 - x0 < 0.01 or y1 - y0 < 0.01:
        return None
    return ((x0 + x1) / 2, 1.0 - (y0 + y1) / 2, x1 - x0, y1 - y0)


def main():
    for i in range(N):
        try:
            s = reset_scene()
            add_ground(); add_lights()
            cam = add_camera(s)
            targets = [add_target() for _ in range(random.choices([0, 1, 2, 3, 4, 5],
                                                                   weights=[1, 3, 4, 3, 2, 1])[0])]
            for _ in range(random.randint(0, 3)):
                add_distractor()
            for _ in range(random.randint(0, 3)):
                add_distractor()

            bpy.context.view_layer.update()
            depsgraph = bpy.context.evaluated_depsgraph_get()
            lines = []
            for t in targets:
                if not visible(s, cam, t, depsgraph):
                    continue
                bb = yolo_bbox(s, cam, t)
                if bb:
                    lines.append(f"0 {bb[0]:.6f} {bb[1]:.6f} {bb[2]:.6f} {bb[3]:.6f}")

            stem = f"synth_{i:05d}"
            s.render.filepath = str(IMG_DIR / f"{stem}.png")
            bpy.ops.render.render(write_still=True)
            (LBL_DIR / f"{stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
            print(f"[gen] {stem}: {len(lines)} target(s)")
        except Exception as e:
            print(f"[gen] image {i} FAILED: {e}")


if __name__ == "__main__":
    main()
