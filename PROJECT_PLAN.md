# Cuboid Detection & Grasping — Project Plan

> AI-based perception + grasping for coloured cuboids on the Yahboom ROSMASTER M3 Pro.
> Goal: detect **graspable-sized** cuboids (any colours, including different colours per face), name their colour, and pick them up top-down.

---

## 1. Goal

Build a robot that can:
1. **Detect** a cuboid in view — including cuboids it was never trained on.
2. **Name its colour.**
3. **Drive to it** and **pick it up.**

The headline requirement: **clients will bring their own cuboids of arbitrary colours (including different colours on different faces) and sizes** to test whether the system generalises to *any* cuboid, not just the training objects. Generalisation is the #1 success criterion.

> **Refined target definition (2026-07-01):** a *target* is a **graspable-sized cuboid** — one whose **narrowest dimension is ≤ ~5.5 cm** (the claw opens 6 cm and grips the short axis). Colour is irrelevant (faces may differ). Anything too big to grip is **not** a target. This makes **size** — not colour — the discriminator (see §5).

---

## 2. Hardware

| Item | Detail |
|---|---|
| Robot | Yahboom **ROSMASTER M3 Pro** |
| Compute | **Jetson Orin NX SUPER** |
| OS / framework | **ROS2 Humble** |
| Arm | **6-DOF**, with **MoveIt** motion planning |
| Camera | **Orbbec Dabai DCW2** binocular structured-light depth camera, **eye-in-hand**, mounted **4 cm above the gripper's centre of rotation** |
| Base | **Mecanum wheels** (omnidirectional) |
| Other sensors | 2× ToF LiDAR (360° navigation) |
| Dev machine | MacBook (Apple Silicon, arm64), Homebrew installed, **Docker not yet installed** |

> **Training machines are always Apple Silicon.** Whenever training is offloaded to another/faster machine, that machine is a **MacBook Pro (Apple Silicon → use `device="mps"`, not CUDA)**. Any exported training bundle/script must target MPS, not NVIDIA.

---

## 3. Key Constraints (these shaped every decision)

1. **Must generalise to unseen cubes** (client test) → cannot train per-object or per-colour classes.
2. **Top-down grasp only** — the gripper motor position allows only vertical (from-above) grabs.
3. **No AprilTags in the product.** The dev objects have tags, but the deployed system must not depend on them. (This also rules out tag-assisted auto-labelling.)
4. **Depth camera is weak on small objects.** The DCW2 is spec'd for 0.2–5 m room/people scanning; on a 3 cm cube up close it produces edge-bleed artifacts (a cube reads as a rounded blob). Trust it only for coarse/navigation and for close-range height, not precise small-object pose.
5. **Limited real objects** — only 13 in hand (≈5 distinct box shapes). Far too little real-world variety to generalise from directly.

---

## 4. Available Objects (dev set)

- **4 cylinders** — Y / B / G / R, Ø3 cm × 3 cm. *(no tags)*
- **4 cuboids** — Y / B / G / R, 6×3×3 cm, AprilTag on one 3×3 face.
- **1 big cube** — 4×4×4 cm, different colour per side, AprilTag on one 4×4 face.
- **4 small cubes** — 3×3×3 cm, pictures on white-background faces, AprilTag on one face.

> Note: tags are useful **only** as a dev-time reference if needed; they are **out of scope for the product** and not relied upon anywhere in the pipeline.

---

## 5. Perception Approach

### What counts as a target — size, not colour

Because client cuboids can be *any* colour (even different per face), colour cannot distinguish a target from clutter. **Size does.** A target is a **graspable-sized cuboid: narrowest dimension ≤ ~5.5 cm.** A tissue box or bin is not a target simply because it's **too big to grab** — which neatly dissolves the earlier "tissue-box" false-positive problem.

Enforced in **two layers**:
1. **Detector (soft, context-based):** trained — via synthetic data — to prefer cuboids that are *small relative to the surrounding scene*. A neural net can learn relative scale from context (ground-texture scale, nearby objects, perspective), biasing toward graspable-looking cuboids and away from large boxes.
2. **Metric size-check at grasp time (hard):** a single RGB image **cannot** measure absolute centimetres (small-close vs big-far are ambiguous). So each detection's *actual* dimensions are measured with the **depth camera + known camera geometry**, and only cuboids with narrowest side ≤ 5.5 cm proceed to grasp. The detector proposes; the depth check confirms.

### Detection — generic shape, not specific objects
- Train **one generic class: `cuboid`** (the model learns "box-ness," which is what generalises to unseen cubes).
- **Do NOT** make colour or object identity a learned class.
- Detector: **YOLO** (Ultralytics), runs in real time on the Orin NX via **TensorRT**.
- Consider **YOLO-OBB (oriented bounding boxes)** so the detector also outputs the cube's **yaw** directly.

### Colour — computed, not learned
- After detection, sample pixels **inside the detected box** and read the dominant hue (**HSV**) → colour name.
- Generalises to any colour, including ones never seen in training. Cuboids may have **different colours on different faces** — colour naming reports the dominant / top-face colour (or per-face if needed).
- **White cubes:** colour is only *named* by HSV (white reads correctly as white because we sample inside the box, not the table). White cubes are *found* by the learned detector + depth-height, never by colour thresholding.

### Rejecting non-targets — size is the discriminator
A generic cuboid detector risks firing on any box-shaped object (the baseline did exactly this on a black bin). Since targets are defined by **graspable size**, non-targets are **large boxes** and **non-cuboid shapes**. Mitigation:
- **Hard negatives, left unlabelled:** large boxes (tissue boxes, bins, packaging, cartons), medium boxes near the size boundary, and non-cuboids (cylinders — we own 4 — spheres, cones).
- Rendered/captured **in-scene** so the model learns "small-in-context cuboid = target; big box = not."
- Backed by the **metric depth size-check** (above) for the hard ≤5.5 cm guarantee.
- This is the single most important part of the dataset.

---

## 6. Grasping Pipeline

Because the grasp is **top-down only** and cubes **rest flat on a surface**, full 6-DoF pose is unnecessary. Gravity fixes pitch/roll. We only need:

| Quantity | How it's obtained |
|---|---|
| **x, y** | Eye-in-hand camera overhead → top-face centroid (ground-plane raycast and/or close-range depth) |
| **yaw** | Top-face rectangle angle (from OBB detector or fitted rectangle) |
| **z** | Table height + cube height (close-range depth, where the cube fills the frame) |
| pitch / roll | Fixed (gripper points down, cube rests flat) |

### Behaviour (coarse-to-fine)
1. **Detect** cuboid at distance (learned model + depth-height).
2. **Approach** — drive the mecanum base toward it (visual servoing: keep box centred, drive until it grows to target size; coarse depth optional).
3. **Align** — position the arm so the eye-in-hand camera looks **straight down** at the top face (this is the "see only 1–2 faces" end state).
4. **Compute** x, y (centroid), yaw (rectangle angle), z (depth/height). Apply the **4 cm hand-eye offset** (calibrate once).
5. **Size-check** — measure the cuboid's real dimensions (depth + geometry); if narrowest side > 5.5 cm, skip it (cannot grasp).
6. **Grasp** — rotate gripper to align with the cube's **narrowest** dimension, descend, close, lift.

### Mechanical limit & size gate
- Claw max opening ≈ **6 cm** → a cuboid is graspable only if its **narrowest dimension ≤ ~5.5 cm** (grip the short axis).
- Top-down grip spans two opposite faces → a cuboid too wide on all axes cannot be picked.
- The **metric depth size-check** (step 5) enforces this: measure real dimensions and skip anything with narrowest side > 5.5 cm.

### Sensor roles (final)
- **RGB (eye-in-hand):** detection, colour, top-face geometry.
- **Depth (DCW2):** coarse approach + close-range height/z; navigation.
- **LiDAR:** navigation / obstacle avoidance.

---

## 7. Data Strategy (the real engineering effort)

13 objects can't teach "any cuboid." We **manufacture diversity** instead of collecting it. Cuboids are the easiest possible object to synthesise, which makes this tractable.

### Sources
| Source | Role | Volume |
|---|---|---|
| **Synthetic rendered cuboids** (domain-randomised: random size, aspect, colour, texture, lighting, angle, background) with auto-generated labels | Generalisation backbone | ~10–50k |
| Real photos of the 13 objects | Sim-to-real bridge | hundreds |
| Any scrounged boxes (cereal boxes, blocks, toys) | Extra real variety | as many as possible |
| Clutter / non-cuboids, unlabelled | Hard negatives | lots |

### Techniques
- **Synthetic generation:** Blender + BlenderProc (free, laptop) or NVIDIA Isaac Sim / Omniverse Replicator (native to the NVIDIA/Jetson stack).
- **Heavy hue randomisation:** teaches colour-invariance so the model keys on shape (colour handled separately by HSV).
- **Copy-paste augmentation:** segment real objects, paste at random scale/rotation onto varied backgrounds → background variety + hard negatives.
- **Transfer learning:** fine-tune a **COCO-pretrained YOLO** to need less data.

### Validation
- **Hold out real cubes the model never trained on** and measure detection on them — this mirrors the client test. Measure it ourselves before clients do.

### Synthetic randomisation design (what we vary & why)

Guiding principle: **randomise everything *except* cuboid shape**, and bake in fixes for the baseline's two weaknesses (false-positives on boxy clutter, misses under occlusion).

**Target cuboids (labelled):**
- Small footprint **relative to the scene** (narrowest side small vs surroundings); vary near the 5.5 cm boundary down to ~2 cm.
- **Any colours, including a different colour per face**; occasional pictured faces (covers the picture-cubes).
- Full range of proportions (cube → elongated); tiny random **edge bevel** (avoids razor-sharp CG edges — a sim-to-real "tell"); random finish (matte → glossy).

**Distractors (unlabelled — the key negative signal):**
- **Large boxes** (tissue boxes, bins, packaging, cartons) — same shape, too big → teaches "big box ≠ target."
- **Medium boxes near the size boundary** — sharpens the small/large line.
- **Non-cuboid shapes** — cylinders, spheres, cones.

**Scale anchors (so "small relative to scene" means something):**
- Realistic ground-texture scale + occasional everyday objects (cup, hand, furniture) for scale reference. Cubes floating on random backgrounds give weak scale cues; grounded scenes give strong ones.

**Other axes:**
- **Scene/ground:** random surfaces incl. **white** (white-on-white case), random backgrounds / HDRIs.
- **Lighting:** random number/direction/intensity/colour-temperature; hard vs soft shadows (shadows are the key cue for white-on-white edges).
- **Camera:** bias to the robot's **eye-in-hand oblique→top-down** views; random distance; **FOV matched to the DCW2**; small roll + slight lens distortion.
- **Count & occlusion:** 0–several cubes (0 = pure background); spread / touching / **overlapping** / partly off-frame.
- **Sensor realism:** random noise, motion blur, mild defocus, exposure shifts, JPEG artifacts, at the robot camera's resolution.

**Labels:** auto-generated from known geometry; only **targets** labelled, never distractors. Because sim knows exact rotation & size, **OBB labels and true metric size are available for free** whenever we want them (start with plain upright boxes to match the baseline).

---

## 7b. Sim-to-real overhaul — realistic-room synthetic (decided 2026-07-01)

### The finding that triggered this
The first synthetic batch (`synth/generate.py` v1: primitive cubes on a **flat solid-colour plane**, solid-colour world background, single sun, razor-sharp edges, no sensor noise) **hurt** held-out-real performance instead of helping. Measured on the same 80 held-out real frames:

| Train set | best mAP50 | mAP50-95 | P | R |
|---|---|---|---|---|
| 245 real only (`runs/cuboid_v1`) | **0.566** | 0.356 | 0.662 | 0.648 |
| 245 real + 2000 synth (`runs/cuboid_v2_mix`) | 0.542 | 0.346 | 0.627 | 0.608 |

Adding 2000 synthetic images made every metric slightly **worse** → the current renders are too "CG" and the domain gap is real.

### Two root causes
1. **Renders look synthetic.** The geometry/size/distractor logic is good, but the **environment is bare**: solid-colour background (no HDRI), untextured solid-colour ground plane (no scale cues), single-sun lighting, razor-sharp cube edges (a known sim-to-real tell), zero sensor realism.
2. **Training procedure lets the CG look win.** Joint training at **2000 synth : 245 real (8:1)** means the loss is dominated by synthetic, pulling weights toward the CG distribution.

### Fixes (ROI order)
1. **Two-stage training (free, no re-render):** pretrain on synthetic → **fine-tune on real only**. Re-anchors weights on real pixels while keeping synthetic shape priors; usually flips synthetic from "hurts" to "helps."
2. **Realistic-room synthetic (the main new approach, below).**
3. **Sensor-realism post-render pass:** downscale to DCW2 resolution, mild blur, gaussian noise, JPEG recompress, exposure/white-balance jitter.
4. **Copy-paste augmentation** (possibly highest ROI): segment the 13 real cubes, paste onto varied real backgrounds — real object pixels = zero object-domain gap.

> **Note:** old abstract/HDRI synthetic frames are **kept**, not replaced — the realistic-room set is **additive**. Aim to keep ~20–30% of renders in the abstract style so the detector doesn't become brittle to non-interior scenes.

### Realistic-room synthetic — the approach
Download **free CC0 "room" 3D scenes**, place graspable cubes inside, and heavily randomise. This is **structured domain randomisation** — it should look very close to the real frames the robot captures.

- **Camera regime = the SEARCH/FIND task, not grasp.** The eye-in-hand camera on the 6-DOF arm scans the room from arm-reachable poses; the cube is a **small object somewhere in the frame** at varied distance/angle/partial occlusion. (The close-range top-down view is a *later* pipeline stage, not this dataset.) Small-in-scene also teaches the "small relative to surroundings" size cue.
- **Placement:** let **Blender physics settle** each cube — it drops, tumbles to a natural resting face, and comes to rest on whatever surface is beneath it (no floating / intersecting furniture).
- **Randomise per render:** which room, furniture colours, wall colours, lighting (number/direction/intensity/colour temp), camera pose, cube count/size/colour/placement. Keep some abstract-background renders in the mix.
- **Diversity beats fidelity:** aim for **15–30+ distinct scenes** (a handful rendered many times → the model memorises backgrounds).
- **Speed:** use **EEVEE Next** (Blender 4.2+), load each room once and render many cubes from it, cap resolution, prefer foreground-GPU. Cycles on the MacBook CPU would take days (Metal GPU crashes headless).

**Sourcing decision (2026-07-01):** This is a **proof-of-concept, not a commercial product**, so asset licensing is *not* a constraint (personal-use/attribution assets are fine). The real blocker for pre-built room `.blend`s is **download friction** — Free3D / CGTrader / BlenderKit / Sketchfab gate downloads behind accounts with no bulk API, so they can't be auto-fetched.
- **Primary engine = procedural composition from Poly Haven** (the only public, no-auth, scriptable API — all CC0). Given the **low 10–50 cm camera** (floor + low furniture + clutter fill the frame, walls barely visible), composing beats hunting for full rooms: exact camera/placement control + thousands of unique scenes (no 20-room memorization).
  - Catalogue confirmed: **491 models** (163 props / 81 furniture / 33 seating / 25 tables / 16 shelves / 20 appliances / 12 office…), **296 indoor HDRIs**, **257 floor textures**.
  - Composition = random indoor **HDRI** (lighting + background) + random **PBR floor** + several **furniture/prop** models (the legs/bodies seen at low height) + floor clutter + physics-settled cubes.
  - Asset fetch via `synth/fetch_assets.py` (Poly Haven file API → `synth/assets/{hdris,floors,models}`); glTF @ 1–2k for models.
- **Optional "hero" rooms:** the generator will also load any full room `.blend`/`.glb` dropped into a folder, if the user hand-downloads a few for extra coherence.

### Occlusion & label convention (decided 2026-07-01)
Room-search views mean cubes are constantly **partially behind furniture / half off-frame**. Decisions:

- **Amodal boxes** — box the cube's **full projected extent** (drawn *through* occluders), clipped to frame. Matches how humans/CVAT label. **The real 245-frame CVAT labels must also be amodal** or the conventions conflict *(open: confirm how they were drawn)*.
- **Visibility measured via dense ray-sampling** — cast camera rays to a grid of points spread over the cube's **front-facing** faces; a sample is *visible* if it projects in-frame **and** its ray hits this cube first (not an occluder). No render pass needed — robust on Blender 5.1, which removed the compositor `scene.node_tree` an index pass would have required.
- **Visibility fraction = visible front-facing samples ÷ total front-facing samples.** Back faces are excluded (self-occlusion is expected), off-frame samples count as not-visible, and occluder-blocked samples count as not-visible — so occlusion, self-occlusion, and truncation are all handled by one number.
- **Keep if ≥ 30% visible, else drop** the label. This same ratio also handles **truncation** (off-frame) for free — a cube 75% out of frame reads ~25% visible and is dropped.
- **Rationale for drop-vs-ignore:** YOLO has no native "ignore" region; a near-invisible cube left unlabelled becomes a *negative* (hurts recall), so the 30% line is set low enough that only near-invisible cubes are dropped.

### Label format decision
- **Target label = YOLO-OBB (oriented box):** gives grasp-time **centre + yaw** directly, with far less symmetry ambiguity than full vertices.
- **Full vertex/edge keypoints were considered and rejected** — auto-labelling them in sim is free, but a cube's **symmetry causes corner-identity ambiguity**, self-occlusion is noisy, plain faces give low-contrast corners, and hand-labelling 8 ordered corners on real frames is impractical. The grasp pipeline consumes centre + yaw + depth-z, not full 6-DoF, so vertices are over-engineering.
- **Free hedge:** the generator will **dump the 8 projected vertices + per-corner visibility to a sidecar file** alongside each render, so corner-regression can be revisited later *without re-rendering*.

---

## 8. Labelling

- Manual labelling (tag-assisted auto-labelling is out, since tags aren't in the product).
- Tool: **CVAT**, run locally via Docker (chosen for its video frame **interpolation** — box once on two frames, auto-fill between — ideal for video captured off the moving robot).
  - Alternative: **Label Studio** (pip install, no Docker, lighter, but weaker video interpolation).
- Draw **oriented boxes** if using YOLO-OBB, so yaw is captured.
- Export in **YOLO format**.
- *Note:* synthetic data needs **no** manual labelling (labels are generated automatically) — labelling effort applies only to real photos.

---

## 9. Software Stack

- **ROS2 Humble** (already on the robot).
- **YOLO (Ultralytics)** for detection; **TensorRT** for Jetson inference.
- **MoveIt** for arm planning (already integrated).
- **Blender/BlenderProc** or **Isaac Sim/Replicator** for synthetic data.
- **OpenCV** for HSV colour naming, ground-plane raycast, classical baselines.
- **CVAT** (Docker) for labelling real images.

---

## Model Evaluation — On-Robot Results (2026-07-03)

**Three models run live on the robot (DCW2 camera). These REAL results trump the offline metrics.**

| Model | On-robot behaviour |
|---|---|
| **`best`** (baseline `cuboid_v1`, real-only, original labels) | **Most reliable** — no jitter, high & consistent confidence. **Weakness: struggles on black/dark surfaces.** |
| **`two_stage_rooms`** (synth-room pretrain → real fine-tune) | **Detects cubes better (higher recall)**, incl. on dark surfaces — **but jitters a lot** and is less confident. |
| **`v3_clean`** (real-only, pseudo-cleaned labels) | **Broken — barely detects anything, low confidence. Something is wrong.** Do not deploy. |

### Key takeaways
- **Offline metrics were misleading.** `v3_clean` scored *highest* on the pseudo-labelled val yet is *worst* on the robot — proving the **circular-benchmark** warning. Trust **on-device / human-verified** evaluation, not pseudo-labelled mAP.
- **No single winner.** Baseline = stability + confidence; two-stage = recall + dark-surface robustness. The ideal model **combines both**.
- **`v3_clean` regressed badly** — likely the pseudo-label self-training added label noise and/or overfit the iPhone/pseudo distribution. Net-negative approach; debug before any reuse.

### Implications / next actions
- **Dark-surface data** = baseline's main real failure → add real + synthetic dark/black-surface scenes (two-stage's synthetic rooms already helped here → make more).
- **Jitter (two-stage)** → reduce via more real fine-tuning, **temporal smoothing / a tracker** on the robot (e.g. ByteTrack), and confidence-threshold tuning.
- **Deploy now:** `best` for stability, or `two_stage_rooms` if recall matters more (ideally + temporal smoothing). **Not** `v3_clean`.
- Human-verified val set still needed for a *fair* ranking, but on-robot behaviour is the current source of truth.

---

## Investor Demo — Milestone Plan (2026-07-03)

**Goal:** a filmable, end-to-end vertical slice that *actually works* in ideal conditions — to satisfy investors now, then harden each part one at a time. Detection is done (`best` works on-robot); **the demo effort points at the GRASP, not the detector** — a physical pick-up is the "wow," boxes on a screen are not.

- **Model:** `best` (baseline `cuboid_v1`) — most reliable on-robot (stable, confident).
- **Scope (smallest impressive slice):** cube on the floor **within arm reach** → detect → compute grasp pose → **top-down pick up** → drop in a bin. **Static base.**
  - **Deferred to a later video:** autonomous driving/approach (navigation + visual servoing) — separate risk surface.
- **Ideal conditions to engineer** (play to `best`, dodge its dark-surface weakness): light, matte, uniform floor (add a light mat if the floor is dark/reflective); even bright lighting; **one** cube; **no dark clutter** (keep the black bin away).
- **De-risking:**
  - **No depth camera for the grasp** (DCW2 is unreliable up close). Use **ground-plane raycast**: project the detection's floor-contact pixel onto the known floor plane → (x, y); z from **known cube height**. → `robot/grasp_pose.py`.
  - **Reuse Yahboom's grasp/sort demo** code; swap in `best.pt` + our pose math. Do **not** rebuild MoveIt from scratch.
  - **Pre-calibrate once** for the fixed geometry: hand-eye (4 cm cam→gripper, lives in TF) + camera→floor transform.
  - **Yaw:** cube grasp is rotationally forgiving → fixed gripper yaw for v1 (refine later via top-face fit).
- **Filming:** multiple takes, show a **genuinely reproducible** run (~3–5 in a row so a live demo won't embarrass); keep an internal note of what is **demo-tuned vs robust** to avoid over-promising.
- **After the video:** refine one part at a time — dark-surface data, two-stage + tracker (jitter), driving/approach, real metric depth/pose, human-verified val.

---

## 10. Open Items / To Decide

- [x] Gripper max opening ≈ 6 cm → graspable if narrowest dimension ≤ ~5.5 cm.
- [x] **Label format = YOLO-OBB** (centre + yaw); full vertex/edge keypoints rejected (see §7b).
- [x] **Occlusion convention = amodal boxes, 30% visibility cutoff, ray-sampled measurement** (see §7b).
- [x] **Synthetic engine = Blender/EEVEE Next** for now (Isaac Sim deferred).
- [x] **Deployment env = any flat surface; arm reach = 0.30 m sphere @ 0.20 m** → camera 0.10–0.50 m off ground (low search view).
- [x] **Camera = 16:9, 86° H × 55° V** (DCW2 default; real frames are 1920×1080 16:9).
- [x] **Assets = Poly Haven CC0**, auto-fetched (`synth/fetch_assets.py`): 30 HDRIs, 30 floors, ~110 models.
- [x] **Realistic-room generator built & validated** (`synth/generate_rooms.py`): HDRI + PBR floor + furniture, physics-settle, ray-sampled amodal visibility (30% gate), OBB labels + vertex sidecar. Smoke-tested: renders ~1.5 s/img on EEVEE, OBB labels wrap cubes correctly (`synth/rooms_output/preview/`).
  - *Note:* occlusion uses **dense ray-sampling** (an index/segmentation pass was ruled out — Blender 5.1 removed `scene.node_tree`); also confirmed **Cycles Metal crashes headless but EEVEE renders fine**.
- [ ] **Confirm the real 245-frame CVAT labels are amodal** (must match synthetic; see §7b).
- [ ] **Optional:** hand-download a few full room `.blend`s for extra coherence (generator can load them).
- [ ] **Investigate why `v3_clean` regressed on-robot** (pseudo-label self-training suspected). Do not reuse until understood.
- [ ] **Add dark/black-surface training data** — baseline's main real-world failure mode.
- [ ] **Add temporal smoothing / tracker on the robot** to fix two-stage's jitter.
- [ ] Deployable models copied to `models/` (`cuboid_v3_clean.pt`, `cuboid_twostage_rooms.pt`) with a model card for the robot session.
- [ ] **Run full generation** (~2000 room images) and wire `synth/rooms_output` into `mix_data.yaml`.
- [ ] Set up **two-stage training** (synth-pretrain → real-only fine-tune) as the default recipe.
- [ ] Tune: widen camera-distance/cube-spread range for more small-in-scene search variety.
- [ ] Implement the metric depth size-check (reject cuboids with narrowest side > 5.5 cm).
- [ ] Measure/calibrate the eye-in-hand hand-eye transform (camera is 4 cm above gripper axis).
- [ ] Calibrate the camera→table-plane transform (or read from ROS TF).
- [ ] Inspect Yahboom's existing M3 Pro vision/sorting demos to see what we plug into.

---

## 11. Suggested Next Steps

> **Revised order (2026-07-01) — after the v1 synthetic batch was found to *hurt* real performance (see §7b):**
> 1. **Two-stage training experiment** (free): pretrain on existing synth → fine-tune on real only. Isolates procedure vs. realism.
> 2. **Rewrite `generate.py` → realistic-room generator** (structured domain randomisation): CC0 rooms, physics-settle placement, ray-sampled amodal occlusion (30% cutoff), OBB labels, EEVEE.
> 3. **Re-mix and re-train**; validate on the same held-out real frames — realistic-room set must beat real-only, not just match it.
> 4. **Sensor-realism pass + copy-paste augmentation** as further gap-closers.
>
> **Blockers for step 2:** confirm real labels are amodal · source CC0 rooms · specify deployment environment.
>
> **Prior order (chosen 2026-06-30, partly superseded):**
> 1. Test `best.pt` on the Jetson Orin NX (on-device baseline).
> 2. Build the Blender synthetic-data generator.
> 3. Wire up `best.pt` as a pre-labeller for the next footage batch.
>
> (Baseline trained 2026-06-30: YOLO11n, mAP50≈0.57, P/R≈0.55. Known weakness: false-positives on box-shaped clutter — needs hard negatives + synthetic diversity.)


1. **Stand up a Blender synthetic-cuboid generator** — a script that outputs randomised cuboid images + YOLO labels (no manual labelling). This is the generalisation engine.
2. In parallel, **capture real photos/video** of the 13 objects + scrounged boxes + clutter from the robot's own camera.
3. **Set up CVAT** to label the real images (oriented boxes).
4. **Fine-tune COCO-pretrained YOLO** on synthetic + real; **validate on held-out real cubes.**
5. Build the **top-down grasp pipeline** (raycast position + yaw + z) as ROS2 nodes and test with MoveIt.
6. Iterate: add hard negatives and more synthetic variety where the model fails.

---

*Core insight: the robotics (top-down grasp, table-plane localisation) is largely solved on paper. The project's real center of gravity is a **synthetic-data pipeline for a generic cuboid detector that generalises to unseen cubes and rejects clutter.***
