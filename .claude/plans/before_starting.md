# Before Starting — Repo Cleanup + Gitflow Setup

Pre-work that must happen before executing `plan_claude.md`. The plan introduces ~6 new GPU components (NanoOWL, NanoSAM, FoundationPose, cuRobo, cuVSLAM, Nvblox) plus Isaac Sim assets, BehaviorTree nodes, and ROS2 packages. The current repo layout will collapse under that load within a week. Half a day of hygiene now saves a month of regret later.

---

## Part 1 — Is the plan ready to execute?

**Short answer: clean up first, but skip full gitflow.** A limited gitflow is added in Part 2 (for portfolio/admissions purposes).

### What's wrong with the current repo
- **One commit of history** (`a53f0c0 Cube detector v1`) covering ~3 months of work. Bisecting a regression is impossible.
- **8 parallel cube-detector experiments** in `perception/scripts/`:
  `cube_detector.py`, `cube_detector_red.py`, `cube_detector_v1.py`, `blue_mask.py`, `hsv_tuner.py`, `hsv_tuner_red.py`, `scene_highlighter.py`, `view_debug.py`.
  No canonical entry point.
- **No package structure** — `color_tracking_orin.py` sits at repo root, `perception/` has no `package.xml`, no `pyproject.toml`, no colcon workspace. Phase 1 will need this.
- **`CLAUDE.md` is empty** — every session rediscovers the repo from scratch.
- **No tests, no CI.** Acceptable for solo research, but zero regression safety net before introducing 6 new GPU components.

### Minimal cleanup (~half a day)
1. **Decide the workspace shape**: proper ROS2 / colcon workspace (`src/perception_pkg/`, `src/manipulation_pkg/`, …) or plain Python project. The plan assumes ROS2 nodes throughout, so **colcon is the right call**.
2. **Archive the experiments**: move `cube_detector*.py`, `hsv_tuner*.py`, `blue_mask.py`, `scene_highlighter.py` into `experiments/2026-04_color_detector/` or delete the obsolete ones. Keep one canonical detector as the Phase 0 baseline.
3. **Populate `CLAUDE.md`**: hardware summary, where things live, how to launch, current phase. ~50 lines, pays for itself within one session.
4. **Branch per phase** (expanded in Part 2 below).
5. **Commit per logical change** going forward. "Cube detector v1" as the entire prior history is fine as a starting point; just don't do it again.

### What to skip
- **Heavy CI** — Jetson-targeted code is hard to CI off-device anyway. A `make smoke` target that runs on the Orin is enough.
- **Pre-commit hooks, linters, formatters** — nice-to-have, but not before the plan executes.

---

## Part 2 — Gitflow Lite (for portfolio / admissions visibility)

This project is a gateway into university and needs to demonstrate gitflow competence for future group work. Full gitflow is overkill for a solo repo, but a **portfolio-visible limited gitflow** is genuinely useful when a reviewer checks the GitHub network graph.

### The branch model (keep)
- **`main`** — only tagged, working milestones. Never commit directly.
- **`develop`** — integration branch. All features merge here first.
- **`feature/*`** — one per Phase or major sub-task. Branched from `develop`, merged back via PR.
- **`release/*`** — branched from `develop` when prepping a Phase milestone for `main`. Where version bumps, CHANGELOG updates, and final sim regression happen. Merged to both `main` (tagged) and `develop`.
- **`hotfix/*`** — only if a genuinely broken-on-`main` bug appears. **Don't manufacture them for show** — a reviewer can tell the difference between a real hotfix and a ceremonial one.

### What makes it visible to an admissions reviewer
1. **GitHub network graph** — branch topology must be readable. Use `--no-ff` on every merge so feature branches don't get squashed into oblivion. Gitflow without `--no-ff` is invisible.
2. **Releases / tags page** — each Phase milestone tagged as `v0.1.0`, `v0.2.0`, etc., with release notes. Stronger signal than commit count.
3. **PRs** — even self-merged, write a real description: what changed, why, how tested. The artifact that proves collaboration capability.
4. **Commit messages** — Conventional Commits (`feat(perception): add NanoOWL detector node`). Standard format, reads as professional.

### Concrete setup
```bash
# install git-flow CLI (homebrew)
brew install git-flow-avh

# initialize (accept defaults: main, develop, feature/, release/, hotfix/, v prefix)
git flow init

# tag current state as the baseline so main has a starting point
git tag -a v0.1.0-baseline -m "Cube detector v1 baseline"
git push origin main develop --tags

# start Phase 1
git flow feature start phase-1-perception-nanoowl
# ... do work, commit ...
git flow feature finish phase-1-perception-nanoowl   # merges into develop with --no-ff

# when Phase 1 is integration-tested in sim, cut a release
git flow release start v0.2.0
# bump versions, update CHANGELOG.md
git flow release finish v0.2.0   # tags main, merges back to develop
git push origin main develop --tags
```

### Versioning convention (tags map to phases)
- `v0.1.0` — baseline (current state)
- `v0.2.0` — Phase 1 (perception)
- `v0.3.0` — Phase 2 (pose)
- `v0.4.0` — Phase 3 (grasp)
- `v0.5.0` — Phase 4 (nav)
- `v0.6.0` — Phase 5 (mission)
- `v0.7.0` — Phase 6 (sim2real)
- `v1.0.0` — Phase 7 complete (indoor pickup end-to-end demo)
- `v2.0.0` — Extension complete (outdoor grass demo)

Each minor bump = a release branch + PR + tag. **Eight to ten clean releases by the end of the project is a much stronger portfolio signal than hundreds of micro-commits.**

### What to skip even with gitflow enabled
- **Don't fake hotfixes.** A reviewer reading commit history can tell a fabricated `hotfix/typo-in-readme` from a real one. Use the branch when warranted, leave it empty otherwise.
- **Don't add `release/*` for every feature** — only at Phase boundaries.
- **Don't enforce mandatory PR reviews on yourself** — branch protection requiring a second reviewer will just block you. Require status checks (CI green) instead, once CI exists.

---

## Consolidated checklist (do in order)

1. [ ] **Workspace shape**: convert to colcon ROS2 workspace (`src/<pkg>/` layout)
2. [ ] **Create `.gitignore`**: exclude colcon artifacts (`build/`, `install/`, `log/`), Python junk (`__pycache__/`, `*.pyc`, venvs), model weights (`*.pt`, `*.onnx`, `*.engine`, `*.trt`), datasets, ROS bags (`*.bag`, `*.db3`), and editor/OS cruft. Keep large binaries out of git from the start.
3. [ ] **Archive experiments**: move stale `cube_detector*.py`, `hsv_tuner*.py`, `blue_mask.py`, `scene_highlighter.py` → `experiments/2026-04_color_detector/`. Keep one canonical detector.
4. [ ] **Populate `CLAUDE.md`** (~50 lines: hardware, layout, launch, current phase)
5. [ ] **Create `CHANGELOG.md`** (Keep-a-Changelog format)
6. [ ] **Create `RELEASING.md`** documenting the release procedure
7. [ ] **Install git-flow-avh** (`brew install git-flow-avh`)
8. [ ] **`git flow init`** with default branch names
9. [ ] **Tag baseline**: `v0.1.0-baseline` on `main`, push tags
10. [ ] **Push `develop` branch** to origin
11. [ ] **Open the first feature branch**: `feature/phase-1-perception-nanoowl`

After step 11, `plan_claude.md` Phase 1 can begin.
