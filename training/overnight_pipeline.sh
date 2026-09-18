#!/usr/bin/env bash
# Overnight chain: wait for room generation -> OBB->AABB convert -> two-stage training.
# Wrapped in caffeinate so the Mac stays awake for the whole run.
set -u
cd "/Users/georgiitiutin/ProjectsRoot/Making Data"
LOG="synth/rooms_output/pipeline.log"
VENV="/private/tmp/claude-501/-Users-georgiitiutin-ProjectsRoot-Making-Data/8ae9d5e5-1a80-4e0a-b632-cfb864158f77/scratchpad/train-venv/bin/python"
say(){ echo "[pipeline $(date '+%H:%M:%S')] $*" | tee -a "$LOG"; }

say "waiting for generate_rooms.py to finish..."
while pgrep -f generate_rooms.py >/dev/null 2>&1; do sleep 30; done
N=$(grep -c '\[gen\]' synth/rooms_output/gen.log 2>/dev/null || echo 0)
say "generation done: $N images, $(ls synth/rooms_output/images/*.png 2>/dev/null | wc -l) pngs on disk"

say "converting OBB -> AABB..."
python3 synth/obb_to_aabb.py 2>&1 | tee -a "$LOG"
AABB=$(ls synth/rooms_aabb/labels/*.txt 2>/dev/null | wc -l)
say "aabb labels: $AABB"
if [ "$AABB" -lt 100 ]; then say "ABORT: too few AABB labels"; exit 1; fi

say "starting two-stage training (this is the multi-hour step)..."
"$VENV" train_two_stage.py 2>&1 | tee -a "$LOG"
say "PIPELINE COMPLETE — compare runs/cuboid_v3_finetune vs cuboid_v1 (0.566)"
