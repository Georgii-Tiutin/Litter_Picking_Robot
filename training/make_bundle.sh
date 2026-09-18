#!/usr/bin/env bash
# Assemble the portable two-stage training bundle at DEST (materialises the
# rooms_aabb/images symlink into real files so it works on another PC).
#   ./make_bundle.sh /Volumes/YOUR_DRIVE/cuboid_training_bundle
set -eu
SRC="/Users/georgiitiutin/ProjectsRoot/Making Data"
STAGE="/private/tmp/claude-501/-Users-georgiitiutin-ProjectsRoot-Making-Data/fd2093b8-28a3-4c18-b533-35c5ee484df1/scratchpad/bundle_src"
DEST="${1:?usage: ./make_bundle.sh /Volumes/DRIVE/cuboid_training_bundle}"

echo "[bundle] dest: $DEST"
mkdir -p "$DEST/data/rooms_aabb/images" "$DEST/data/rooms_aabb/labels" "$DEST/data/dataset_clean"

# -L dereferences symlinks -> real files land in the bundle (portable to other PCs)
echo "[bundle] copying 2000 room images (~4 GB, materialising symlink)..."
rsync -aL "$SRC/synth/rooms_output/images/" "$DEST/data/rooms_aabb/images/"
echo "[bundle] copying room labels..."
rsync -aL "$SRC/synth/rooms_aabb/labels/"   "$DEST/data/rooms_aabb/labels/"
echo "[bundle] copying cleaned real dataset (dataset_clean images/ are symlinks -> deref)..."
rsync -aL "$SRC/dataset_clean/"             "$DEST/data/dataset_clean/"

echo "[bundle] copying scripts + weights..."
cp "$SRC/yolo11n.pt"            "$DEST/"
cp "$STAGE/train_two_stage.py"  "$DEST/"
cp "$STAGE/requirements.txt"    "$DEST/"
cp "$STAGE/README.md"           "$DEST/"

echo "[bundle] verifying..."
RI=$(ls "$DEST/data/rooms_aabb/images"/*.png 2>/dev/null | wc -l | tr -d ' ')
RL=$(ls "$DEST/data/rooms_aabb/labels"/*.txt 2>/dev/null | wc -l | tr -d ' ')
TR=$(ls "$DEST/data/dataset_clean/images/train" 2>/dev/null | wc -l | tr -d ' ')
VA=$(ls "$DEST/data/dataset_clean/images/val"   2>/dev/null | wc -l | tr -d ' ')
BROKEN=$(find "$DEST" -type l 2>/dev/null | wc -l | tr -d ' ')
echo "  rooms: $RI imgs / $RL labels | real: $TR train / $VA val | symlinks remaining: $BROKEN"
echo "  size: $(du -sh "$DEST" | cut -f1)"
[ "$RI" = "2000" ] && [ "$BROKEN" = "0" ] && echo "[bundle] OK -> plug into the other PC and follow README.md" || echo "[bundle] WARN: check counts/symlinks above"
