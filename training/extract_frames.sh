#!/usr/bin/env bash
#
# extract_frames.sh — cut every video in data/videos/ into frames.
#
# Default: 2 frames per second (one frame every 0.5 s).
# Frames are written to data/frames/ as <videoname>_<NNNN>.jpg
#
# Usage:
#   ./extract_frames.sh                          # 2 fps, from data/videos
#   ./extract_frames.sh 1                         # 1 fps (every 1.0 s)
#   ./extract_frames.sh 2 "Training videos"       # 2 fps, from a custom folder
#
set -euo pipefail

# Resolve paths relative to this script so it works from anywhere.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRAME_DIR="$SCRIPT_DIR/data/frames"

FPS="${1:-2}"   # frames per second; 2 => every 0.5 s

# Optional 2nd arg = input folder (absolute, or relative to the script dir).
INPUT="${2:-data/videos}"
case "$INPUT" in
  /*) VIDEO_DIR="$INPUT" ;;
  *)  VIDEO_DIR="$SCRIPT_DIR/$INPUT" ;;
esac

mkdir -p "$FRAME_DIR"

shopt -s nullglob nocaseglob
videos=("$VIDEO_DIR"/*.mov "$VIDEO_DIR"/*.mp4 "$VIDEO_DIR"/*.m4v "$VIDEO_DIR"/*.avi)
shopt -u nocaseglob

if [ ${#videos[@]} -eq 0 ]; then
  echo "No videos found in $VIDEO_DIR"
  echo "Drop your iPad videos (.mov/.mp4) there and re-run."
  exit 0
fi

total=0
for video in "${videos[@]}"; do
  base="$(basename "$video")"
  name="${base%.*}"
  # Sanitise spaces in the video name for clean frame filenames.
  safe="${name// /_}"
  echo "==> $base  (fps=$FPS)"
  ffmpeg -hide_banner -loglevel error -i "$video" \
         -vf "fps=$FPS" -q:v 2 \
         "$FRAME_DIR/${safe}_%04d.jpg"
  count=$(find "$FRAME_DIR" -name "${safe}_*.jpg" | wc -l | tr -d ' ')
  echo "    -> $count frames"
  total=$((total + count))
done

echo ""
echo "Done. Total frames in $FRAME_DIR: $(find "$FRAME_DIR" -name '*.jpg' | wc -l | tr -d ' ')"
