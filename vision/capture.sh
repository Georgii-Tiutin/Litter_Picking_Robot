#!/bin/bash
# Capture one settled frame from the C505, addressed BY NAME.
# avfoundation device indices shuffle between calls - never use an index.
# usage: ./capture.sh <output.jpg> [warmup_frames]
OUT="${1:-frame.jpg}"
WARM="${2:-20}"
ffmpeg -hide_banner -loglevel error \
  -f avfoundation -framerate 30 -video_size 1280x720 -pixel_format uyvy422 \
  -i "C505 HD Webcam" -vf "select=gte(n\,${WARM})" -frames:v 1 -y "$OUT" 2>&1 | grep -v "^$"
[ -s "$OUT" ] && echo "captured $OUT ($(stat -f%z "$OUT") bytes)" || echo "CAPTURE FAILED"
