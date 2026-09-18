# Data kept locally (not on GitHub)

The code, labels, training runs and models are in this repo. The bulky images, renders and videos below are not: together they are about 6.5 GB, well past what GitHub holds without Git LFS. They live in the local project folder (`Prior_work_before_reboot/`) and can mostly be regenerated.

| Local path | Size | What it is | How to get it back |
|---|---|---|---|
| `Training videos/IMG_0962–0971.mov` | 291 MB | 10 iPhone videos of the cuboids, the source of all real training data | Original recordings; no other copy |
| `Computer_vision/data/frames/` | 110 MB | Frames pulled out of the videos | `extract_frames.sh` (on `yolo-detector`) |
| `Computer_vision/dataset/images/` | 913 MB | Real training/val images (CVAT-labelled) | Frames from the videos; the labels are on `yolo-detector` under `training/dataset/labels/` |
| `dataset_clean/images/` | (symlinked) | Same images, used with the cleaned labels | Labels are on `yolo-detector` under `training/dataset_clean/labels/` |
| `synth/output/` | 797 MB | First synthetic renders (plain backgrounds) | `synth/generate.py` |
| `synth/rooms_output/` | 4.1 GB | Synthetic "room" renders used for pretraining | `synth/generate_rooms.py`; the generation logs are committed |
| `synth/assets/` | 346 MB | 3D models and HDRIs used by the renderer | `synth/fetch_assets.py` |
| `synth/label_review/`, `synth/preview/` | 92 MB | Label-gap review images and render previews | `synth/scan_label_gaps.py`, `synth/preview_labels.py` |
| `yolo11n.pt` | 5 MB | Ultralytics YOLO11n base weights | Downloaded automatically by Ultralytics |

Paths in the training scripts point at the original `Making Data` folder on the Mac, so they need adjusting before they run anywhere else.
