#!/usr/bin/env python3
"""Fetch CC0 assets from Poly Haven for the realistic-room synthetic generator.

Downloads into synth/assets/{hdris,floors,models}:
  - indoor HDRIs        -> lighting + background
  - floor PBR textures  -> the dominant surface at the robot's 10-50cm camera height
  - furniture/prop glTF  -> the low furniture legs/bodies + floor clutter seen in-scene

No auth required (Poly Haven public API, all CC0). Re-running skips existing files.

  python3 synth/fetch_assets.py                 # default curated set
  python3 synth/fetch_assets.py --hdris 40 --floors 40 --models 120
  python3 synth/fetch_assets.py --res 2k        # texture/model resolution (1k default)
"""
import argparse, json, shutil, urllib.request, urllib.error
from pathlib import Path

API = "https://api.polyhaven.com"
UA = {"User-Agent": "cuboid-synth-fetcher/1.0 (research POC)"}  # API 403s the default urllib UA
ROOT = Path(__file__).parent / "assets"
# model categories worth having at a low camera: things with legs / floor-level bulk / clutter
MODEL_CATS = ["furniture", "seating", "table", "shelves", "appliances", "office",
              "props", "containers", "decorative", "potted plants", "electronics"]
TEX_MAPS = ["Diffuse", "nor_gl", "Rough", "Displacement", "arm"]  # what a floor material needs


def get_json(url):
    with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=30) as r:
        return json.load(r)


def download(url, dest: Path):
    if dest.exists() and dest.stat().st_size > 0:
        return "skip"
    dest.parent.mkdir(parents=True, exist_ok=True)
    try:
        with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=120) as r, \
                open(dest, "wb") as f:
            shutil.copyfileobj(r, f)
        return "ok"
    except (urllib.error.URLError, urllib.error.HTTPError, OSError) as e:
        print(f"    ! failed {url}: {e}")
        if dest.exists():
            dest.unlink()
        return "fail"


def pick_res(node, want):
    """node = {'1k': {...}, '2k': {...}}. Return the requested res or closest available."""
    if want in node:
        return want, node[want]
    for r in ("1k", "2k", "4k"):
        if r in node:
            return r, node[r]
    k = next(iter(node)); return k, node[k]


def fetch_hdris(n, res):
    print(f"[hdris] fetching up to {n} indoor HDRIs @ {res}")
    ids = list(get_json(f"{API}/assets?type=hdris&categories=indoor").keys())[:n]
    for i, aid in enumerate(ids):
        files = get_json(f"{API}/files/{aid}")
        _, node = pick_res(files["hdri"], res)
        ext = "hdr" if "hdr" in node else "exr"
        s = download(node[ext]["url"], ROOT / "hdris" / f"{aid}.{ext}")
        print(f"  [{i+1}/{len(ids)}] {aid} {s}")


def fetch_floors(n, res):
    print(f"[floors] fetching up to {n} floor textures @ {res}")
    ids = list(get_json(f"{API}/assets?type=textures&categories=floor").keys())[:n]
    for i, aid in enumerate(ids):
        files = get_json(f"{API}/files/{aid}")
        got = []
        for m in TEX_MAPS:
            if m not in files:
                continue
            _, node = pick_res(files[m], res)
            ext = "jpg" if "jpg" in node else ("png" if "png" in node else next(iter(node)))
            if download(node[ext]["url"], ROOT / "floors" / aid / f"{m}.{ext}") != "fail":
                got.append(m)
        print(f"  [{i+1}/{len(ids)}] {aid}: {'+'.join(got) or 'NONE'}")


def fetch_models(n, res):
    print(f"[models] fetching up to {n} models @ {res} (glTF)")
    seen, ids = set(), []
    for cat in MODEL_CATS:
        for aid in get_json(f"{API}/assets?type=models&categories={cat.replace(' ', '%20')}"):
            if aid not in seen:
                seen.add(aid); ids.append(aid)
    ids = ids[:n]
    for i, aid in enumerate(ids):
        files = get_json(f"{API}/files/{aid}")
        if "gltf" not in files:
            print(f"  [{i+1}/{len(ids)}] {aid}: no glTF, skip"); continue
        _, node = pick_res(files["gltf"], res)   # node = {'gltf': {url, include, ...}}
        main = node["gltf"]
        d = ROOT / "models" / aid
        n_ok = 0
        # main .gltf keeps its original basename so internal URIs to .bin/textures resolve
        if download(main["url"], d / Path(main["url"]).name) != "fail":
            n_ok += 1
        for rel, meta in main.get("include", {}).items():
            if download(meta["url"], d / rel) != "fail":
                n_ok += 1
        print(f"  [{i+1}/{len(ids)}] {aid}: {n_ok} file(s) -> {Path(main['url']).name}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hdris", type=int, default=30)
    ap.add_argument("--floors", type=int, default=30)
    ap.add_argument("--models", type=int, default=100)
    ap.add_argument("--res", default="1k", choices=["1k", "2k", "4k"])
    a = ap.parse_args()
    ROOT.mkdir(parents=True, exist_ok=True)
    if a.hdris:  fetch_hdris(a.hdris, a.res)
    if a.floors: fetch_floors(a.floors, a.res)
    if a.models: fetch_models(a.models, a.res)
    print(f"[done] assets in {ROOT}")


if __name__ == "__main__":
    main()
