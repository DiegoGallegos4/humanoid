#!/usr/bin/env python3
"""Render a recorded SLAM run (ros/record_map.py) to a PNG and an MP4.

  .venv/bin/python simulation/mujoco/render_map.py

Top-down, world-fixed view. Grey = unknown, white = free, black = occupied,
red dots = the current lidar scan, blue = robot path, green = robot + heading.
Outputs docs/media/slam_map.png (final map) and docs/media/slam_build.mp4.
Needs Pillow + ffmpeg on the Mac.
"""
import math
import os
import pickle
import shutil
import subprocess
import tempfile

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "map_log.pkl")
OUT_DIR = os.path.join(HERE, "..", "..", "docs", "media")

VIEW = 3.6          # show +/- VIEW metres around the room centre
PX_PER_M = 110
SIZE = int(2 * VIEW * PX_PER_M)
FPS = 20

UNKNOWN, FREE, OCC = (150, 155, 165), (250, 250, 250), (25, 25, 30)


def to_px(x, y):
    return (x + VIEW) * PX_PER_M, (VIEW - y) * PX_PER_M


def map_image(m):
    """OccupancyGrid snapshot -> RGB canvas in the world-fixed view."""
    canvas = np.empty((SIZE, SIZE, 3), dtype=np.uint8)
    canvas[:] = UNKNOWN
    # world coords of every canvas pixel centre -> grid cell
    xs = (np.arange(SIZE) + 0.5) / PX_PER_M - VIEW
    ys = VIEW - (np.arange(SIZE) + 0.5) / PX_PER_M
    col = np.floor((xs - m["ox"]) / m["res"]).astype(int)
    row = np.floor((ys - m["oy"]) / m["res"]).astype(int)
    cv, rv = np.meshgrid(col, row)
    inside = (cv >= 0) & (cv < m["w"]) & (rv >= 0) & (rv < m["h"])
    vals = np.full(cv.shape, -1, dtype=np.int16)
    vals[inside] = m["data"][rv[inside], cv[inside]]
    canvas[(vals >= 0) & (vals < 65)] = FREE
    canvas[vals >= 65] = OCC
    return canvas


def draw_frame(base, path, pose, scan, t):
    img = Image.fromarray(base)
    d = ImageDraw.Draw(img)
    if len(path) > 1:
        d.line([to_px(px, py) for px, py in path], fill=(40, 110, 230), width=3)
    for sx, sy in scan:
        u, v = to_px(sx, sy)
        d.ellipse([u - 2, v - 2, u + 2, v + 2], fill=(225, 40, 40))
    x, y, th = pose
    u, v = to_px(x, y)
    r = 0.15 * PX_PER_M   # chassis half-width
    d.ellipse([u - r, v - r, u + r, v + r], outline=(20, 150, 60), width=4)
    hu, hv = to_px(x + 0.25 * math.cos(th), y + 0.25 * math.sin(th))
    d.line([(u, v), (hu, hv)], fill=(20, 150, 60), width=4)
    d.rectangle([0, 0, 250, 30], fill=(255, 255, 255))
    d.text((8, 8), f"Nori SLAM  t = {t:5.1f} s", fill=(0, 0, 0))
    return img


def main():
    with open(LOG, "rb") as f:
        log = pickle.load(f)
    frames, maps = log["frames"], log["maps"]
    if not frames or not maps:
        raise SystemExit("log has no frames/maps — rerun ros/record_map.sh")
    os.makedirs(OUT_DIR, exist_ok=True)

    tmp = tempfile.mkdtemp()
    cache = {}          # map index -> rendered canvas (maps update ~1 Hz)
    path = []
    mi = -1
    for i, fr in enumerate(frames):
        while mi + 1 < len(maps) and maps[mi + 1]["t"] <= fr["t"]:
            mi += 1
        if mi < 0:
            base = np.full((SIZE, SIZE, 3), UNKNOWN, dtype=np.uint8)
        else:
            if mi not in cache:
                cache = {mi: map_image(maps[mi])}
            base = cache[mi]
        path.append(fr["pose"][:2])
        # frames are sampled at 10 Hz; duplicate each for smooth 20 fps playback
        img = draw_frame(base, path, fr["pose"], fr["scan"], fr["t"])
        img.save(os.path.join(tmp, f"f{2 * i:05d}.png"))
        img.save(os.path.join(tmp, f"f{2 * i + 1:05d}.png"))

    # final still: the finished map with the full path, no scan clutter
    final = draw_frame(map_image(maps[-1]), path, frames[-1]["pose"], [], frames[-1]["t"])
    png = os.path.join(OUT_DIR, "slam_map.png")
    final.save(png)

    mp4 = os.path.join(OUT_DIR, "slam_build.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS),
                    "-i", os.path.join(tmp, "f%05d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", mp4], check=True)
    shutil.rmtree(tmp)
    print(f"wrote {png}\nwrote {mp4}  ({len(frames)} samples, {len(maps)} map updates)")


if __name__ == "__main__":
    main()
