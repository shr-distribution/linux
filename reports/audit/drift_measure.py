#!/usr/bin/env python3
"""Measure per-frame vertical drift in NV12 640x480 captures.

For each frame we take the Y plane, build a row-mean profile (480 values),
and cross-correlate frame N's profile against frame 0's to find the vertical
shift (in lines) that best aligns them. Stable capture => shift ~0 for all N.
Rolling/drift => monotonic shift growing by ~30 lines/frame.
"""
import sys, numpy as np

W, H = 640, 480
YSZ = W * H
FRAME = W * H * 3 // 2  # NV12

def load_y(path):
    raw = np.fromfile(path, dtype=np.uint8)
    n = len(raw) // FRAME
    frames = []
    for i in range(n):
        off = i * FRAME
        y = raw[off:off + YSZ].reshape(H, W)
        frames.append(y)
    return frames

def row_profile(y):
    p = y.astype(np.float64).mean(axis=1)
    p -= p.mean()
    return p

def best_shift(ref, prof, maxshift=240):
    # circular cross-correlation; image rolls (wraps) so circular is right
    best_s, best_c = 0, -1e18
    n = len(ref)
    rn = np.linalg.norm(ref); pn = np.linalg.norm(prof)
    if rn < 1e-6 or pn < 1e-6:
        return None, 0.0
    for s in range(-maxshift, maxshift + 1):
        c = np.dot(ref, np.roll(prof, s)) / (rn * pn)
        if c > best_c:
            best_c, best_s = c, s
    return best_s, best_c

def analyze(path, label):
    frames = load_y(path)
    ref = row_profile(frames[0])
    print(f"\n=== {label}  ({len(frames)} frames) ===")
    print(f"  scene row-profile energy (std of frame0 rows): {frames[0].astype(float).mean(axis=1).std():.2f}")
    shifts = []
    prev = None
    for i, f in enumerate(frames):
        s, c = best_shift(ref, row_profile(f))
        delta = "" if prev is None or s is None else f"  d={s-prev:+d}"
        shifts.append(s)
        print(f"  frame {i:2d}: shift_vs_frame0 = {str(s).rjust(4)} lines  (corr={c:.3f}){delta}")
        prev = s
    valid = [s for s in shifts if s is not None]
    if len(valid) >= 3:
        diffs = np.diff(valid)
        print(f"  --> mean |per-frame step| = {np.abs(diffs).mean():.1f} lines, "
              f"total span = {max(valid)-min(valid)} lines")
    return shifts

a = analyze("cam_skip1.raw", "skip_short_pkt=1  (BASELINE / current default)")
b = analyze("cam_skip0.raw", "skip_short_pkt=0  (TEST / FS-FE short packets ON)")

print("\n=== VERDICT ===")
def span(s):
    v=[x for x in s if x is not None]
    return (max(v)-min(v)) if len(v)>1 else 0
print(f"  baseline (skip=1) vertical span: {span(a)} lines")
print(f"  test     (skip=0) vertical span: {span(b)} lines")
