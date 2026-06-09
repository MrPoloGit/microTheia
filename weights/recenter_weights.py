# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors
"""Remove the per-class DC (density) bias from the linear voxel gesture classifier.

Investigation summary
----------------------
The classifier score is a pure dot product  score_c = sum_i w[c][i] * feature[i]
with **no bias term** (see src/voxel_gesture_classifier.sv: the argmax is over the
raw MAC scores). The shipped int8 weights had very different DC gains per class:

    sum(weights):  Down -1391   Left -2184   Right -907   Up +4520

Because each feature window is ~95% dense (≈3900/4096 cells nonzero), every score is
dominated by a near-uniform "bulk activity" term ≈ density * sum(weights[c]). Up's
large positive weight-sum (concentrated in the newest time bins) made *any* dense
window score highest on Up, so the Down gesture lost to Up. The per-gesture tests
only passed because each appends a 1 s empty-bin flush whose sparse tail let Down
win on a majority vote; with gestures stitched back-to-back there is no flush, the
window stays dense, and Up wins the whole Down phase.

Fix (pure weights, no RTL change)
---------------------------------
Subtract, from every (class, time-bin) block of 256 cells, that block's mean weight.
This forces every class to have the same (zero) DC gain per time bin, so the bulk-
activity term becomes common-mode and cancels in the argmax — while the *relative*
weights within each block (the spatial pattern that actually encodes direction) are
left untouched. Re-quantized to signed int8; on the shipped weights this clips 0%.

The transform is idempotent (re-running on already-centered weights is a no-op).

Run:  python weights/recenter_weights.py
"""
from pathlib import Path

GRID_SIZE = 16
READOUT_BINS = 16
CELLS_PER_BIN = GRID_SIZE * GRID_SIZE          # 256
FEATURE_COUNT = READOUT_BINS * CELLS_PER_BIN    # 4096
NUM_CLASSES = 4
WEIGHTS_DIR = Path(__file__).resolve().parent
MEM = [WEIGHTS_DIR / f"4096weights_q8_c{c}.mem" for c in range(NUM_CLASSES)]
BACKUP_DIR = WEIGHTS_DIR / "orig_pre_recenter"


def to_signed(v, bits=8):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v & (1 << (bits - 1)) else v


def read_mem(path):
    vals = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("//"):
            continue
        vals.append(to_signed(int(line, 16), 8))
    assert len(vals) == FEATURE_COUNT, f"{path.name}: got {len(vals)} weights"
    return vals


def write_mem(path, vals):
    path.write_text("".join(f"{v & 0xFF:02X}\n" for v in vals), encoding="ascii")


def recenter(vals):
    out = []
    clipped = 0
    for b in range(READOUT_BINS):
        blk = vals[b * CELLS_PER_BIN:(b + 1) * CELLS_PER_BIN]
        mean = sum(blk) / len(blk)
        for w in blk:
            v = round(w - mean)
            if v > 127:
                v = 127; clipped += 1
            elif v < -128:
                v = -128; clipped += 1
            out.append(v)
    return out, clipped


def main():
    BACKUP_DIR.mkdir(exist_ok=True)
    total_clip = 0
    for c, path in enumerate(MEM):
        vals = read_mem(path)
        backup = BACKUP_DIR / path.name
        if not backup.exists():                 # preserve pristine originals once
            write_mem(backup, vals)
        centered, clipped = recenter(vals)
        total_clip += clipped
        write_mem(path, centered)
        print(f"class {c}: sum {sum(vals):+6d} -> {sum(centered):+4d}   clipped {clipped}/{FEATURE_COUNT}")
    print(f"done: {total_clip} weights clipped to int8 range "
          f"({100*total_clip/(FEATURE_COUNT*NUM_CLASSES):.2f}%)")
    print(f"originals backed up under {BACKUP_DIR}")


if __name__ == "__main__":
    main()
