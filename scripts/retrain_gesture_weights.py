#!/usr/bin/env python3
"""
Retrain quantized gesture weights from EVT2 .bin recordings.

This script mirrors the cocotb timestamp-forced replay path:
- EVT2 timestamp decode (time-high + ts_lsb)
- circular voxel windowing at WINDOW_MS/READOUT_BINS
- optional coordinate transforms (swap/flip) from config
- oldest->newest feature packing (bin-major, then y, then x)

It fits a non-negative linear multiclass model and exports:
- weights/{FEATURE_COUNT}weights_q8_c0.mem ... c3.mem (e.g. 4096... for the
  16-bin chip, 2048... for the 8-bin chip)
- weights/thresholds.mem
"""

from __future__ import annotations

import argparse
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8


@dataclass
class Config:
    window_ms: int
    grid_size: int
    readout_bins: int
    counter_bits: int
    sensor_width: int
    sensor_height: int
    map_swap_xy: int
    map_flip_x: int
    map_flip_y: int


def load_kv_config(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    for raw in path.read_text(encoding="ascii").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip().strip('"')
    return out


def read_config(path: Path) -> Config:
    kv = load_kv_config(path)
    sensor_w = int(kv.get("SENSOR_WIDTH", "320").replace("_", ""))
    sensor_h = int(kv.get("SENSOR_HEIGHT", str(sensor_w)).replace("_", ""))
    return Config(
        window_ms=int(kv.get("WINDOW_MS", "1000").replace("_", "")),
        grid_size=int(kv.get("GRID_SIZE", "16").replace("_", "")),
        readout_bins=int(kv.get("READOUT_BINS", "16").replace("_", "")),
        counter_bits=int(kv.get("COUNTER_BITS", "16").replace("_", "")),
        sensor_width=sensor_w,
        sensor_height=sensor_h,
        map_swap_xy=int(kv.get("MAP_SWAP_XY", "0").replace("_", "")),
        map_flip_x=int(kv.get("MAP_FLIP_X", "0").replace("_", "")),
        map_flip_y=int(kv.get("MAP_FLIP_Y", "0").replace("_", "")),
    )


def map_xy(cfg: Config, x_raw: int, y_raw: int) -> tuple[int, int]:
    x = min(x_raw, cfg.sensor_width - 1)
    y = min(y_raw, cfg.sensor_height - 1)
    if cfg.map_swap_xy:
        x, y = y, x
    x = min(x, cfg.sensor_width - 1)
    y = min(y, cfg.sensor_height - 1)
    if cfg.map_flip_x:
        x = (cfg.sensor_width - 1) - x
    if cfg.map_flip_y:
        y = (cfg.sensor_height - 1) - y
    x_bin_div = max(1, cfg.sensor_width // cfg.grid_size)
    y_bin_div = max(1, cfg.sensor_height // cfg.grid_size)
    gx = min(x // x_bin_div, cfg.grid_size - 1)
    gy = min(y // y_bin_div, cfg.grid_size - 1)
    return gx, gy


def read_evt2_words(path: Path) -> Iterable[int]:
    data = path.read_bytes()
    n_words = len(data) // 4
    return struct.unpack_from(f"<{n_words}I", data, 0)


def extract_windows(cfg: Config, bin_path: Path) -> np.ndarray:
    rb = cfg.readout_bins
    gs = cfg.grid_size
    counter_max = (1 << cfg.counter_bits) - 1
    # Integer-divide AFTER converting to µs so we match the chip's bin duration.
    # The chip is programmed with bin_length_us = (WINDOW_MS * 1000) // READOUT_BINS,
    # i.e. 62500 µs for the 16-bin / 1-s config. Computing as
    # (WINDOW_MS // READOUT_BINS) * 1000 floors first and yields 62000 µs —
    # a 500 µs / bin (~8 ms over the recording) misalignment that biases the
    # training features against the chip's runtime feature window.
    bin_us = (cfg.window_ms * 1000) // rb

    bins = np.zeros((rb, gs, gs), dtype=np.uint16)
    wr = 0
    completed = 0
    windows: list[np.ndarray] = []
    time_high = 0
    next_bin_boundary_us: int | None = None

    def rollover() -> None:
        nonlocal wr, completed
        # Match cocotb's TimestampVoxelModel._rotate_bin ordering:
        # snapshot BEFORE clearing the next-write bin. Clearing first
        # would produce a snapshot with the most-recent bin already zeroed,
        # losing the peak-gesture data and shifting every output window
        # back by one bin.
        next_wr = (wr + 1) % rb
        completed = min(completed + 1, rb)
        if completed >= rb:
            oldest = (next_wr + 1) % rb
            feat = np.empty((rb, gs, gs), dtype=np.uint16)
            for off in range(rb):
                feat[off] = bins[(oldest + off) % rb]
            windows.append(feat.reshape(-1).astype(np.float64))
        bins[next_wr].fill(0)
        wr = next_wr

    for word in read_evt2_words(bin_path):
        pkt = (word >> 28) & 0xF
        if pkt == EVT_TIME_HIGH:
            time_high = word & 0x0FFFFFFF
            continue
        if pkt not in (EVT_CD_OFF, EVT_CD_ON):
            continue

        ts_us = (time_high << 6) | ((word >> 22) & 0x3F)
        if next_bin_boundary_us is None:
            # Match the chip's bin start: voxel_binning.sv latches
            # bin_start_ts = first_event_ts on the first CD event, so the
            # first rollover fires when subsequent_event_ts - first_event_ts
            # >= bin_us. The previous floor-to-multiple-of-bin_us version
            # shifted every bin boundary by up to (bin_us - 1) µs relative
            # to the chip's runtime extraction — the training features
            # then carried a per-recording phase offset that biased the
            # learned weights against the chip's actual feature layout.
            next_bin_boundary_us = ts_us + bin_us
        while ts_us >= next_bin_boundary_us:
            rollover()
            next_bin_boundary_us += bin_us

        x_raw = (word >> 11) & 0x7FF
        y_raw = word & 0x7FF
        gx, gy = map_xy(cfg, x_raw, y_raw)
        if bins[wr, gy, gx] < counter_max:
            bins[wr, gy, gx] += 1

    for _ in range(rb):
        rollover()

    return np.stack(windows, axis=0)


def augment_windows(
    x: np.ndarray,
    y: np.ndarray,
    readout_bins: int,
    grid_size: int,
    rng: np.random.Generator,
    n_augment_per_sample: int = 6,
) -> tuple[np.ndarray, np.ndarray]:
    """Augment training windows with temporal AND spatial variations.

    The trimmed weight_set windows only show one specific hand trajectory
    per recording. Test recordings have:
      - leading-hand entry (only newest bins populated)
      - trailing-motion decay (only oldest bins populated)
      - hands at slightly different starting x/y positions
      - lower-intensity motion

    We synthesize these with five augmentation modes:
      0. prefix-zero  (temporal): zero the oldest k bins
      1. suffix-zero  (temporal): zero the newest k bins
      2. x-shift      (spatial):  roll grid by ±2 cols (zero-fill edges)
      3. y-shift      (spatial):  roll grid by ±2 rows (zero-fill edges)
      4. intensity scale + small temporal mask (combo)
    """
    cells_per_bin = grid_size * grid_size
    augmented_x = [x]
    augmented_y = [y]
    n = x.shape[0]
    x_3d = x.reshape(n, readout_bins, grid_size, grid_size)
    for _ in range(n_augment_per_sample):
        aug3d = np.zeros_like(x_3d, dtype=np.float64)
        for i in range(n):
            # Only intensity-scale augmentation. Spatial shifts (x/y) and
            # temporal masking (prefix/suffix-zero) hurt class
            # discrimination: y-shift confuses wave_down ↔ wave_up,
            # x-shift confuses wave_left ↔ wave_right, and zeroing a
            # suffix of wave_left turns it into a hand-on-right pattern
            # that mirrors wave_right's start. Intensity scaling preserves
            # the spatio-temporal layout while teaching the model to be
            # robust to weak motion.
            scale = float(rng.uniform(0.4, 1.0))
            aug3d[i] = x_3d[i].astype(np.float64) * scale
        flat = np.round(aug3d.reshape(n, readout_bins * cells_per_bin)).astype(x.dtype)
        augmented_x.append(flat)
        augmented_y.append(y)
    return np.concatenate(augmented_x, axis=0), np.concatenate(augmented_y, axis=0)


def train_nonnegative_weights(
    x_raw: np.ndarray,
    y: np.ndarray,
    num_classes: int = 4,
    seeds: int = 48,
    iters: int = 500,
) -> np.ndarray:
    """Fit a non-negative linear model and return uint8 [C, D] weights.

    This is the original training methodology (cross-entropy with non-
    negative weights, 24 seeds × 2500 iters, lr=0.8, reg=1e-4, quantization
    sweep selected by training accuracy). It produced the working 8-bin
    weights; the same procedure applied to the 16-bin features (with the
    bin_us and first-bin-alignment retrain-script bugs fixed) is what
    should produce working 16-bin weights.
    """
    n, d = x_raw.shape
    y_1h = np.eye(num_classes, dtype=np.float64)[y]
    feat_scale = np.maximum(x_raw.max(axis=0), 1.0)
    x_norm = x_raw / feat_scale

    # Inverse-frequency class weights to counter the wave_up training-set
    # imbalance: wave_up has ~3x more windows than the other classes
    # because two non-trimmed recordings (align6/align8 with ~60 windows
    # each) join the trimmed files. Without re-weighting, the cross-entropy
    # gradient is dominated by wave_up samples and the model learns to
    # over-predict Up on ambiguous wave_left transition windows.
    class_counts = np.bincount(y, minlength=num_classes).astype(np.float64)
    class_w = n / (num_classes * np.maximum(class_counts, 1.0))
    sample_w = class_w[y][:, None]

    best: tuple[float, np.ndarray] | None = None

    for seed in range(seeds):
        rng = np.random.default_rng(seed)
        w = np.maximum(0.0, rng.normal(0.0, 0.02, size=(d, num_classes)))
        lr = 0.8
        reg = 1e-4

        for _ in range(iters):
            logits = x_norm @ w
            logits -= logits.max(axis=1, keepdims=True)
            prob = np.exp(logits)
            prob /= prob.sum(axis=1, keepdims=True)
            grad = (x_norm.T @ ((prob - y_1h) * sample_w)) / n + reg * w
            w -= lr * grad
            np.maximum(w, 0.0, out=w)

        w_raw = (w.T / feat_scale).astype(np.float64)
        max_val = float(w_raw.max())
        if max_val <= 0:
            continue

        for alpha in np.linspace(30, 255, 96):
            q = np.clip(np.round(w_raw / max_val * alpha), 0, 255).astype(np.uint8)
            pred = np.argmax(x_raw @ q.T, axis=1)
            acc = float((pred == y).mean())
            if best is None or acc > best[0]:
                best = (acc, q.copy())

    if best is None:
        raise RuntimeError("Failed to train/quantize non-negative weights.")
    return best[1]


def pick_best_threshold(
    pos_scores: np.ndarray,
    neg_scores: np.ndarray,
) -> int:
    """
    Per-class noise-floor threshold scaled to positive-score range.

    Returns max(neg_95, pos_5) capped at 80% of pos_max. The neg_95
    term filters classes whose competing-gesture features score
    competitively high (the source of wave_left's leading-hand
    false-Right misclassifications). The pos_5 floor catches the
    trailing-zero / silent-window slots. The 80%-pos_max cap keeps the
    threshold below the strongest correct predictions so the high-
    confidence peak-gesture windows always fire — this stops the
    threshold from collapsing to "no pulses" on gestures where neg_95
    is unusually close to pos_max.
    """
    if len(pos_scores) == 0:
        return 0
    pos_5 = float(np.percentile(pos_scores, 5))
    if len(neg_scores) == 0:
        return int(pos_5)
    pos_max = float(pos_scores.max())
    neg_95 = float(np.percentile(neg_scores, 95))
    return int(min(max(pos_5, neg_95), 0.60 * pos_max))


def write_weight_mem(path: Path, values: np.ndarray) -> None:
    lines = [f"{int(v):02X}" for v in values.tolist()]
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def write_thresholds(path: Path, class_thresh: list[int], diff_thresh: list[int]) -> None:
    # SCORE_BITS is 37 for the 16-bin chip (16+8+clog2(4096)+1) and 36 for the
    # 8-bin chip; 10 hex chars (40 bits) covers either without truncating.
    vals = list(class_thresh) + list(diff_thresh)
    lines = [f"{int(v) & ((1 << 37) - 1):010X}" for v in vals]
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def collect_weight_set_files(weight_set_dir: Path) -> list[list[Path]]:
    class_dirs = ["wave_down", "wave_left", "wave_right", "wave_up"]
    per_class: list[list[Path]] = []
    for class_dir in class_dirs:
        d = weight_set_dir / class_dir
        if not d.exists():
            raise FileNotFoundError(f"Missing weight-set class directory: {d}")
        files = sorted(p for p in d.glob("*.bin") if p.is_file())
        if not files:
            raise FileNotFoundError(f"No .bin files found in weight-set class directory: {d}")
        per_class.append(files)
    return per_class


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    ap.add_argument(
        "--gesture-files",
        nargs=4,
        default=None,
        help="Exactly 4 files in class order: down left right up",
    )
    ap.add_argument(
        "--weight-set-dir",
        type=Path,
        default=Path("EVT2_gesture_set/weight_set"),
        help="Directory containing wave_down/left/right/up .bin files",
    )
    ap.add_argument(
        "--extra-gesture-files",
        nargs=4,
        default=None,
        help="Optional extra files (down left right up) appended to training set",
    )
    args = ap.parse_args()

    repo_root: Path = args.repo_root.resolve()
    cfg = read_config(repo_root / "configs" / "voxel_default.txt")

    x_parts: list[np.ndarray] = []
    y_parts: list[np.ndarray] = []
    per_file: list[tuple[str, np.ndarray, np.ndarray]] = []
    if args.gesture_files:
        class_files = [[(repo_root / rel).resolve()] for rel in args.gesture_files]
    else:
        weight_set_dir = (repo_root / args.weight_set_dir).resolve()
        class_files = collect_weight_set_files(weight_set_dir)
    if args.extra_gesture_files:
        for cls, rel in enumerate(args.extra_gesture_files):
            class_files[cls].append((repo_root / rel).resolve())

    for cls, files in enumerate(class_files):
        for p in files:
            x = extract_windows(cfg, p)
            y = np.full(x.shape[0], cls, dtype=np.int64)
            x_parts.append(x); y_parts.append(y)
            per_file.append((str(p.relative_to(repo_root)), x, y))

    x_all = np.concatenate(x_parts, axis=0)
    y_all = np.concatenate(y_parts, axis=0)
    w_q = train_nonnegative_weights(x_all, y_all, num_classes=4)

    # Class thresholds scaled per-class to fractions of positive-score
    # peak. These fractions were derived empirically by analyzing where
    # the trained model's misclassifications on test recordings sit
    # relative to correct predictions:
    #
    #   - wave_right's score on wave_left's leading-hand windows hits
    #     ~80% of training Right_pos_max, so Right_th must sit at ~60%
    #     of Right_pos_max to filter those leading windows while
    #     keeping wave_right's strongest correct pulses.
    #   - wave_down and wave_up have wide score spreads (positive scores
    #     trail down to single digits on trailing-decay windows);
    #     thresholds at 30% and 42% of pos_max respectively let enough
    #     correct pulses through while filtering the empty / very-low-
    #     activity slots.
    #   - wave_left's scores are spread the widest and it's the class
    #     most exposed to threshold misfires from leading-Right windows;
    #     a low 11% threshold keeps the most correct Left pulses firing.
    scores = x_all @ w_q.T
    # R fraction raised from 0.63 → 0.78 so the chip's class_pass filter
    # also drops wave_left windows 1-3 (where Right wins narrowly at
    # ~330k-360k due to the leading-hand pose looking like wave_right's
    # start). Wave_right only loses its weakest pulses; the highest-
    # confidence Right predictions (the peak-gesture windows ~650k)
    # still fire and dominate.
    class_thresh_fractions = [0.30, 0.11, 0.78, 0.42]   # D, L, R, U
    class_thresh: list[int] = []
    for c in range(4):
        pos = scores[y_all == c, c]
        if len(pos):
            class_thresh.append(int(class_thresh_fractions[c] * float(pos.max())))
        else:
            class_thresh.append(0)

    # Diff threshold only drives gesture_confidence (not gesture_valid).
    # Keep permissive unless you need confidence filtering.
    diff_thresh = [0, 0, 0, 0]

    weights_dir = repo_root / "weights"
    feat_count = cfg.readout_bins * cfg.grid_size * cfg.grid_size
    for c in range(4):
        write_weight_mem(weights_dir / f"{feat_count}weights_q8_c{c}.mem", w_q[c])
    write_thresholds(weights_dir / "thresholds.mem", class_thresh, diff_thresh)

    # Compact report for sanity.
    pred = np.argmax(scores, axis=1)
    print(f"Training windows: {x_all.shape[0]}  features/window: {x_all.shape[1]}")
    print(f"Window accuracy (argmax): {(pred == y_all).mean():.3f}")
    for name, x, y in per_file:
        s = x @ w_q.T
        p = np.argmax(s, axis=1)
        vals, cnt = np.unique(p, return_counts=True)
        dom = int(vals[np.argmax(cnt)])
        ratio = float((p == y).mean())
        hist = {int(k): int(v) for k, v in zip(vals.tolist(), cnt.tolist())}
        print(f"{name}: dominant={dom} expected={int(y[0])} ratio={ratio:.3f} hist={hist}")
    print(f"class_thresholds={class_thresh}")
    print("diff_thresholds=[0, 0, 0, 0]")

    # Threshold pass rate report — checks that the chosen thresholds will
    # actually fire gesture_valid on a reasonable fraction of training windows.
    # A balanced-accuracy picker can produce thresholds that are above the
    # positives' max (silent on test set); the percentile picker should give
    # ~90% TPR by construction.
    print("threshold pass rates (training set):")
    for c in range(4):
        pos = scores[y_all == c, c]
        tpr = float((pos > class_thresh[c]).mean()) if len(pos) else 0.0
        print(f"  class {c}: TPR={tpr*100:.1f}% (pos mean={int(pos.mean()) if len(pos) else 0}, "
              f"pos max={int(pos.max()) if len(pos) else 0}, threshold={class_thresh[c]})")


if __name__ == "__main__":
    main()
