# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors
#
# soc_tb — integration testbench for src/soc.sv.
#
# Debugging behavior:
#   1. Stream the real EVT2 .bin recording.
#   2. Periodically print compact live status and full signed 37-bit MAC scores.
#   3. Append exactly one synthetic timestamp-advanced CD event after the stream.
#   4. Wait only for the first class_valid pulse.
#   5. Use the first gesture_valid/class_valid result as the recording result.
#
# RUN_ALL_BIN_FILES=1 behavior:
#   - test_soc_boot_then_stream_one_bin_file skips itself.
#   - test_soc_boot_then_stream_all_bin_files runs all 4 recordings.
#   - default stream log interval becomes 50,000 words instead of 1,000.
#   - each recording result is logged immediately and again in a final summary.

from collections import Counter
from pathlib import Path
import os
import struct

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, NextTimeStep, ReadOnly

try:
    from util.config_parser import load_config
except Exception:
    load_config = None


# ── Config ────────────────────────────────────────────────────────────────────
MODULE = os.environ.get("TOPLEVEL", "soc")

if load_config is not None:
    try:
        CFG = load_config(MODULE)
    except Exception:
        CFG = {}
else:
    CFG = {}

RUN_ALL_BIN_FILES = int(os.environ.get("RUN_ALL_BIN_FILES", "0"))

CLK_FREQ_HZ = int(os.environ.get("CLK_FREQ_HZ", "64000000"))
_RAW_PERIOD_PS = int(round(1_000_000_000_000 / CLK_FREQ_HZ))
CHIP_PERIOD_PS = _RAW_PERIOD_PS + (_RAW_PERIOD_PS % 2)
CHIP_PERIOD_NS = CHIP_PERIOD_PS / 1000.0

DATA_WIDTH = 32

GRID_SIZE = int(CFG.get("GRID_SIZE", os.environ.get("GRID_SIZE", 16)))
READOUT_BINS = int(CFG.get("READOUT_BINS", os.environ.get("READOUT_BINS", 16)))
NUM_CLASSES = int(CFG.get("NUM_CLASSES", os.environ.get("NUM_CLASSES", 4)))
WINDOW_MS = int(CFG.get("WINDOW_MS", os.environ.get("WINDOW_MS", 1000)))
SENSOR_WIDTH = int(CFG.get("SENSOR_WIDTH", os.environ.get("SENSOR_WIDTH", 320)))
SENSOR_HEIGHT = int(CFG.get("SENSOR_HEIGHT", os.environ.get("SENSOR_HEIGHT", SENSOR_WIDTH)))

FEATURE_COUNT = GRID_SIZE * GRID_SIZE * READOUT_BINS

# Full classifier/MAC score width:
# COUNTER_BITS + WEIGHT_BITS + clog2(FEATURE_COUNT) + 1 = 37 for the current chip.
SCORE_BITS = int(CFG.get("SCORE_BITS", os.environ.get("SCORE_BITS", 37)))

BIN_LENGTH_US = int(
    os.environ.get("BIN_LENGTH_US", str((WINDOW_MS * 1000) // READOUT_BINS))
)

SPI_HALF_CHIP_CYCLES_DEFAULT = int(os.environ.get("SPI_HALF_CHIP_CYCLES", "1"))
MAX_BIN_WORDS = int(os.environ.get("MAX_BIN_WORDS", "0"))
ASSERT_EXPECTED_LABEL = int(os.environ.get("ASSERT_EXPECTED_LABEL", "1"))

# Run-all mode is long, so make the default much quieter.
_DEFAULT_LOG_WORDS = "50000" if RUN_ALL_BIN_FILES else "1000"
LOG_EVERY_WORDS = int(os.environ.get("LOG_EVERY_WORDS", _DEFAULT_LOG_WORDS))

# Print compact pipeline/scores while the real .bin is streaming.
# Set to 0 to disable live stream score logs.
STREAM_SCORE_LOG_EVERY_WORDS = int(
    os.environ.get("STREAM_SCORE_LOG_EVERY_WORDS", _DEFAULT_LOG_WORDS)
)

# Append one synthetic TIME_HIGH + CD_OFF pair after the real stream to give
# the binner one final rollover opportunity.
POST_STREAM_SINGLE_FLUSH_EVENT = int(
    os.environ.get("POST_STREAM_SINGLE_FLUSH_EVENT", "1")
)

# Wait only for the first class_valid pulse after streaming and the single
# synthetic rollover event.
FIRST_CLASSIFICATION_TIMEOUT_CYCLES = int(
    os.environ.get("FIRST_CLASSIFICATION_TIMEOUT_CYCLES", "500000")
)

GESTURE_NAMES = {0: "Down", 1: "Left", 2: "Right", 3: "Up"}
EXPECTED_BIN_FILE_CLASS = {0: 0, 1: 1, 2: 2, 3: 3}


# ── EVT2 opcode encodings ─────────────────────────────────────────────────────
EVT_WEIGHT = 0x2
EVT_THRESH_U = 0x3
EVT_THRESH_L = 0x4
EVT_BIN_LENGTH_U = 0x5
EVT_BIN_LENGTH_L = 0x6
EVT_BOOT_REQ = 0xC
DEBUG_PAGE = 0xE
EVT_READS_DONE = 0xF

EVT_TIME_HIGH = 0x8
EVT_CD_OFF = 0x0


def build_evt2_weight(weight, feature_addr, class_id):
    return (
        ((EVT_WEIGHT & 0xF) << 28)
        | ((int(weight) & 0xFF) << 20)
        | ((int(feature_addr) & 0xFFF) << 8)
        | ((int(class_id) & 0x3F) << 2)
    )


def build_evt2_thresh_upper(threshold_value, threshold_addr):
    threshold_value = int(threshold_value) & ((1 << SCORE_BITS) - 1)
    upper = (threshold_value >> 18) & 0x7FFFF
    return ((EVT_THRESH_U & 0xF) << 28) | ((upper & 0x7FFFF) << 9)


def build_evt2_thresh_lower(threshold_value, threshold_addr):
    threshold_value = int(threshold_value) & ((1 << SCORE_BITS) - 1)
    lower = threshold_value & 0x3FFFF
    return (
        ((EVT_THRESH_L & 0xF) << 28)
        | ((lower & 0x3FFFF) << 10)
        | ((int(threshold_addr) & 0x7) << 7)
    )


def build_evt2_bin_length_upper(bin_length_us):
    return ((EVT_BIN_LENGTH_U & 0xF) << 28) | ((int(bin_length_us) >> 17) & 0x1FFFF)


def build_evt2_bin_length_lower(bin_length_us):
    return ((EVT_BIN_LENGTH_L & 0xF) << 28) | (int(bin_length_us) & 0x1FFFF)


def build_evt2_reads_done():
    return (EVT_READS_DONE & 0xF) << 28


def build_evt2_debug_page(page):
    return ((DEBUG_PAGE & 0xF) << 28) | ((int(page) & 0xF) << 24)


def build_evt2_boot_req():
    return (EVT_BOOT_REQ & 0xF) << 28


# ── File loading ──────────────────────────────────────────────────────────────
_REPO_ROOT = Path(__file__).resolve().parents[1]


def load_weights_from_mem():
    """weights[class_id][feature_addr]"""
    mem_keys = [f"WEIGHT_MEM_C{c}" for c in range(NUM_CLASSES)]
    mem_defaults = [
        f"weights/{FEATURE_COUNT}weights_q8_c{c}.mem"
        for c in range(NUM_CLASSES)
    ]

    weights = []
    for c in range(NUM_CLASSES):
        path = _REPO_ROOT / CFG.get(mem_keys[c], mem_defaults[c])
        if not path.exists():
            raise FileNotFoundError(f"Missing weight file for class {c}: {path}")

        vals = []
        for line in path.read_text(encoding="ascii").splitlines():
            line = line.strip()
            if not line or line.startswith("//"):
                continue
            try:
                vals.append(int(line, 16))
            except ValueError:
                vals.append(0)

        while len(vals) < FEATURE_COUNT:
            vals.append(0)

        weights.append(vals[:FEATURE_COUNT])

    return weights


def load_thresholds():
    """addr 0..NUM_CLASSES-1 = class thresholds; addr NUM_CLASSES..2N-1 = diff thresholds."""
    path = _REPO_ROOT / "weights" / "thresholds.mem"
    if not path.exists():
        cocotb.log.warning(f"No thresholds.mem found at {path}; using zeros")
        return [0] * (2 * NUM_CLASSES)

    vals = []
    for line in path.read_text(encoding="ascii").splitlines():
        line = line.strip()
        if not line or line.startswith("//"):
            continue
        try:
            vals.append(int(line, 16))
        except ValueError:
            vals.append(0)

    while len(vals) < 2 * NUM_CLASSES:
        vals.append(0)

    return vals[: 2 * NUM_CLASSES]


def build_program_stream_words(weights, thresholds, bin_length_us=None):
    if bin_length_us is None:
        bin_length_us = BIN_LENGTH_US

    words = []

    for class_id in range(NUM_CLASSES):
        for feature_addr in range(FEATURE_COUNT):
            words.append(
                build_evt2_weight(
                    weight=weights[class_id][feature_addr],
                    feature_addr=feature_addr,
                    class_id=class_id,
                )
            )

    for threshold_addr, threshold_value in enumerate(thresholds):
        words.append(build_evt2_thresh_upper(threshold_value, threshold_addr))
        words.append(build_evt2_thresh_lower(threshold_value, threshold_addr))

    words.append(build_evt2_bin_length_upper(bin_length_us))
    words.append(build_evt2_bin_length_lower(bin_length_us))
    words.append(build_evt2_reads_done())

    return words


def _default_bin_paths():
    test_set = _REPO_ROOT / "EVT2_gesture_set" / "test_set"
    legacy = _REPO_ROOT / "EVT2_gesture_set"
    root = test_set if test_set.exists() else legacy

    return [
        root / "wave_down_sun_test1.bin",
        root / "wave_left_sun_test1.bin",
        root / "wave_right_sun_test1.bin",
        root / "wave_up_sun_test1.bin",
    ]


def _resolve_bin_files():
    """Override with GESTURE_BIN_FILES='path0:path1:path2:path3'."""
    env = os.environ.get("GESTURE_BIN_FILES", "")

    if env.strip():
        parts = [p.strip() for p in env.split(":") if p.strip()]
        if len(parts) != 4:
            raise ValueError(
                f"GESTURE_BIN_FILES must contain exactly 4 colon-separated paths, "
                f"got {len(parts)}: {parts}"
            )

        resolved = []
        for p in parts:
            path = Path(p)
            if not path.is_absolute():
                path = _REPO_ROOT / path
            resolved.append(path)

        return resolved

    return list(_default_bin_paths())


def _read_evt2_bin(path):
    """Read raw EVT2.0 file as little-endian 32-bit words; reject LFS pointers."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Missing EVT2 .bin file: {path}")

    data = path.read_bytes()

    if data.startswith(b"version https://git-lfs.github.com/spec/v1"):
        raise RuntimeError(
            f"{path} is a Git LFS pointer file, not the real .bin recording.\n"
            f"Run 'git lfs install && git lfs pull' from the repo root."
        )

    if len(data) % 4 != 0:
        raise RuntimeError(f"{path} size is {len(data)} bytes, not divisible by 4.")

    n_words = len(data) // 4
    words = list(struct.unpack_from(f"<{n_words}I", data, 0))

    if MAX_BIN_WORDS > 0:
        words = words[:MAX_BIN_WORDS]

    return words


# ── Reset / startup ───────────────────────────────────────────────────────────
def _drive_idle(dut):
    dut.SCLK.value = 0
    dut.CS.value = 1
    dut.MOSI.value = 0


async def _apply_reset(dut, hold_cycles=16, post_cycles=50):
    await NextTimeStep()
    _drive_idle(dut)

    dut.rst.value = 1
    await ClockCycles(dut.clk, hold_cycles)
    await NextTimeStep()
    dut.rst.value = 0
    await ClockCycles(dut.clk, post_cycles)


async def setup_system(dut):
    dut._log.info(
        f"Clock: {CLK_FREQ_HZ} Hz, period={CHIP_PERIOD_PS} ps ({CHIP_PERIOD_NS:.3f} ns)"
    )
    cocotb.start_soon(Clock(dut.clk, CHIP_PERIOD_PS, "ps").start())
    await _apply_reset(dut)


async def reset_system_no_new_clock(dut):
    await _apply_reset(dut)


async def wait_for_spi_ready(dut, max_cycles=5000):
    for cycle in range(max_cycles):
        await RisingEdge(dut.clk)
        if int(dut.spi_ready.value):
            dut._log.info(f"spi_ready after {cycle} cycles")
            return

    raise AssertionError("spi_ready never asserted")


# ── Internal signal helpers ───────────────────────────────────────────────────
def _read_sig(handle, name):
    """Read a signal by dotted path; return int or '?' if unreachable."""
    try:
        obj = handle
        for p in name.split("."):
            obj = getattr(obj, p)
        return int(obj.value)
    except Exception:
        return "?"


def twos_to_signed(value, bits):
    value = int(value) & ((1 << bits) - 1)
    sign_bit = 1 << (bits - 1)
    return value - (1 << bits) if (value & sign_bit) else value


def unpack_signed_scores_flat(raw_value, num_classes=NUM_CLASSES, score_bits=SCORE_BITS):
    raw_value = int(raw_value)
    mask = (1 << score_bits) - 1

    scores = []
    for g in range(num_classes):
        raw_score = (raw_value >> (g * score_bits)) & mask
        scores.append(twos_to_signed(raw_score, score_bits))

    return scores


def _read_sig_signed(handle, name, bits):
    try:
        obj = handle
        for p in name.split("."):
            obj = getattr(obj, p)
        return twos_to_signed(int(obj.value), bits)
    except Exception:
        return "?"


def _read_mac_scores_signed(dut):
    try:
        raw = int(dut.u_core.mac_scores_flat.value)
        return unpack_signed_scores_flat(raw, NUM_CLASSES, SCORE_BITS)
    except Exception:
        return [
            _read_sig_signed(dut, "u_core.score_A", 32),
            _read_sig_signed(dut, "u_core.score_B", 32),
            _read_sig_signed(dut, "u_core.score_C", 32),
            _read_sig_signed(dut, "u_core.score_D", 32),
        ]


def _read_classifier_decision_debug(dut):
    return {
        "max_class": _read_sig(dut, "u_core.u_voxel_gesture_classifier.max_class_r2"),
        "max_score": _read_sig_signed(
            dut, "u_core.u_voxel_gesture_classifier.max_score_r2", SCORE_BITS
        ),
        "diff": _read_sig_signed(
            dut, "u_core.u_voxel_gesture_classifier.diff_r2", SCORE_BITS
        ),
        "class_thresh": _read_sig_signed(
            dut, "u_core.u_voxel_gesture_classifier.class_thresh_r", SCORE_BITS
        ),
        "diff_thresh": _read_sig_signed(
            dut, "u_core.u_voxel_gesture_classifier.thresh_data_s", SCORE_BITS
        ),
    }


def _gesture_name(value):
    if value == "?":
        return "?"
    return GESTURE_NAMES.get(int(value), int(value))


def _scores_str(scores):
    return "[" + ", ".join(str(s) for s in scores) + "]"


def _result_name(result):
    g = result.get("gesture", result.get("class_gesture", "?"))
    return _gesture_name(g)


async def log_pipeline_state(dut, tag=""):
    scores = _read_mac_scores_signed(dut)
    class_valid = _read_sig(dut, "u_core.class_valid")
    class_pass = _read_sig(dut, "u_core.class_pass")
    class_gesture = _read_sig(dut, "u_core.class_gesture")

    dut._log.info(
        f"{tag}: "
        f"scores={_scores_str(scores)} "
        f"class=({_gesture_name(class_gesture)}, valid={class_valid}, pass={class_pass}) "
        f"bins={_read_sig(dut, 'u_core.u_voxel_binning.completed_bins')} "
        f"fwr={_read_sig(dut, 'u_core.feature_window_ready')} "
        f"mac_busy={_read_sig(dut, 'u_core.mac_busy')}"
    )


# ── SPI mode-0 streaming ──────────────────────────────────────────────────────
async def spi_mode0_stream_words(
    dut,
    mosi_words,
    width=DATA_WIDTH,
    half_cycles=SPI_HALF_CHIP_CYCLES_DEFAULT,
    pre_start_cycles=4,
    inter_word_low_cycles=4,
    post_finish_cycles=4,
    cs_high_gap_cycles=4,
    tag="spi_stream",
    capture_miso=False,
    log_scores_during_stream=False,
):
    assert len(mosi_words) > 0, "mosi_words must not be empty"

    dut._log.info(
        f"{tag}: stream {len(mosi_words)} words "
        f"(half_cycles={half_cycles}, log_every={STREAM_SCORE_LOG_EVERY_WORDS if log_scores_during_stream else LOG_EVERY_WORDS})"
    )

    miso_words = [] if capture_miso else None

    _drive_idle(dut)
    await ClockCycles(dut.clk, cs_high_gap_cycles)

    first_word = int(mosi_words[0]) & 0xFFFFFFFF
    dut.CS.value = 0
    dut.MOSI.value = (first_word >> (width - 1)) & 1
    await ClockCycles(dut.clk, pre_start_cycles)

    for word_idx, mosi_word in enumerate(mosi_words):
        mosi_word = int(mosi_word) & 0xFFFFFFFF
        miso_word = 0

        if (
            log_scores_during_stream
            and STREAM_SCORE_LOG_EVERY_WORDS > 0
            and word_idx % STREAM_SCORE_LOG_EVERY_WORDS == 0
        ):
            await log_pipeline_state(dut, f"{tag} word {word_idx}/{len(mosi_words)}")
        elif (
            not log_scores_during_stream
            and LOG_EVERY_WORDS > 0
            and word_idx % LOG_EVERY_WORDS == 0
        ):
            dut._log.info(f"{tag}: word {word_idx}/{len(mosi_words)}")

        for bit_idx in range(width):
            dut.SCLK.value = 1
            await ClockCycles(dut.clk, half_cycles)

            if capture_miso:
                miso_word = (miso_word << 1) | int(dut.MISO.value)

            dut.SCLK.value = 0
            await ClockCycles(dut.clk, half_cycles)

            if bit_idx != width - 1:
                dut.MOSI.value = (mosi_word >> (width - 2 - bit_idx)) & 1

        if capture_miso:
            miso_words.append(miso_word)

        await ClockCycles(dut.clk, inter_word_low_cycles)

        if word_idx != len(mosi_words) - 1:
            next_word = int(mosi_words[word_idx + 1]) & 0xFFFFFFFF
            dut.MOSI.value = (next_word >> (width - 1)) & 1

    await ClockCycles(dut.clk, post_finish_cycles)
    _drive_idle(dut)
    await ClockCycles(dut.clk, 8)

    if log_scores_during_stream:
        await log_pipeline_state(dut, f"{tag} done")

    dut._log.info(f"{tag}: finished")
    return miso_words if capture_miso else []


async def spi_mode0_transfer_word(dut, mosi_word, tag="spi_single"):
    miso_words = await spi_mode0_stream_words(
        dut,
        [mosi_word],
        tag=tag,
        capture_miso=True,
    )
    return miso_words[0]


def expected_miso_from_classification(gesture, confidence):
    classification = ((int(confidence) & 1) << 2) | (int(gesture) & 0b11)
    return classification << (DATA_WIDTH - 3)


# ── Single-rollover synthetic event generation ────────────────────────────────
def _extract_last_evt2_timestamp_us(words):
    time_high = 0
    last_ts_us = 0

    for w in words:
        w = int(w)
        evt_type = (w >> 28) & 0xF

        if evt_type == EVT_TIME_HIGH:
            time_high = w & 0x0FFFFFFF
            last_ts_us = time_high * 64

        elif evt_type == EVT_CD_OFF:
            ts_lsb = (w >> 22) & 0x3F
            last_ts_us = time_high * 64 + ts_lsb

    return last_ts_us


def build_single_bin_rollover_words(real_words, bin_length_us=None):
    if bin_length_us is None:
        bin_length_us = BIN_LENGTH_US

    last_ts_us = _extract_last_evt2_timestamp_us(real_words)
    flush_ts_us = last_ts_us + int(bin_length_us)

    th_val = (flush_ts_us >> 6) & 0x0FFFFFFF
    ts_lsb = flush_ts_us & 0x3F

    return [
        (EVT_TIME_HIGH << 28) | th_val,
        (EVT_CD_OFF << 28) | (ts_lsb << 22) | (160 << 11) | 160,
    ]


# ── Internal-signal monitors ──────────────────────────────────────────────────
class GestureMonitor:
    def __init__(self):
        self.pulses = []
        self.class_events = []

    @property
    def count(self):
        return len(self.pulses)

    @property
    def class_count(self):
        return len(self.class_events)

    def dominant(self):
        if not self.pulses:
            return None, None

        counts = Counter(g for g, _ in self.pulses)
        winner = counts.most_common(1)[0][0]

        for g, c in reversed(self.pulses):
            if g == winner:
                return winner, c

        return winner, 0

    def histogram_str(self):
        if not self.pulses:
            return "(none)"

        counts = Counter(g for g, _ in self.pulses)
        return " ".join(
            f"{GESTURE_NAMES.get(g, g)}:{n}"
            for g, n in sorted(counts.items())
        )

    async def run(self, dut):
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()

            scores = _read_mac_scores_signed(dut)
            dec = _read_classifier_decision_debug(dut)

            class_valid = _read_sig(dut, "u_core.class_valid")
            class_pass = _read_sig(dut, "u_core.class_pass")
            class_g = _read_sig(dut, "u_core.class_gesture")

            try:
                gesture_valid = int(dut.gesture_valid.value)
            except Exception:
                gesture_valid = 0

            try:
                g = int(dut.gesture.value)
            except Exception:
                g = 0

            try:
                c = int(dut.gesture_confidence.value)
            except Exception:
                c = 0

            if class_valid == 1:
                event = {
                    "class_gesture": class_g,
                    "class_pass": class_pass,
                    "scores": scores,
                    "decision": dec,
                }
                self.class_events.append(event)

                dut._log.info(
                    f"class_valid #{self.class_count}: "
                    f"class={_gesture_name(class_g)} pass={class_pass} "
                    f"scores={_scores_str(scores)} "
                    f"max={_gesture_name(dec['max_class'])}:{dec['max_score']} "
                    f"diff={dec['diff']}"
                )

            if gesture_valid:
                self.pulses.append((g, c))
                dut._log.info(
                    f"gesture_valid #{self.count}: "
                    f"gesture={GESTURE_NAMES.get(g, g)} conf={c} "
                    f"scores={_scores_str(scores)}"
                )


class HandshakeMonitor:
    def __init__(self):
        self.count = 0

    async def run(self, dut):
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()

            try:
                v = int(dut.evt_word_valid.value)
                r = int(dut.evt_word_ready.value)
            except Exception:
                continue

            if v and r:
                self.count += 1


async def wait_for_evt_ld_en(dut, max_cycles=5000):
    for cycle in range(max_cycles):
        await RisingEdge(dut.clk)
        if _read_sig(dut, "u_core.evt_ld_en") == 1:
            dut._log.info(f"evt_ld_en after {cycle} cycles")
            return

    raise AssertionError(
        "Timed out waiting for evt_ld_en after BOOT_REQ: "
        f"core_rst_o={_read_sig(dut, 'u_core.core_rst_o')} "
        f"boot_done_o={_read_sig(dut, 'u_core.boot_done_o')} "
        f"main_state={_read_sig(dut, 'u_core.main_state_dbg_o')} "
        f"load_state={_read_sig(dut, 'u_core.load_state_dbg_o')}"
    )


async def wait_after_reads_done(dut):
    await ClockCycles(dut.clk, 100)

    dut._log.info(
        f"POST-BOOT: "
        f"core_rst={_read_sig(dut, 'u_core.core_rst_o')} "
        f"boot_done={_read_sig(dut, 'u_core.boot_done_o')} "
        f"evt_ld_en={_read_sig(dut, 'u_core.evt_ld_en')}"
    )


async def wait_for_classification_count(dut, monitor, min_count=1, max_cycles=500000):
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if monitor.class_count >= min_count:
            return

    await log_pipeline_state(dut, "class-valid-timeout")

    raise AssertionError(
        f"Timed out waiting for class_valid count >= {min_count}; "
        f"class_count={monitor.class_count}, gesture_count={monitor.count}"
    )


# ── High-level system helpers ─────────────────────────────────────────────────
async def boot_core_over_spi(dut, hs_monitor=None):
    weights = load_weights_from_mem()
    thresholds = load_thresholds()

    program_words = build_program_stream_words(weights, thresholds)

    dut._log.info(
        f"Boot: {len(program_words)} program words, "
        f"features={FEATURE_COUNT}, classes={NUM_CLASSES}, bin_us={BIN_LENGTH_US}"
    )

    await spi_mode0_stream_words(dut, [build_evt2_boot_req()], tag="boot_req")
    await wait_for_evt_ld_en(dut)
    await spi_mode0_stream_words(dut, program_words, tag="program")
    await wait_after_reads_done(dut)

    if hs_monitor is not None:
        expected = 1 + len(program_words)
        dut._log.info(f"Handshake count after boot: {hs_monitor.count} / expected ~{expected}")

    return weights, thresholds


async def stream_bin_recording_over_spi(dut, bin_path):
    real_words = _read_evt2_bin(bin_path)

    dut._log.info(f"Recording: {Path(bin_path).name}, real_words={len(real_words)}")

    await spi_mode0_stream_words(
        dut,
        real_words,
        tag=f"evt2_{Path(bin_path).stem}",
        capture_miso=False,
        log_scores_during_stream=True,
    )

    if POST_STREAM_SINGLE_FLUSH_EVENT:
        flush_words = build_single_bin_rollover_words(real_words, bin_length_us=BIN_LENGTH_US)

        dut._log.info(
            f"Single rollover event: {[f'0x{w:08X}' for w in flush_words]}"
        )

        await spi_mode0_stream_words(
            dut,
            flush_words,
            tag=f"rollover_{Path(bin_path).stem}",
            capture_miso=False,
            log_scores_during_stream=True,
        )

    return []


async def read_classification_over_spi(dut):
    return await spi_mode0_transfer_word(
        dut,
        build_evt2_debug_page(0),
        tag="classification_readback",
    )


async def _classify_one_recording(dut, bin_path):
    monitor = GestureMonitor()
    monitor_task = cocotb.start_soon(monitor.run(dut))

    await stream_bin_recording_over_spi(dut, bin_path)

    dut._log.info(
        f"Waiting for first class_valid, timeout={FIRST_CLASSIFICATION_TIMEOUT_CYCLES} cycles"
    )

    await wait_for_classification_count(
        dut,
        monitor,
        min_count=1,
        max_cycles=FIRST_CLASSIFICATION_TIMEOUT_CYCLES,
    )

    await log_pipeline_state(dut, "after-first-class-valid")

    # Allow any same-cycle gesture_valid log to settle, then stop monitoring.
    await ClockCycles(dut.clk, 1)

    monitor_task.cancel()

    if not monitor.class_events:
        raise AssertionError(
            f"No class_valid pulse fired after streaming {Path(bin_path).name} "
            f"and one synthetic rollover event"
        )

    return monitor


async def _readback_and_build_result(dut, bin_path, bin_index, monitor):
    expected_class = EXPECTED_BIN_FILE_CLASS.get(bin_index)

    first_class = monitor.class_events[0]
    first_class_g = first_class["class_gesture"]
    first_class_pass = first_class["class_pass"]
    scores = first_class["scores"]

    if not monitor.pulses:
        raise AssertionError(
            f"[{Path(bin_path).name}] class_valid occurred but no gesture_valid was observed"
        )

    first_g, first_c = monitor.pulses[0]

    await ClockCycles(dut.clk, 20)

    miso_word = await read_classification_over_spi(dut)
    expected_miso = expected_miso_from_classification(first_g, first_c)

    miso_ok = miso_word == expected_miso
    expected_ok = (
        expected_class is None
        or ASSERT_EXPECTED_LABEL == 0
        or first_g == expected_class
    )

    result = {
        "name": Path(bin_path).name,
        "index": bin_index,
        "expected": expected_class,
        "class_gesture": first_class_g,
        "class_pass": first_class_pass,
        "gesture": first_g,
        "confidence": first_c,
        "scores": scores,
        "decision": first_class["decision"],
        "miso_word": miso_word,
        "expected_miso": expected_miso,
        "miso_ok": miso_ok,
        "expected_ok": expected_ok,
        "status": "PASS" if first_class_pass == 1 and miso_ok and expected_ok else "FAIL",
    }

    return result


def _log_result(dut, result, prefix="RESULT"):
    dut._log.info(
        f"{prefix} [{result['name']}]: {result['status']} "
        f"expected={_gesture_name(result['expected'])} "
        f"got={_gesture_name(result['gesture'])} "
        f"conf={result['confidence']} "
        f"scores={_scores_str(result['scores'])} "
        f"MISO=0x{result['miso_word']:08X}"
    )


def _assert_result_ok(result):
    assert result["class_pass"] == 1, (
        f"[{result['name']}] first class_valid did not pass threshold: "
        f"class={_gesture_name(result['class_gesture'])}, scores={result['scores']}, "
        f"decision={result['decision']}"
    )

    assert result["miso_ok"], (
        f"[{result['name']}] MISO mismatch: "
        f"got 0x{result['miso_word']:08X}, expected 0x{result['expected_miso']:08X}"
    )

    assert result["expected_ok"], (
        f"[{result['name']}] expected {_gesture_name(result['expected'])}, "
        f"got {_gesture_name(result['gesture'])}, scores={result['scores']}"
    )


# ── Tests ─────────────────────────────────────────────────────────────────────
@logged_test()
async def test_soc_boot_stream_over_spi(dut):
    await setup_system(dut)
    await wait_for_spi_ready(dut)

    hs = HandshakeMonitor()
    hs_task = cocotb.start_soon(hs.run(dut))

    await boot_core_over_spi(dut, hs_monitor=hs)

    hs_task.cancel()

    min_expected = NUM_CLASSES * FEATURE_COUNT
    assert hs.count >= min_expected, (
        f"spi_wrapper -> voxel_bin_core handshake count too low: "
        f"got {hs.count}, expected >= {min_expected}"
    )

    dut._log.info(f"PASS: boot stream accepted, handshake_count={hs.count}")


@logged_test()
async def test_soc_debug_page_sweep(dut):
    PAGE_NAMES = {
        0: "classifier+mac",
        1: "voxel_binning",
        2: "evt2+fifo+core",
        3: "control",
        4: "evt2 output",
    }

    await setup_system(dut)
    await wait_for_spi_ready(dut)

    await spi_mode0_stream_words(dut, [build_evt2_boot_req()], tag="boot_req")
    await ClockCycles(dut.clk, 20)

    for page in range(5):
        await spi_mode0_stream_words(
            dut,
            [build_evt2_debug_page(page)],
            tag=f"page_{page}",
        )
        await ClockCycles(dut.clk, 10)

        try:
            bus = int(dut.debug_bus.value)
            dut._log.info(f"Page {page} ({PAGE_NAMES[page]}): debug_bus=0x{bus:08X}")
        except Exception:
            dut._log.info(f"Page {page} ({PAGE_NAMES[page]}): debug_bus=X/Z")

    dut._log.info("PASS: debug page sweep")


@logged_test()
async def test_soc_boot_then_stream_one_bin_file(dut):
    if RUN_ALL_BIN_FILES:
        dut._log.info("Skipping single-bin test because RUN_ALL_BIN_FILES=1")
        return

    bin_files = _resolve_bin_files()
    bin_index = int(os.environ.get("BIN_INDEX", "0"))

    assert 0 <= bin_index < len(bin_files), f"Invalid BIN_INDEX={bin_index}"

    bin_path = bin_files[bin_index]

    await setup_system(dut)
    await wait_for_spi_ready(dut)
    await boot_core_over_spi(dut)

    monitor = await _classify_one_recording(dut, bin_path)
    result = await _readback_and_build_result(dut, bin_path, bin_index, monitor)

    _log_result(dut, result, prefix="SINGLE RESULT")
    _assert_result_ok(result)

    dut._log.info(
        f"PASS [{result['name']}]: first classification={_gesture_name(result['gesture'])}"
    )


@logged_test()
async def test_soc_boot_then_stream_all_bin_files(dut):
    if RUN_ALL_BIN_FILES == 0:
        dut._log.info("Skipping all-bin-files test. Set RUN_ALL_BIN_FILES=1 to enable.")
        return

    bin_files = _resolve_bin_files()
    summary = []
    failures = []

    for i, bin_path in enumerate(bin_files):
        dut._log.info("=" * 72)
        dut._log.info(f"RUN_ALL recording {i}: {Path(bin_path).name}")
        dut._log.info("=" * 72)

        try:
            if i == 0:
                await setup_system(dut)
            else:
                await reset_system_no_new_clock(dut)

            await wait_for_spi_ready(dut)
            await boot_core_over_spi(dut)

            monitor = await _classify_one_recording(dut, bin_path)
            result = await _readback_and_build_result(dut, bin_path, i, monitor)

            _log_result(dut, result, prefix="RUN_ALL RESULT")
            summary.append(result)

            if result["status"] != "PASS":
                failures.append(result)

        except Exception as exc:
            fail_result = {
                "name": Path(bin_path).name,
                "index": i,
                "expected": EXPECTED_BIN_FILE_CLASS.get(i),
                "class_gesture": "?",
                "class_pass": 0,
                "gesture": "?",
                "confidence": "?",
                "scores": [],
                "decision": {},
                "miso_word": 0,
                "expected_miso": 0,
                "miso_ok": False,
                "expected_ok": False,
                "status": "FAIL",
                "error": str(exc),
            }
            summary.append(fail_result)
            failures.append(fail_result)
            dut._log.error(f"RUN_ALL RESULT [{Path(bin_path).name}]: FAIL error={exc}")

    dut._log.info("=" * 72)
    dut._log.info("RUN_ALL FINAL SUMMARY")
    dut._log.info("=" * 72)

    for result in summary:
        if result.get("error"):
            dut._log.info(
                f"{result['status']:4s} {result['name']}: "
                f"expected={_gesture_name(result['expected'])} error={result['error']}"
            )
        else:
            dut._log.info(
                f"{result['status']:4s} {result['name']}: "
                f"expected={_gesture_name(result['expected'])} "
                f"got={_gesture_name(result['gesture'])} "
                f"conf={result['confidence']} "
                f"scores={_scores_str(result['scores'])} "
                f"MISO=0x{result['miso_word']:08X}"
            )

    assert not failures, (
        "RUN_ALL_BIN_FILES failures: "
        + ", ".join(f"{r['name']}({r.get('error', r['status'])})" for r in failures)
    )

    dut._log.info("PASS: all recordings classified by first valid result")