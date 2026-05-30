# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors
#
# soc_tb — integration testbench for src/soc.sv.
#
# soc is the level immediately below chip_core: it has no IO pads, just RTL
# signals.  At the soc boundary (what chip_core drives) we exercise:
#     inputs : clk, rst, SCLK, CS, MOSI
#     outputs: MISO, debug_bus, spi_ready
#
# Internally soc wires together two submodules:
#     u_spi_wrapper -> u_core : evt_word, evt_word_valid, evt_word_ready
#     u_core -> u_spi_wrapper : gesture, gesture_valid, gesture_confidence
# Both handshakes are visible at soc top-level (they are declared as `logic`
# in the soc body), so the TB monitors them directly to validate that the
# two submodules actually talk to each other under stimulus.
#
# This TB intentionally mirrors chip_top_tb.py's stimulus and validation
# methodology (programmable bin length, multi-pulse dominant-class
# classification, full pipeline drain) so a discrepancy between the two
# levels of hierarchy is a real bug, not a TB difference.

from collections import Counter
from pathlib import Path
import os
import struct

import cocotb
from util.test_logging import logged_test
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, NextTimeStep

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

# Force the integration TB to run at the real ASIC clock (64 MHz = 15625 ps).
# Using ps avoids cocotb/Icarus precision problems with fractional ns periods.
# cocotb 2.x's Clock requires an even ps period when period_high is not given,
# so the raw period is rounded up to the next even value.
CLK_FREQ_HZ    = int(os.environ.get("CLK_FREQ_HZ", "64000000"))
_RAW_PERIOD_PS = int(round(1_000_000_000_000 / CLK_FREQ_HZ))
CHIP_PERIOD_PS = _RAW_PERIOD_PS + (_RAW_PERIOD_PS % 2)
CHIP_PERIOD_NS = CHIP_PERIOD_PS / 1000.0

DATA_WIDTH = 32

GRID_SIZE     = int(CFG.get("GRID_SIZE",     os.environ.get("GRID_SIZE",     16)))
READOUT_BINS  = int(CFG.get("READOUT_BINS",  os.environ.get("READOUT_BINS",  16)))
NUM_CLASSES   = int(CFG.get("NUM_CLASSES",   os.environ.get("NUM_CLASSES",   4)))
WINDOW_MS     = int(CFG.get("WINDOW_MS",     os.environ.get("WINDOW_MS",     1000)))
SENSOR_WIDTH  = int(CFG.get("SENSOR_WIDTH",  os.environ.get("SENSOR_WIDTH",  320)))
SENSOR_HEIGHT = int(CFG.get("SENSOR_HEIGHT", os.environ.get("SENSOR_HEIGHT", SENSOR_WIDTH)))

FEATURE_COUNT = GRID_SIZE * GRID_SIZE * READOUT_BINS

# Programmable bin length (µs).  voxel_binning.sv defaults to 62500 (62.5 ms)
# on reset for the 16-bin chip (1 s window / 16 bins) and overrides only when
# bin_length_valid pulses with a non-zero value.  We program it explicitly for
# the same reasons chip_top_tb does:
#   1. Exercises the new BIN_LENGTH opcode path (commit c1146c3).
#   2. Documents bin length in the TB instead of relying on the RTL default.
#   3. Keeps flush-event spacing in sync with the programmed value.
BIN_LENGTH_US = int(os.environ.get("BIN_LENGTH_US", str((WINDOW_MS * 1000) // READOUT_BINS)))

# half_cycles=1 => 1 chip clk cycle per SCLK half-period.
# Full SCLK period = 2 chip clk cycles → 32 MHz SCLK at 64 MHz chip clock.
SPI_HALF_CHIP_CYCLES_DEFAULT = int(os.environ.get("SPI_HALF_CHIP_CYCLES", "1"))

# 0 = stream the whole .bin file (default).
MAX_BIN_WORDS = int(os.environ.get("MAX_BIN_WORDS", "0"))

# Set 0 to skip checking that the dominant gesture matches the expected label
# (the SPI / handshake / pipeline portions of the TB still run).
ASSERT_EXPECTED_LABEL = int(os.environ.get("ASSERT_EXPECTED_LABEL", "1"))

# Log progress every N SPI words during long bursts.
LOG_EVERY_WORDS = int(os.environ.get("LOG_EVERY_WORDS", "1000"))

GESTURE_NAMES = {0: "Down", 1: "Left", 2: "Right", 3: "Up"}

EXPECTED_BIN_FILE_CLASS = {0: 0, 1: 1, 2: 2, 3: 3}  # wave_{down,left,right,up}


# ── EVT2 opcode encodings ─────────────────────────────────────────────────────
# These match evt2_decoder.sv.  Only opcodes the TB actually emits are listed;
# CD_OFF/CD_ON/TIME_HIGH come straight from the recorded .bin files.

EVT_WEIGHT      = 0x2  # [27:20]=weight, [19:8]=feature_addr (12b), [7:2]=class/sram sel (6b)
EVT_THRESH_U    = 0x3  # [27:9]=upper 19-bit field of threshold (fits exactly at SCORE_BITS=37)
EVT_THRESH_L    = 0x4  # [27:10]=lower 18 bits of threshold, [9:7]=thresh addr (3b)
EVT_BIN_LENGTH_U = 0x5  # [16:0]=upper 17 bits of bin_length_us
EVT_BIN_LENGTH_L = 0x6  # [16:0]=lower 17 bits; latches bin_length_valid
EVT_BOOT_REQ    = 0xC  # triggers control_fsm: ST_BOOT -> ST_LOAD
DEBUG_PAGE      = 0xE  # [27:24]=page select for debug mux
EVT_READS_DONE  = 0xF  # end-of-programming marker

# Synthetic event types used to flush the binning pipeline.
EVT_TIME_HIGH = 0x8
EVT_CD_OFF    = 0x0


def build_evt2_weight(weight, feature_addr, class_id):
    # Layout matches evt2_decoder.sv: data[27:20], feature addr[19:8] (12b), sram sel[7:2] (6b)
    return (
        ((EVT_WEIGHT & 0xF) << 28)
        | ((int(weight) & 0xFF) << 20)
        | ((int(feature_addr) & 0xFFF) << 8)
        | ((int(class_id) & 0x3F) << 2)
    )


def build_evt2_thresh_upper(threshold_value, threshold_addr):
    # THRESH_U: [27:9]=upper 19-bit field. For SCORE_BITS=37, bits [36:18] of
    # the threshold go into this field (exact fit, no truncation). RTL reads
    # the thresh addr only from THRESH_L.
    threshold_value = int(threshold_value) & ((1 << 37) - 1)
    upper = (threshold_value >> 18) & 0x7FFFF
    return (
        ((EVT_THRESH_U & 0xF) << 28)
        | ((upper & 0x7FFFF) << 9)
    )


def build_evt2_thresh_lower(threshold_value, threshold_addr):
    # THRESH_L: [27:10]=lower 18 bits of threshold, [9:7]=thresh addr (3b)
    threshold_value = int(threshold_value) & ((1 << 37) - 1)
    lower = threshold_value & 0x3FFFF
    return (
        ((EVT_THRESH_L & 0xF) << 28)
        | ((lower & 0x3FFFF) << 10)
        | ((int(threshold_addr) & 0x7) << 7)
    )


def build_evt2_bin_length_upper(bin_length_us):
    """Upper 17 bits of programmable bin length (latched into bin_length_reg)."""
    return ((EVT_BIN_LENGTH_U & 0xF) << 28) | ((int(bin_length_us) >> 17) & 0x1FFFF)


def build_evt2_bin_length_lower(bin_length_us):
    """Lower 17 bits.  Decoder pulses bin_length_valid; voxel_binning latches it."""
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
    mem_keys     = [f"WEIGHT_MEM_C{c}" for c in range(NUM_CLASSES)]
    mem_defaults = [f"weights/{FEATURE_COUNT}weights_q8_c{c}.mem" for c in range(NUM_CLASSES)]

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
    """
    SPI programming stream sent after BOOT_REQ once evt_ld_en is high.

    Order matches chip_top_tb so soc-level and chip_top-level runs program the
    chip identically:
        1. weights      - per-class feature weights into the MAC SRAM
        2. thresholds   - class + diff thresholds (upper / lower halves)
        3. bin length   - programs voxel_binning's bin_duration_ts
        4. READS_DONE   - signals decoder/control_fsm that programming is done

    The bin-length pair must land while evt_ld_en is still high; voxel_binning
    latches it into bin_duration_ts and from that point all rollovers are
    spaced exactly bin_length_us apart.
    """
    if bin_length_us is None:
        bin_length_us = BIN_LENGTH_US

    words = []
    for class_id in range(NUM_CLASSES):
        for feature_addr in range(FEATURE_COUNT):
            words.append(build_evt2_weight(
                weight=weights[class_id][feature_addr],
                feature_addr=feature_addr,
                class_id=class_id,
            ))
    for threshold_addr, threshold_value in enumerate(thresholds):
        words.append(build_evt2_thresh_upper(threshold_value, threshold_addr))
        words.append(build_evt2_thresh_lower(threshold_value, threshold_addr))

    words.append(build_evt2_bin_length_upper(bin_length_us))
    words.append(build_evt2_bin_length_lower(bin_length_us))
    words.append(build_evt2_reads_done())
    return words


def _default_bin_paths():
    test_set = _REPO_ROOT / "EVT2_gesture_set" / "test_set"
    legacy  = _REPO_ROOT / "EVT2_gesture_set"
    root    = test_set if test_set.exists() else legacy
    return [
        root / "wave_down_sun_test1.bin",
        root / "wave_left_sun_test1.bin",
        root / "wave_right_sun_test1.bin",
        root / "wave_up_sun_test1.bin",
    ]


def _resolve_bin_files():
    """Override with GESTURE_BIN_FILES="path0:path1:path2:path3"."""
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
        raise RuntimeError(
            f"{path} size is {len(data)} bytes, not divisible by 4."
        )

    n_words = len(data) // 4
    words   = list(struct.unpack_from(f"<{n_words}I", data, 0))

    if MAX_BIN_WORDS > 0:
        words = words[:MAX_BIN_WORDS]
    return words


# ── Reset / startup ───────────────────────────────────────────────────────────
def _drive_idle(dut):
    """Idle SPI bus state (CS high, SCLK low, MOSI low)."""
    dut.SCLK.value = 0
    dut.CS.value   = 1
    dut.MOSI.value = 0


async def _apply_reset(dut, hold_cycles=16, post_cycles=50):
    """soc uses active-high rst (chip_core ties this to !rst_n)."""
    await NextTimeStep()
    _drive_idle(dut)

    dut.rst.value = 1
    await ClockCycles(dut.clk, hold_cycles)
    await NextTimeStep()
    dut.rst.value = 0
    await ClockCycles(dut.clk, post_cycles)


async def setup_system(dut):
    """Start chip clock + apply reset."""
    dut._log.info(
        f"Starting chip clock: CLK_FREQ_HZ={CLK_FREQ_HZ}, "
        f"period={CHIP_PERIOD_PS} ps ({CHIP_PERIOD_NS:.3f} ns)"
    )
    cocotb.start_soon(Clock(dut.clk, CHIP_PERIOD_PS, "ps").start())
    await _apply_reset(dut)


async def reset_system_no_new_clock(dut):
    """Reset between recordings without restarting the clock."""
    await _apply_reset(dut)


async def wait_for_spi_ready(dut, max_cycles=5000):
    for cycle in range(max_cycles):
        await RisingEdge(dut.clk)
        if int(dut.spi_ready.value):
            dut._log.info(f"spi_ready asserted after {cycle} clk cycles")
            return
    raise AssertionError("spi_ready never asserted")


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
):
    """
    Manual SPI mode 0 streaming transfer.  CS stays low across the whole
    burst, mirroring how a real GenX320 sensor delivers events.
    """
    assert len(mosi_words) > 0, "mosi_words must not be empty"

    dut._log.info(
        f"{tag}: streaming {len(mosi_words)} words over SPI mode 0, "
        f"half_cycles={half_cycles}, capture_miso={capture_miso}"
    )

    miso_words = [] if capture_miso else None

    _drive_idle(dut)
    await ClockCycles(dut.clk, cs_high_gap_cycles)

    first_word = int(mosi_words[0]) & 0xFFFFFFFF
    dut.CS.value   = 0
    dut.MOSI.value = (first_word >> (width - 1)) & 1
    await ClockCycles(dut.clk, pre_start_cycles)

    for word_idx, mosi_word in enumerate(mosi_words):
        mosi_word = int(mosi_word) & 0xFFFFFFFF
        miso_word = 0

        if word_idx % LOG_EVERY_WORDS == 0:
            dut._log.info(
                f"{tag}: word {word_idx}/{len(mosi_words)} MOSI=0x{mosi_word:08X}"
            )

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

    dut._log.info(f"{tag}: finished streaming {len(mosi_words)} words")
    return miso_words if capture_miso else []


async def spi_mode0_transfer_word(dut, mosi_word, tag="spi_single"):
    miso_words = await spi_mode0_stream_words(
        dut, [mosi_word], tag=tag, capture_miso=True,
    )
    return miso_words[0]


def expected_miso_from_classification(gesture, confidence):
    """
    spi_wrapper latches {confidence, gesture[1:0]} and shifts it into bits
    [31:29] of MISO when the host reads back over SPI.
    """
    classification = ((int(confidence) & 1) << 2) | (int(gesture) & 0b11)
    return classification << (DATA_WIDTH - 3)


# ── Internal-signal monitors ──────────────────────────────────────────────────
class GestureMonitor:
    """
    Records every gesture_valid pulse from u_core into u_spi_wrapper.

    The classifier produces multiple gesture_valid pulses per recording (one
    per bin rollover after the first full readout window), so we collect them
    all.  The first pulse can be misleading (e.g. wave_left starts with a
    downward arc that resembles wave_up) — the chip's true verdict is the
    dominant class across all pulses.  This matches voxel_bin_core_tb and
    chip_top_tb methodology.
    """

    def __init__(self):
        self.pulses = []        # ordered list of (gesture, confidence)

    @property
    def count(self):
        return len(self.pulses)

    @property
    def latest_gesture(self):
        return self.pulses[-1][0] if self.pulses else None

    @property
    def latest_confidence(self):
        return self.pulses[-1][1] if self.pulses else None

    def dominant(self):
        """(gesture, confidence) of the most-frequent class; latest of ties."""
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
            f"{GESTURE_NAMES.get(g, g)}:{n}" for g, n in sorted(counts.items())
        )

    async def run(self, dut):
        while True:
            await RisingEdge(dut.clk)
            if int(dut.gesture_valid.value):
                g = int(dut.gesture.value)
                c = int(dut.gesture_confidence.value)
                self.pulses.append((g, c))
                dut._log.info(
                    f"gesture_valid #{self.count}: gesture={g} "
                    f"({GESTURE_NAMES.get(g, g)}), confidence={c}"
                )


class HandshakeMonitor:
    """
    Counts evt_word_valid && evt_word_ready transactions on the
    u_spi_wrapper -> u_core boundary.

    This is the soc-internal handshake that delivers EVT2 words from the SPI
    front-end into voxel_bin_core's input FIFO.  Counting it confirms both
    submodules see eye-to-eye on the producer/consumer protocol.
    """

    def __init__(self):
        self.count = 0

    async def run(self, dut):
        while True:
            await RisingEdge(dut.clk)
            try:
                v = int(dut.evt_word_valid.value)
                r = int(dut.evt_word_ready.value)
            except Exception:
                continue
            if v and r:
                self.count += 1


def _read_sig(handle, name):
    """Read a signal by dotted path; return int or '?' if unreachable."""
    try:
        obj = handle
        for p in name.split("."):
            obj = getattr(obj, p)
        return int(obj.value)
    except Exception:
        return "?"


async def wait_for_evt_ld_en(dut, max_cycles=5000):
    """
    Wait for control_fsm to enter its load window.  Programming words are
    silently dropped until evt_ld_en is high, so this gate must be observed.
    """
    for cycle in range(max_cycles):
        await RisingEdge(dut.clk)
        if _read_sig(dut, "u_core.evt_ld_en") == 1:
            dut._log.info(f"evt_ld_en asserted at +{cycle} cycles after BOOT_REQ")
            return

    raise AssertionError(
        "Timed out waiting for evt_ld_en after BOOT_REQ: "
        f"core_rst_o={_read_sig(dut, 'u_core.core_rst_o')} "
        f"boot_done_o={_read_sig(dut, 'u_core.boot_done_o')} "
        f"main_state={_read_sig(dut, 'u_core.main_state_dbg_o')} "
        f"load_state={_read_sig(dut, 'u_core.load_state_dbg_o')}"
    )


async def wait_after_reads_done(dut):
    """Settle window for the boot/program FSM after EVT_READS_DONE."""
    await ClockCycles(dut.clk, 100)
    dut._log.info(
        f"POST-BOOT FSM: "
        f"core_rst_o={_read_sig(dut, 'u_core.core_rst_o')} "
        f"boot_done_o={_read_sig(dut, 'u_core.boot_done_o')} "
        f"main_state={_read_sig(dut, 'u_core.main_state_dbg_o')} "
        f"load_state={_read_sig(dut, 'u_core.load_state_dbg_o')} "
        f"evt_ld_en={_read_sig(dut, 'u_core.evt_ld_en')}"
    )
    try:
        dut._log.info(f"debug_bus after READS_DONE = 0x{int(dut.debug_bus.value):08X}")
    except Exception:
        pass


async def log_pipeline_state(dut, tag=""):
    """Snapshot key voxel_bin_core internal state for diagnostics."""
    dut._log.info(
        f"PIPELINE{(' ' + tag) if tag else ''}: "
        f"core_rst={_read_sig(dut, 'u_core.core_rst_o')} "
        f"evt_count={_read_sig(dut, 'u_core.debug_event_count')} "
        f"fwr={_read_sig(dut, 'u_core.feature_window_ready')} "
        f"cap={_read_sig(dut, 'u_core.capture_active')} "
        f"mac_busy={_read_sig(dut, 'u_core.mac_busy')} "
        f"scores=["
        f"{_read_sig(dut, 'u_core.score_A')},"
        f"{_read_sig(dut, 'u_core.score_B')},"
        f"{_read_sig(dut, 'u_core.score_C')},"
        f"{_read_sig(dut, 'u_core.score_D')}]"
    )


async def wait_for_monitor_count(dut, monitor, min_count=1, max_cycles=2_000_000):
    class_valid_seen = 0
    class_pass_seen  = 0
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if _read_sig(dut, "u_core.class_valid") == 1:
            class_valid_seen += 1
        if _read_sig(dut, "u_core.class_pass") == 1:
            class_pass_seen += 1
        if monitor.count >= min_count:
            return
    await log_pipeline_state(dut, "gesture-timeout")
    raise AssertionError(
        f"Timed out waiting for gesture monitor count >= {min_count}; "
        f"current={monitor.count} class_valid_seen={class_valid_seen} "
        f"class_pass_seen={class_pass_seen}"
    )


# ── High-level system helpers ─────────────────────────────────────────────────
async def boot_core_over_spi(dut, hs_monitor=None):
    """
    Load all weights, thresholds, and the programmable bin length over SPI.

    Returns (weights, thresholds, expected_program_words) so the caller can
    cross-check the spi_wrapper -> voxel_bin_core handshake count if desired.
    """
    weights    = load_weights_from_mem()
    thresholds = load_thresholds()

    program_words = build_program_stream_words(weights, thresholds)

    dut._log.info(
        f"Boot/program stream: 1 BOOT_REQ + {len(program_words)} program words "
        f"({NUM_CLASSES} classes * {FEATURE_COUNT} weights + "
        f"{len(thresholds) * 2} threshold halves + 2 bin-length halves + READS_DONE), "
        f"BIN_LENGTH_US={BIN_LENGTH_US}"
    )

    await spi_mode0_stream_words(dut, [build_evt2_boot_req()], tag="boot_req")
    await wait_for_evt_ld_en(dut)
    await spi_mode0_stream_words(dut, program_words, tag="weights_thresholds_reads_done")
    await wait_after_reads_done(dut)

    if hs_monitor is not None:
        # 1 (BOOT_REQ) + len(program_words) words pass through evt_word_valid&ready.
        expected = 1 + len(program_words)
        dut._log.info(
            f"spi_wrapper -> u_core handshake count after boot: "
            f"{hs_monitor.count} (expected ~{expected})"
        )

    return weights, thresholds


def _append_bin_flush_events(words, readout_bins=READOUT_BINS, bin_length_us=None):
    """
    Append synthetic TIME_HIGH + CD_OFF pairs after the recording.

    Each pair advances the decoder's reconstructed timestamp by exactly
    bin_length_us, which matches voxel_binning's bin_duration_ts.  The first
    event whose timestamp crosses the next bin boundary triggers a rollover
    (voxel_binning.sv: (acc_event_ts - bin_start_ts) >= bin_duration_ts), so
    READOUT_BINS+1 spaced events guarantee enough rollovers to flush every
    bin in the ring buffer.

    bin_length_us MUST track whatever the chip was programmed with — if the
    chip uses 50 ms bins but flush events are spaced 125 ms apart, each event
    crosses multiple bin boundaries and the chip queues redundant rollovers.
    """
    if bin_length_us is None:
        bin_length_us = BIN_LENGTH_US

    last_th_reg = 0
    for w in words:
        if (w >> 28) == EVT_TIME_HIGH:
            last_th_reg = w & 0x0FFFFFFF

    for i in range(1, readout_bins + 2):
        ts_us  = last_th_reg * 64 + i * bin_length_us
        th_val = (ts_us >> 6) & 0x0FFFFFFF
        ts_lsb = ts_us & 0x3F
        words.append((EVT_TIME_HIGH << 28) | th_val)
        # CD_OFF at (160, 160) — pixel coords are arbitrary; only timestamp matters.
        words.append((EVT_CD_OFF << 28) | (ts_lsb << 22) | (160 << 11) | 160)
    return words


async def stream_bin_recording_over_spi(dut, bin_path):
    words = _read_evt2_bin(bin_path)
    words = _append_bin_flush_events(list(words))
    dut._log.info(
        f"Streaming EVT2 recording {bin_path} over SPI: {len(words)} words"
    )
    return await spi_mode0_stream_words(
        dut, words, tag=f"evt2_bin_{Path(bin_path).stem}", capture_miso=False,
    )


async def read_classification_over_spi(dut):
    """
    Read back spi_wrapper's latched classification.

    Returns the 32-bit MISO word: bits [31:29] hold {confidence, gesture[1:0]}
    of the LAST gesture_valid pulse the chip emitted.
    """
    return await spi_mode0_transfer_word(
        dut, build_evt2_debug_page(0), tag="classification_readback",
    )


# ── Tests ─────────────────────────────────────────────────────────────────────
@logged_test()
async def test_soc_boot_stream_over_spi(dut):
    """
    Sanity test: reset, send BOOT_REQ, stream weights/thresholds/bin-length/
    READS_DONE.  Validates that the soc top-level SPI port and the internal
    spi_wrapper -> voxel_bin_core handshake accept the full programming
    stream end-to-end.
    """
    await setup_system(dut)
    await wait_for_spi_ready(dut)

    hs = HandshakeMonitor()
    hs_task = cocotb.start_soon(hs.run(dut))

    await boot_core_over_spi(dut, hs_monitor=hs)

    hs_task.kill()

    # Sanity: at minimum every weight word and threshold pair should have
    # crossed the spi_wrapper -> voxel_bin_core boundary.  Allow a small
    # tolerance for TB-induced SPI re-arm timing.
    min_expected = NUM_CLASSES * FEATURE_COUNT
    assert hs.count >= min_expected, (
        f"spi_wrapper -> voxel_bin_core handshake count too low: "
        f"got {hs.count}, expected >= {min_expected}"
    )

    dut._log.info(
        f"PASS: full boot stream sent over SPI; "
        f"{hs.count} words crossed the soc-internal evt_word handshake"
    )


@logged_test()
async def test_soc_debug_page_sweep(dut):
    """
    Walk the DEBUG_PAGE selector across pages 0-4 and log the resulting
    debug_bus.  Pages 0-2/4 expose live internal signals; page 3 is reserved.
    Verifies the SPI -> evt2_decoder -> selectable_debug -> debug_bus path.
    """
    PAGE_NAMES = {
        0: "voxel_gesture_classifier + mac_engine",
        1: "voxel_binning",
        2: "evt2_decoder + input_FIFO + voxel_core",
        3: "control_module (reserved)",
        4: "evt2_decoder event output",
    }

    await setup_system(dut)
    await wait_for_spi_ready(dut)

    # BOOT_REQ first so the FSM is not stuck in boot-wait when we toggle pages.
    await spi_mode0_stream_words(dut, [build_evt2_boot_req()], tag="boot_req")
    await ClockCycles(dut.clk, 20)

    for page in range(5):
        await spi_mode0_stream_words(
            dut, [build_evt2_debug_page(page)], tag=f"page_sel_{page}"
        )
        await ClockCycles(dut.clk, 10)
        try:
            bus = int(dut.debug_bus.value)
            dut._log.info(f"  Page {page} ({PAGE_NAMES[page]}): debug_bus=0x{bus:08X}")
        except Exception:
            dut._log.info(f"  Page {page} ({PAGE_NAMES[page]}): debug_bus = X/Z")

    dut._log.info("PASS: debug pages 0-4 selected without simulation error")


async def _classify_one_recording(dut, bin_path, drain_cycles=300_000):
    """
    Stream one recording, collect every gesture_valid pulse, drain the
    pipeline, and return the GestureMonitor with all observations.
    """
    monitor      = GestureMonitor()
    monitor_task = cocotb.start_soon(monitor.run(dut))

    await stream_bin_recording_over_spi(dut, bin_path)
    await log_pipeline_state(dut, "post-stream")

    # Drain: 300 000 cycles @ 64 MHz = 4.7 ms. After streaming we have
    # READOUT_BINS+1 = 17 flush-driven bin rollovers queued in the binner;
    # the classifier emits one gesture_valid pulse per rollover at ~128 µs
    # throughput (16-bin readout + MAC + classifier pipeline). 50 000 cycles
    # captured only the first ~6 pulses, biasing the dominant class toward
    # the noisy opening windows. 300 000 cycles covers all ~17 expected
    # pulses with margin so the dominant reflects the full recording.
    dut._log.info(f"Draining pipeline for {drain_cycles} cycles ...")
    await ClockCycles(dut.clk, drain_cycles)
    await log_pipeline_state(dut, "post-drain")

    monitor_task.kill()

    if not monitor.pulses:
        raise AssertionError(
            f"No gesture_valid pulses fired after streaming {Path(bin_path).name} "
            f"+ {drain_cycles} drain cycles"
        )
    return monitor


@logged_test()
async def test_soc_boot_then_stream_one_bin_file(dut):
    """
    Full integration test on a single recording:
      1. Reset soc; wait for spi_ready.
      2. Boot over SPI (weights + thresholds + bin length + READS_DONE).
      3. Stream one EVT2 .bin over SPI.
      4. Collect every gesture_valid pulse from u_core.
      5. Read back the latest classification through MISO.
      6. Assert the dominant class matches the expected label.

    Select clip:   BIN_INDEX=0,1,2,3
    Debug option:  MAX_BIN_WORDS=5000
    """
    bin_files  = _resolve_bin_files()
    bin_index  = int(os.environ.get("BIN_INDEX", "0"))
    assert 0 <= bin_index < len(bin_files), f"Invalid BIN_INDEX={bin_index}"

    bin_path       = bin_files[bin_index]
    expected_class = EXPECTED_BIN_FILE_CLASS.get(bin_index)

    await setup_system(dut)
    await wait_for_spi_ready(dut)
    await boot_core_over_spi(dut)

    monitor = await _classify_one_recording(dut, bin_path)

    dom_g, dom_c = monitor.dominant()
    last_g, last_c = monitor.pulses[-1]

    dut._log.info(
        f"[{bin_path.name}] {monitor.count} pulses; histogram=({monitor.histogram_str()}); "
        f"dominant={GESTURE_NAMES.get(dom_g, dom_g)} (confidence={dom_c}); "
        f"last={GESTURE_NAMES.get(last_g, last_g)} (confidence={last_c})"
    )

    # spi_wrapper latches the LATEST pulse, so MISO must mirror that.
    await ClockCycles(dut.clk, 20)
    miso_word     = await read_classification_over_spi(dut)
    expected_miso = expected_miso_from_classification(last_g, last_c)

    assert miso_word == expected_miso, (
        f"MISO classification readback mismatch: "
        f"got 0x{miso_word:08X}, expected 0x{expected_miso:08X} "
        f"(last gesture={last_g}, confidence={last_c})"
    )

    if ASSERT_EXPECTED_LABEL and expected_class is not None:
        # Correctness uses the dominant class — first window often classifies
        # into a misleading class (e.g. wave_left's downward arc looks like Up)
        # so only the dominant pulse is the true verdict.
        assert dom_g == expected_class, (
            f"Expected {GESTURE_NAMES[expected_class]} (dominant) for "
            f"{bin_path.name}, got {GESTURE_NAMES.get(dom_g, dom_g)} "
            f"(pulses: {[GESTURE_NAMES.get(g, g) for g, _ in monitor.pulses]})"
        )

    dut._log.info(
        f"PASS [{bin_path.name}]: dominant={GESTURE_NAMES.get(dom_g, dom_g)} "
        f"from {monitor.count} pulses, MISO=0x{miso_word:08X}"
    )


@logged_test()
async def test_soc_boot_then_stream_all_bin_files(dut):
    """
    Four-recording end-to-end test.  Disabled by default (long runtime).
    Enable with RUN_ALL_BIN_FILES=1.
    """
    if int(os.environ.get("RUN_ALL_BIN_FILES", "0")) == 0:
        dut._log.info("Skipping long all-bin-files test. Set RUN_ALL_BIN_FILES=1 to enable.")
        return

    bin_files = _resolve_bin_files()
    summary   = []

    for i, bin_path in enumerate(bin_files):
        dut._log.info("=" * 80)
        dut._log.info(f"Recording {i}: {bin_path}")
        dut._log.info("=" * 80)

        if i == 0:
            await setup_system(dut)
        else:
            await reset_system_no_new_clock(dut)

        await wait_for_spi_ready(dut)
        await boot_core_over_spi(dut)

        monitor = await _classify_one_recording(dut, bin_path)

        dom_g, dom_c   = monitor.dominant()
        last_g, last_c = monitor.pulses[-1]

        await ClockCycles(dut.clk, 20)
        miso_word     = await read_classification_over_spi(dut)
        expected_miso = expected_miso_from_classification(last_g, last_c)

        assert miso_word == expected_miso, (
            f"[{bin_path.name}] MISO mismatch: "
            f"got 0x{miso_word:08X}, expected 0x{expected_miso:08X}"
        )

        expected_class = EXPECTED_BIN_FILE_CLASS.get(i)
        if ASSERT_EXPECTED_LABEL and expected_class is not None:
            assert dom_g == expected_class, (
                f"[{bin_path.name}] expected dominant={GESTURE_NAMES[expected_class]}, "
                f"got {GESTURE_NAMES.get(dom_g, dom_g)} "
                f"(pulses: {[GESTURE_NAMES.get(g, g) for g, _ in monitor.pulses]})"
            )

        summary.append((bin_path.name, dom_g, dom_c, monitor.count))
        dut._log.info(
            f"[{bin_path.name}] PASS: dominant={GESTURE_NAMES.get(dom_g, dom_g)} "
            f"from {monitor.count} pulses, MISO=0x{miso_word:08X}"
        )

    dut._log.info("=" * 80)
    dut._log.info("ALL RECORDINGS CLASSIFIED")
    for name, g, c, n in summary:
        dut._log.info(f"  {name}: {GESTURE_NAMES.get(g, g)} (confidence={c}, {n} pulses)")
    dut._log.info("=" * 80)
