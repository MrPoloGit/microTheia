#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors
"""
Patch an OpenROAD-emitted SDF so Icarus Verilog 13 can consume it without
fatal errors or VPI assertion crashes.

Three specific Icarus 13 limitations bite the raw SDF that
`librelane/runs/<RUN>/54-openroad-stapostpnr/<corner>/chip_top__<corner>.sdf`
emits:

  1. Header triplets in (VOLTAGE) / (TEMPERATURE) / (PROCESS) use the
     SDF v3.0 shorthand `max::min` (typical value omitted). Icarus's
     selector defaults to the typ value and aborts with
     "Chosen value not defined" before it even reaches the cell entries.
     We fix this by promoting `max::min` to `max:min:min`.

  2. TIMINGCHECK sections are parsed but not implemented by Icarus 13, which
     floods the log with one warning per SETUP/HOLD/WIDTH entry. These checks
     are not enforced by vvp, so dropping the blocks is equivalent to keeping
     them for simulation behavior.

  3. INTERCONNECT entries that reference IO pad/SRAM instances with escaped
     names (e.g. `analog\\[1\\]\\.pad.ASIG5V`) crash vvp:
        SDF ERROR: ...: Submodule analog[1] in port path not found!
        SDF ERROR: ...: Submodule pad in port path not found!
        ERROR: NULL handle passed to vpi_scan.
        vvp: vpi_iter.cc:71: assertion '0' failed.
     Icarus's SDF path splitter doesn't honour the SDF v3.0 backslash
     escapes (`\\[`, `\\]`, `\\.`) inside the scope component of an
     INTERCONNECT path. It also cannot bind the zero-delay top-level
     clk/rst port-to-pad INTERCONNECT entries (`clk_PAD clk_pad.PAD`,
     `rst_n_PAD rst_n_pad.PAD`), even though the pad instances exist.

  4. CELL blocks for flattened instance names containing escaped dots or
     brackets (e.g. `bidir\\[0\\]\\.pad` or `i_chip_core\\.u_soc...`) hit
     the same parser limitation:
        SDF ERROR: ...: Cannot find bidir[0] in scope ...
        SDF ERROR: ...: Cannot find i_chip_core in scope ...
     Icarus treats the escaped dot as a hierarchy separator and cannot match
     the Verilog escaped identifier. There is no alternate SDF spelling that
     resolves these names, so we drop only those CELL blocks.

Flat std-cell delay annotations (IOPATHs and conditional delays whose
instances are named `_12345_` or similar) are left untouched. Those are the
bulk of the gate timing data and Icarus can annotate them once the wrapper
specify blocks are enabled.

Usage:
    sdf_fix_for_icarus.py <input.sdf> <output.sdf>
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


# Match the header triplet entries that use the shorthand max::min.
# Examples seen in OpenROAD output:
#   (VOLTAGE 3.300::3.300)
#   (TEMPERATURE 25.000::25.000)
#   (PROCESS "1.000::1.000")
# The PROCESS form quotes the entire payload; VOLTAGE / TEMPERATURE don't.
# We expand max::min  ->  max:min:min so the typ slot (which Icarus reads
# by default) is no longer empty.
_TRIPLET_RE = re.compile(
    r'\(\s*(VOLTAGE|TEMPERATURE|PROCESS)\s+'
    r'(?:"\s*([\-\d.]+)\s*::\s*([\-\d.]+)\s*"'   # quoted form
    r'|([\-\d.]+)\s*::\s*([\-\d.]+))'             # bare form
    r'\s*\)'
)

_TOP_PAD_INTERCONNECT_RE = re.compile(
    r'^\s*\(INTERCONNECT\s+\S+_PAD(?:\[\d+\])?\s+\w+_pad\.PAD\s+'
)


def _fix_triplet(m: re.Match) -> str:
    keyword = m.group(1)
    if m.group(2) is not None:                   # quoted PROCESS form
        vmax, vmin = m.group(2), m.group(3)
        return f'({keyword} "{vmax}:{vmin}:{vmin}")'
    vmax, vmin = m.group(4), m.group(5)
    return f'({keyword} {vmax}:{vmin}:{vmin})'


def _is_unannotatable_interconnect(line: str) -> bool:
    """
    True if `line` is an INTERCONNECT entry whose source or destination
    path contains an escaped bracket or dot. These crash vvp on Icarus 13.
    The same INTERCONNECT entries are also all 0-delay in our flow, so
    dropping them is functionally identical to keeping them.
    """
    stripped = line.lstrip()
    if not stripped.startswith('(INTERCONNECT'):
        return False
    return (
        '\\[' in line
        or '\\.' in line
        or _TOP_PAD_INTERCONNECT_RE.match(line) is not None
    )


def _paren_delta(line: str) -> int:
    """Return paren balance delta for SDF text outside string literals."""
    delta = 0
    in_string = False
    escaped = False
    for char in line:
        if in_string:
            if escaped:
                escaped = False
            elif char == '\\':
                escaped = True
            elif char == '"':
                in_string = False
            continue

        if char == '"':
            in_string = True
        elif char == '(':
            delta += 1
        elif char == ')':
            delta -= 1
    return delta


def _cell_has_unannotatable_instance(lines: list[str]) -> bool:
    for line in lines:
        stripped = line.lstrip()
        if stripped.startswith('(INSTANCE'):
            return '\\[' in line or '\\.' in line
    return False


def _drop_unannotatable_cell_blocks(text: str) -> tuple[str, int]:
    out_lines: list[str] = []
    cell_lines: list[str] = []
    cell_depth = 0
    cells_dropped = 0

    for line in text.splitlines(keepends=True):
        if not cell_lines and line.lstrip().startswith('(CELL'):
            cell_lines = [line]
            cell_depth = _paren_delta(line)
            if cell_depth <= 0:
                if _cell_has_unannotatable_instance(cell_lines):
                    cells_dropped += 1
                else:
                    out_lines.extend(cell_lines)
                cell_lines = []
                cell_depth = 0
            continue

        if cell_lines:
            cell_lines.append(line)
            cell_depth += _paren_delta(line)
            if cell_depth <= 0:
                if _cell_has_unannotatable_instance(cell_lines):
                    cells_dropped += 1
                else:
                    out_lines.extend(cell_lines)
                cell_lines = []
                cell_depth = 0
            continue

        out_lines.append(line)

    if cell_lines:
        # Preserve malformed/truncated input for downstream diagnostics rather
        # than silently deleting the tail of the file.
        out_lines.extend(cell_lines)

    return ''.join(out_lines), cells_dropped


def _drop_timingcheck_blocks(text: str) -> tuple[str, int]:
    out_lines: list[str] = []
    block_lines: list[str] = []
    block_depth = 0
    blocks_dropped = 0

    for line in text.splitlines(keepends=True):
        if not block_lines and line.lstrip().startswith('(TIMINGCHECK'):
            block_lines = [line]
            block_depth = _paren_delta(line)
            if block_depth <= 0:
                blocks_dropped += 1
                block_lines = []
                block_depth = 0
            continue

        if block_lines:
            block_lines.append(line)
            block_depth += _paren_delta(line)
            if block_depth <= 0:
                blocks_dropped += 1
                block_lines = []
                block_depth = 0
            continue

        out_lines.append(line)

    if block_lines:
        # Preserve malformed/truncated input for downstream diagnostics rather
        # than silently deleting the tail of the file.
        out_lines.extend(block_lines)

    return ''.join(out_lines), blocks_dropped


def fix(src: Path, dst: Path) -> tuple[int, int, int, int]:
    """Return (triplets_fixed, interconnects_dropped, cells_dropped, timingchecks_dropped)."""
    text = src.read_text()

    new_text, triplets_fixed = _TRIPLET_RE.subn(_fix_triplet, text)
    new_text, timingchecks_dropped = _drop_timingcheck_blocks(new_text)

    out_lines: list[str] = []
    interconnects_dropped = 0
    for line in new_text.splitlines(keepends=True):
        if _is_unannotatable_interconnect(line):
            interconnects_dropped += 1
            continue
        out_lines.append(line)

    new_text, cells_dropped = _drop_unannotatable_cell_blocks(''.join(out_lines))

    dst.write_text(new_text)
    return triplets_fixed, interconnects_dropped, cells_dropped, timingchecks_dropped


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(__doc__, file=sys.stderr)
        return 1

    src = Path(argv[1])
    dst = Path(argv[2])

    if not src.exists():
        print(f'error: input SDF not found: {src}', file=sys.stderr)
        return 2

    dst.parent.mkdir(parents=True, exist_ok=True)
    triplets, interconnects, cells, timingchecks = fix(src, dst)
    print(
        f'[sdf_fix] {src.name}: fixed {triplets} triplets, '
        f'dropped {timingchecks} unsupported TIMINGCHECK blocks, '
        f'dropped {interconnects} unannotatable INTERCONNECT entries, '
        f'dropped {cells} unannotatable CELL blocks  ->  {dst}'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
