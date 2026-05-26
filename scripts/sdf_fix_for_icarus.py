#!/usr/bin/env python3
"""
Patch an OpenROAD-emitted SDF so Icarus Verilog 13 can consume it without
errors or VPI assertion crashes.

Two specific Icarus 13 limitations bite the raw SDF that
`librelane/runs/<RUN>/54-openroad-stapostpnr/<corner>/chip_top__<corner>.sdf`
emits:

  1. Header triplets in (VOLTAGE) / (TEMPERATURE) / (PROCESS) use the
     SDF v3.0 shorthand `max::min` (typical value omitted). Icarus's
     selector defaults to the typ value and aborts with
     "Chosen value not defined" before it even reaches the cell entries.
     We fix this by promoting `max::min` to `max:min:min`.

  2. Top-level INTERCONNECT entries that reference IO pad instances with
     escaped names (e.g. `analog\\[1\\]\\.pad.ASIG5V`) crash vvp:
        SDF ERROR: ...: Submodule analog[1] in port path not found!
        SDF ERROR: ...: Submodule pad in port path not found!
        ERROR: NULL handle passed to vpi_scan.
        vvp: vpi_iter.cc:71: assertion '0' failed.
     Icarus's SDF path splitter doesn't honour the SDF v3.0 backslash
     escapes (`\\[`, `\\]`, `\\.`) inside the scope component of an
     INTERCONNECT path. It splits on the unescaped `.`, descends a
     `analog[1]` vector that doesn't exist, then asserts. There is no
     way to recover at runtime, so we drop these entries here. Every
     INTERCONNECT delay in the section is 0.000 ns anyway (these are
     the top-level routing wires from the chip_top port to the IO pad
     PAD inout, which the STA tool reports as zero-length nets), so the
     simulation loses nothing of substance.

Cell-level annotations (std-cell IOPATHs, SRAM IOPATHs, IO pad IOPATHs,
TIMINGCHECKs, conditional delays) are left untouched — they are the
real timing data and Icarus handles them correctly once the wrapper
specify-blocks added in commit "STA GLS specify support" are in place.

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
    return '\\[' in line or '\\.' in line


def fix(src: Path, dst: Path) -> tuple[int, int]:
    """Return (triplets_fixed, interconnects_dropped)."""
    text = src.read_text()

    new_text, triplets_fixed = _TRIPLET_RE.subn(_fix_triplet, text)

    out_lines: list[str] = []
    interconnects_dropped = 0
    for line in new_text.splitlines(keepends=True):
        if _is_unannotatable_interconnect(line):
            interconnects_dropped += 1
            continue
        out_lines.append(line)

    dst.write_text(''.join(out_lines))
    return triplets_fixed, interconnects_dropped


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
    triplets, interconnects = fix(src, dst)
    print(
        f'[sdf_fix] {src.name}: fixed {triplets} triplets, '
        f'dropped {interconnects} unannotatable INTERCONNECT entries  ->  {dst}'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
