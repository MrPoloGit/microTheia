// SPDX-License-Identifier: Apache-2.0
//
// Simulator-friendly replacements for the gf180mcu_fd_io IO pad cells used
// by the chip_top netlist. The upstream models in
//   gf180mcu/gf180mcuD/libs.ref/gf180mcu_fd_io/verilog/gf180mcu_fd_io.v
// use Verilog-1995 `rnmos` primitives for the pull-up / pull-down resistors,
// which Verilator does not support. Drop them — for GLS we only need:
//   - Y reflects PAD (gated by IE on the bidir cell).
//   - PAD is driven by A when OE=1; high-Z otherwise.
//   - PU/PD/CS/SL ignored (no pull effect needed for our pin-level checks;
//     functional simulation behaves the same whether PAD floats or is
//     actively pulled, since the testbench only checks driven values).
//
// Module names and port lists match the upstream cells exactly so they slot
// in as drop-in replacements (the netlist binds by module name).

`timescale 1ns/1ps
`ifndef GF180MCU_FD_IO_V_SIM
`define GF180MCU_FD_IO_V_SIM

// Normal CMOS input pad.
// PAD is declared `input` (not `inout`): these pads are input-only at the
// chip level, and Verilator does not propagate cocotb writes through an
// `inout` port with no internal driver — leaving the chip's internal
// post-pad signals stuck at 0. Declaring `input` makes Verilator treat the
// port as a straightforward one-way wire so cocotb writes reach the chip.
module gf180mcu_fd_io__in_c (PU, PD, PAD, Y, DVDD, DVSS, VDD, VSS);
    input  PU, PD;
    input  PAD;
    output Y;
    inout  DVDD, DVSS, VDD, VSS;
    assign Y = PAD;
endmodule

// Schmitt-trigger input pad (same input-only fix as in_c)
module gf180mcu_fd_io__in_s (PU, PD, PAD, Y, DVDD, DVSS, VDD, VSS);
    input  PU, PD;
    input  PAD;
    output Y;
    inout  DVDD, DVSS, VDD, VSS;
    assign Y = PAD;
endmodule

// Bidirectional 24mA pad
module gf180mcu_fd_io__bi_24t (CS, SL, IE, OE, PU, PD, A, PAD, Y, DVDD, DVSS, VDD, VSS);
    input  CS, SL, IE, OE, PU, PD, A;
    inout  PAD;
    output Y;
    inout  DVDD, DVSS, VDD, VSS;
    assign PAD = OE ? A : 1'bz;
    assign Y   = IE ? PAD : 1'b0;
endmodule

// Analog signal pad — pure passthrough
module gf180mcu_fd_io__asig_5p0 (ASIG5V, DVDD, DVSS, VDD, VSS);
    inout ASIG5V;
    inout DVDD, DVSS, VDD, VSS;
endmodule

`endif
