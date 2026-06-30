// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
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

// Normal CMOS input pad. PAD declared `inout` to match the upstream PDK
// signature and chip_top's port direction. Y = PAD passes through.
module gf180mcu_fd_io__in_c (PU, PD, PAD, Y, DVDD, DVSS, VDD, VSS);
    input  PU, PD;
    inout  PAD;
    output Y;
    inout  DVDD, DVSS, VDD, VSS;
    assign Y = PAD;
`ifndef FUNCTIONAL
    specify
        (PAD => Y) = (0:0:0, 0:0:0);
    endspecify
`endif
endmodule

// Schmitt-trigger input pad
module gf180mcu_fd_io__in_s (PU, PD, PAD, Y, DVDD, DVSS, VDD, VSS);
    input  PU, PD;
    inout  PAD;
    output Y;
    inout  DVDD, DVSS, VDD, VSS;
    assign Y = PAD;
`ifndef FUNCTIONAL
    specify
        (PAD => Y) = (0:0:0, 0:0:0);
    endspecify
`endif
endmodule

// Bidirectional 24mA pad
module gf180mcu_fd_io__bi_24t (CS, SL, IE, OE, PU, PD, A, PAD, Y, DVDD, DVSS, VDD, VSS);
    input  CS, SL, IE, OE, PU, PD, A;
    inout  PAD;
    output Y;
    inout  DVDD, DVSS, VDD, VSS;
    assign PAD = OE ? A : 1'bz;
    assign Y   = IE ? PAD : 1'b0;
`ifndef FUNCTIONAL
    specify
        (PAD => Y)  = (0:0:0, 0:0:0);
        (A   => PAD) = (0:0:0, 0:0:0);
        (OE  => PAD) = (0:0:0, 0:0:0);
        (IE  => Y)   = (0:0:0, 0:0:0);
    endspecify
`endif
endmodule

// Analog signal pad — pure passthrough
module gf180mcu_fd_io__asig_5p0 (ASIG5V, DVDD, DVSS, VDD, VSS);
    inout ASIG5V;
    inout DVDD, DVSS, VDD, VSS;
endmodule

`endif

`ifndef GF180MCU_FD_IO_FILLER_CORNER_MISSING_MODELS
`define GF180MCU_FD_IO_FILLER_CORNER_MISSING_MODELS

module gf180mcu_fd_io__cor (
    inout DVDD,
    inout DVSS,
    inout VDD,
    inout VSS
);
endmodule

module gf180mcu_fd_io__fill10 (
    inout DVDD,
    inout DVSS,
    inout VDD,
    inout VSS
);
endmodule

module gf180mcu_fd_io__fill5 (
    inout DVDD,
    inout DVSS,
    inout VDD,
    inout VSS
);
endmodule

module gf180mcu_fd_io__fill1 (
    inout DVDD,
    inout DVSS,
    inout VDD,
    inout VSS
);
endmodule

module gf180mcu_fd_io__fillnc (
    inout DVDD,
    inout DVSS,
    inout VDD,
    inout VSS
);
endmodule

`endif
