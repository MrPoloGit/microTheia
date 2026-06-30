// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
//
// Simulator-friendly replacements for the gf180mcu_ws_io power pad cells.
// Upstream models in the PDK are pure supply pads — no signal behavior —
// but they include constructs (supply nets bound to inout ports) that some
// older Verilator versions trip on. These trivial stubs are functionally
// identical for simulation purposes (the netlist instantiates them but the
// chip works regardless of what's inside).

`timescale 1ns/1ps
`ifndef GF180MCU_WS_IO_V_SIM
`define GF180MCU_WS_IO_V_SIM

module gf180mcu_ws_io__dvdd (DVDD, DVSS, VSS);
    inout DVDD, DVSS, VSS;
endmodule

module gf180mcu_ws_io__dvss (DVDD, DVSS, VDD);
    inout DVDD, DVSS, VDD;
endmodule

`endif
