// SPDX-License-Identifier: Apache-2.0
//
// Behavioural stubs for standard cells that are missing from the shipped
// gf180mcu_as_sc_mcu7t3v3.v (the gf180mcuD PDK ships an incomplete library
// file for the 3.3V AS std-cell set).  These cells ARE present in the .lib
// timing files and ARE used by the synthesised netlist; without these
// stubs Icarus errors with "Unknown module type".
//
// Boolean functions taken verbatim from
//   gf180mcu_as_sc_mcu7t3v3__tt_025C_3v30.lib:
//     ao211 :  Y = (A & B) | C | D
//     aoi211: Y = !((A & B) | C | D)
//     oai211: Y = !((A | B) & C & D)
//     xor2  : Y = (A & !B) | (B & !A)     // = A ^ B
//
// Pin order matches the netlist instantiations (and the convention used by
// the other ao/aoi/oai cells in gf180mcu_as_sc_mcu7t3v3.v): power pins
// VPW/VNW/VDD/VSS first, then signal inputs A,B[,C,D], output Y.

`timescale 1ns/1ps

module gf180mcu_as_sc_mcu7t3v3__ao211_2 (
    input  VPW,
    input  VNW,
    input  VDD,
    input  VSS,
    input  A,
    input  B,
    input  C,
    input  D,
    output Y
);
    assign Y = (A & B) | C | D;
endmodule

module gf180mcu_as_sc_mcu7t3v3__aoi211_2 (
    input  VPW,
    input  VNW,
    input  VDD,
    input  VSS,
    input  A,
    input  B,
    input  C,
    input  D,
    output Y
);
    assign Y = ~((A & B) | C | D);
endmodule

module gf180mcu_as_sc_mcu7t3v3__oai211_2 (
    input  VPW,
    input  VNW,
    input  VDD,
    input  VSS,
    input  A,
    input  B,
    input  C,
    input  D,
    output Y
);
    assign Y = ~((A | B) & C & D);
endmodule

module gf180mcu_as_sc_mcu7t3v3__xor2_2 (
    input  VPW,
    input  VNW,
    input  VDD,
    input  VSS,
    input  A,
    input  B,
    output Y
);
    assign Y = A ^ B;
endmodule
