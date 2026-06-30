// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
`timescale 1ns/1ps

// GF180MCU synchronous SRAM wrapper — single-read / single-write port.
//
// Tiling:
//   Width : BYTES = ceil(width_p/8) byte-lane macros per bank.
//   Depth : Smallest OCD 3.3V macro that fits depth_p in one bank (256, 512, or
//           1024); for depth_p beyond that, up to 4 banks are cascaded.
//
// Read latency:
//   PIPELINE_READ=0 : 1 cycle, matching original behavior.
//   PIPELINE_READ=1 : 2 cycles, with an extra register after SRAM macro Q.
//                     Intended for timing relief on selected SRAMs only.
//
// Write latency : 0 cycles (write takes effect on CLK rising edge).
//
// Simultaneous read+write to the same bank: write wins (GWEN=0 overrides read).
//
// CEN protocol: CEN is held HIGH (disabled) while reset_i is asserted so the
//   GF180 model sees the required 1→0 falling edge on the first real access.
//   Each bank is only enabled (CEN=0) during its own active operation; all other
//   banks are disabled (CEN=1) every cycle — standard power-saving practice.

module sram_wrapper #(
    parameter int             width_p       = 8,
    parameter int             depth_p       = 512,
    parameter [8*128-1:0]     filename_p    = "",
    parameter bit             PIPELINE_READ = 1'b0
)(
`ifdef USE_POWER_PINS
    inout  wire                           VDD,
    inout  wire                           VSS,
`endif

    input  logic                          clk_i,
    input  logic                          reset_i,

    input  logic                          wr_valid_i,
    input  logic [width_p-1:0]            wr_data_i,
    input  logic [$clog2(depth_p)-1:0]    wr_addr_i,

    input  logic                          rd_valid_i,
    input  logic [$clog2(depth_p)-1:0]    rd_addr_i,
    output logic [width_p-1:0]            rd_data_o
);

    // ------------------------------------------------------------------
    // Tiling parameters
    // ------------------------------------------------------------------
    localparam int BYTES      = (width_p + 7) / 8;

    localparam int MACRO_DEPTH = (depth_p <= 256) ? 256  :
                                  (depth_p <= 512) ? 512  : 1024;
    localparam int MACRO_ABITS = (MACRO_DEPTH == 256) ? 8 :
                                  (MACRO_DEPTH == 512) ? 9 : 10;
    localparam int NUM_BANKS  = (depth_p + MACRO_DEPTH - 1) / MACRO_DEPTH;
    localparam int ADDR_BITS  = $clog2(depth_p > 1 ? depth_p : 2);

    initial begin
        if (depth_p < 1 || depth_p > 4096)
            $fatal(1, "%m: depth_p=%0d out of range — wrapper supports 1–4096 (4 banks × 1024)", depth_p);
    end

`ifndef SYNTHESIS
    // ------------------------------------------------------------------
    // Behavioral simulation model
    // ------------------------------------------------------------------
    logic [width_p-1:0] sim_mem [0:depth_p-1];
    logic [width_p-1:0] sim_rd_data_r;
    logic [width_p-1:0] sim_rd_data_pipe_r;

    integer sim_init_i;
    initial begin
        for (sim_init_i = 0; sim_init_i < depth_p; sim_init_i = sim_init_i + 1)
            sim_mem[sim_init_i] = '0;
    end

    always_ff @(posedge clk_i) begin
        if (reset_i) begin
            sim_rd_data_r      <= '0;
            sim_rd_data_pipe_r <= '0;
        end else begin
            if (wr_valid_i)
                sim_mem[wr_addr_i] <= wr_data_i;

            if (rd_valid_i)
                sim_rd_data_r <= sim_mem[rd_addr_i];

            if (PIPELINE_READ)
                sim_rd_data_pipe_r <= sim_rd_data_r;
        end
    end

    assign rd_data_o = PIPELINE_READ ? sim_rd_data_pipe_r : sim_rd_data_r;

    always @(posedge clk_i) begin
        if (!reset_i && wr_valid_i && rd_valid_i && (wr_addr_i != rd_addr_i))
            $warning("%m @%0t: simultaneous wr+rd to different addresses (wr=%0d rd=%0d) — synthesis drops the read", $time, wr_addr_i, rd_addr_i);
    end

`else
    // ------------------------------------------------------------------
    // Synthesis path — real GF180MCU SRAM macro tiles
    // ------------------------------------------------------------------

    logic [1:0]              wr_bank, rd_bank;
    logic [MACRO_ABITS-1:0]  wr_addr_low, rd_addr_low;

    generate
        if (NUM_BANKS > 1) begin : g_multi_addr
            assign wr_bank     = 2'(wr_addr_i[ADDR_BITS-1:MACRO_ABITS]);
            assign rd_bank     = 2'(rd_addr_i[ADDR_BITS-1:MACRO_ABITS]);
            assign wr_addr_low = wr_addr_i[MACRO_ABITS-1:0];
            assign rd_addr_low = rd_addr_i[MACRO_ABITS-1:0];
        end else begin : g_single_addr
            assign wr_bank     = 2'd0;
            assign rd_bank     = 2'd0;
            assign wr_addr_low = MACRO_ABITS'(wr_addr_i);
            assign rd_addr_low = MACRO_ABITS'(rd_addr_i);
        end
    endgenerate

    // Registered bank select for original 1-cycle read behavior.
    logic [1:0] rd_bank_r;
    always_ff @(posedge clk_i) begin
        if (reset_i)
            rd_bank_r <= 2'd0;
        else if (rd_valid_i)
            rd_bank_r <= rd_bank;
    end

    // Q outputs from every tile: [bank][byte_lane]
    logic [7:0] q_all [NUM_BANKS][BYTES];

    // ------------------------------------------------------------------
    // Read-data output path
    //
    // PIPELINE_READ=0:
    //   q_all -> mux by rd_bank_r -> rd_data_o
    //
    // PIPELINE_READ=1:
    //   q_all -> q_all_r register -> mux by rd_bank_rr -> rd_data_o
    //
    // This keeps all non-pipelined SRAM instances identical to the original
    // wrapper and only adds latency/registers where the parameter is enabled.
    // ------------------------------------------------------------------
    generate
        if (PIPELINE_READ) begin : g_pipelined_read_output
            logic [7:0] q_all_r [NUM_BANKS][BYTES];
            logic [1:0] rd_bank_rr;
            logic [BYTES*8-1:0] rd_data_wide;

            always_ff @(posedge clk_i) begin
                if (reset_i) begin
                    rd_bank_rr <= 2'd0;
                    for (int b_i = 0; b_i < NUM_BANKS; b_i++) begin
                        for (int byte_j = 0; byte_j < BYTES; byte_j++) begin
                            q_all_r[b_i][byte_j] <= '0;
                        end
                    end
                end else begin
                    rd_bank_rr <= rd_bank_r;
                    for (int b_i = 0; b_i < NUM_BANKS; b_i++) begin
                        for (int byte_j = 0; byte_j < BYTES; byte_j++) begin
                            q_all_r[b_i][byte_j] <= q_all[b_i][byte_j];
                        end
                    end
                end
            end

            always_comb begin
                rd_data_wide = '0;
                for (int i = 0; i < BYTES; i++) begin
                    rd_data_wide[i*8 +: 8] = q_all_r[rd_bank_rr][i];
                end
            end

            assign rd_data_o = rd_data_wide[width_p-1:0];

        end else begin : g_unpipelined_read_output
            logic [BYTES*8-1:0] rd_data_wide;

            always_comb begin
                rd_data_wide = '0;
                for (int i = 0; i < BYTES; i++) begin
                    rd_data_wide[i*8 +: 8] = q_all[rd_bank_r][i];
                end
            end

            assign rd_data_o = rd_data_wide[width_p-1:0];
        end
    endgenerate

    // Zero-pad write data to a whole number of bytes.
    logic [BYTES*8-1:0] wr_data_pad;
    assign wr_data_pad = {{(BYTES*8 - width_p){1'b0}}, wr_data_i};

    // Macro tiles: NUM_BANKS banks × BYTES byte-lane macros.
    genvar b_GEN, byte_i_GEN;
    generate
        for (b_GEN = 0; b_GEN < NUM_BANKS; b_GEN++) begin : gen_bank
            logic [MACRO_ABITS-1:0] addr_b;
            assign addr_b = (wr_valid_i & (wr_bank == 2'(b_GEN)))
                            ? wr_addr_low : rd_addr_low;

            for (byte_i_GEN = 0; byte_i_GEN < BYTES; byte_i_GEN++) begin : gen_byte
                logic cen_w, gwen_w;

                assign cen_w  = reset_i | ~(
                                    (wr_valid_i & (wr_bank == 2'(b_GEN))) |
                                    (rd_valid_i & (rd_bank == 2'(b_GEN)))
                                );

                assign gwen_w = ~(wr_valid_i & (wr_bank == 2'(b_GEN)));

                if (MACRO_DEPTH == 256) begin : sel
                    gf180mcu_ocd_ip_sram__sram256x8m8wm1 u_sram (
`ifdef USE_POWER_PINS
                        .VDD (VDD),
                        .VSS (VSS),
`endif
                        .CLK (clk_i),
                        .CEN (cen_w),
                        .GWEN(gwen_w),
                        .WEN (8'h00),
                        .A   (addr_b[7:0]),
                        .D   (wr_data_pad[byte_i_GEN*8 +: 8]),
                        .Q   (q_all[b_GEN][byte_i_GEN])
                    );
                end else if (MACRO_DEPTH == 512) begin : sel
                    gf180mcu_ocd_ip_sram__sram512x8m8wm1 u_sram (
`ifdef USE_POWER_PINS
                        .VDD (VDD),
                        .VSS (VSS),
`endif
                        .CLK (clk_i),
                        .CEN (cen_w),
                        .GWEN(gwen_w),
                        .WEN (8'h00),
                        .A   (addr_b[8:0]),
                        .D   (wr_data_pad[byte_i_GEN*8 +: 8]),
                        .Q   (q_all[b_GEN][byte_i_GEN])
                    );
                end else begin : sel
                    gf180mcu_ocd_ip_sram__sram1024x8m8wm1 u_sram (
`ifdef USE_POWER_PINS
                        .VDD (VDD),
                        .VSS (VSS),
`endif
                        .CLK (clk_i),
                        .CEN (cen_w),
                        .GWEN(gwen_w),
                        .WEN (8'h00),
                        .A   (addr_b[9:0]),
                        .D   (wr_data_pad[byte_i_GEN*8 +: 8]),
                        .Q   (q_all[b_GEN][byte_i_GEN])
                    );
                end
            end
        end
    endgenerate

`endif // SYNTHESIS

endmodule