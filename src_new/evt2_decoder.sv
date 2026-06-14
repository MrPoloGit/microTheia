// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
`timescale 1ns/1ps

// EVT2 word fields: [31:28] type, [27:22] ts_lsb, [21:11] x, [10:0] y
// Packet types: 0x0 CD_OFF, 0x1 CD_ON, 0x8 TIME_HIGH
//
// Weight (EVT_WEIGHT) and threshold (EVT_THRESH_U/L) payloads are SIGNED
// two's-complement values (int8 weights, signed SCORE_BITS thresholds). They are
// transported and written into their SRAMs as raw bit patterns here — the signed
// interpretation lives in the MAC engine / gesture classifier that read them.

module evt2_decoder #(
    parameter int SENSOR_WIDTH      = 320,
    parameter int SENSOR_HEIGHT     = 320,
    parameter int FEATURE_COUNT     = 4096,
    parameter int GRID_SIZE         = 16,
    parameter int SCORE_BITS        = 37,
    parameter int WEIGHT_BITS       = 8,
    parameter bit REQUIRE_TIME_HIGH = 1'b1,
    parameter bit SWAP_INPUT_BYTES  = 1'b0
)(
    input  logic                             clk,
    input  logic                             rst,
    input  logic [31:0]                      data_in,
    input  logic                             data_valid,
    input  logic                             event_ready_i,
    input  logic                             evt_ld_en,
    output logic                             data_ready,
    output logic [$clog2(GRID_SIZE)-1:0]     x_out,
    output logic [$clog2(GRID_SIZE)-1:0]     y_out,
    output logic                             event_valid,
    output logic                             evt_reads_done,
    output logic [$clog2(FEATURE_COUNT)-1:0] weight_addr_o,
    output logic [WEIGHT_BITS-1:0]           weight_data_o,
    output logic [5:0]                       weight_sram_addr_o,
    output logic                             weight_event_valid,
    output logic [SCORE_BITS-1:0]            thresh_data_o,
    output logic [2:0]                       thresh_addr_o,
    output logic                             thresh_event_valid,
    output logic [33:0]                      ts_out,
    output logic [33:0]                      bin_length_us,
    output logic                             bin_length_valid,
    output logic                             debug_req_o,
    output logic                             reload_req_o,
    output logic                             boot_req_o,
    output logic [11:0]                      decoder_dbg,
    output logic [31:0]                      decoder_output_dbg,
    output logic [3:0]                       debug_page_sel
);

    localparam int GRID_BITS  = $clog2(GRID_SIZE);
    localparam logic [3:0] EVT_CD_OFF     = 4'h0;
    localparam logic [3:0] EVT_CD_ON      = 4'h1;
    localparam logic [3:0] EVT_WEIGHT     = 4'h2;
    localparam logic [3:0] EVT_THRESH_U   = 4'h3;
    localparam logic [3:0] EVT_THRESH_L   = 4'h4;
    localparam logic [3:0] BIN_LENGTH_U   = 4'h5;
    localparam logic [3:0] BIN_LENGTH_L   = 4'h6;
    localparam logic [3:0] VOXEL_DIMS     = 4'h7;
    localparam logic [3:0] EVT_TIME_HIGH  = 4'h8;

    localparam logic [3:0] DEBUG_REQ      = 4'ha;
    localparam logic [3:0] RELOAD_REQ     = 4'hb;
    localparam logic [3:0] BOOT_REQ       = 4'hc;

    localparam logic [3:0] DEBUG_PAGE     = 4'he;
    localparam logic [3:0] EVT_READS_DONE = 4'hf;

    localparam int SENSOR_W_M1 = SENSOR_WIDTH  - 1;
    localparam int SENSOR_H_M1 = SENSOR_HEIGHT - 1;

    // Input decode
    wire [31:0] evt_word = SWAP_INPUT_BYTES
                         ? {data_in[7:0], data_in[15:8], data_in[23:16], data_in[31:24]}
                         : data_in;

    wire [3:0]  pkt_type = evt_word[31:28];
    wire [10:0] x_raw    = evt_word[21:11];
    wire [10:0] y_raw    = evt_word[10:0];
    wire        is_cd    = (pkt_type == EVT_CD_OFF) || (pkt_type == EVT_CD_ON);

    // Grid coordinate combinational logic
    logic [10:0]          x_clamped,   y_clamped;
    logic [GRID_BITS-1:0] x_grid,      y_grid;
    logic [10:0]                      xbound_q [0:15];
    logic [10:0]                      xbound_d [0:15];
    logic [10:0]                      ybound_q [0:15];
    logic [10:0]                      ybound_d [0:15];

    always_comb begin
        x_clamped  = (x_raw >= 11'(SENSOR_WIDTH))  ? SENSOR_W_M1[10:0] : x_raw;
        y_clamped  = (y_raw >= 11'(SENSOR_HEIGHT)) ? SENSOR_H_M1[10:0] : y_raw;
        x_grid     = 4'd15;
        y_grid     = 4'd15;

        for (int i = 0; i < 15; i++) begin
            if (x_clamped <= xbound_q[i] && x_grid == 4'd15)
                x_grid = i[3:0];
        end

        for (int j = 0; j < 15; j++) begin
            if (y_clamped <= ybound_q[j] && y_grid == 4'd15)
                y_grid = j[3:0];
        end
    end

    // Backpressure
    assign data_ready = (!is_cd) || event_ready_i;

    // State registers (_q) and next-state signals (_d)
    logic                             have_time_high_q,       have_time_high_d;
    logic [27:0]                      time_high_reg_q,        time_high_reg_d;
    logic [GRID_BITS-1:0]             x_out_q,                x_out_d;
    logic [GRID_BITS-1:0]             y_out_q,                y_out_d;
    logic [33:0]                      ts_out_q,               ts_out_d;
    logic                             event_valid_q,          event_valid_d;
    logic                             weight_event_valid_q,   weight_event_valid_d;
    logic                             thresh_event_valid_q,   thresh_event_valid_d;
    logic                             evt_reads_done_q,       evt_reads_done_d;
    logic [18:0]                      thresh_reg_q,           thresh_reg_d;
    logic [$clog2(FEATURE_COUNT)-1:0] weight_addr_q,          weight_addr_d;
    logic [WEIGHT_BITS-1:0]           weight_data_q,          weight_data_d;
    logic [5:0]                       weight_sram_addr_q,     weight_sram_addr_d;
    logic [SCORE_BITS-1:0]            thresh_data_q,          thresh_data_d;
    logic [2:0]                       thresh_addr_q,          thresh_addr_d;
    logic [3:0]                       debug_page_sel_q,       debug_page_sel_d;
    logic                             boot_req_q,             boot_req_d;
    logic                             reload_req_q,           reload_req_d;
    logic                             debug_req_q,            debug_req_d;
    logic [33:0]                      bin_length_us_q,        bin_length_us_d;
    logic                             bin_length_valid_q,     bin_length_valid_d;
    logic [16:0]                      bin_length_reg_q,       bin_length_reg_d;

    // Next-state combinational block
    always_comb begin
        // Default: hold state, clear pulse signals
        have_time_high_d     = have_time_high_q;
        time_high_reg_d      = time_high_reg_q;
        x_out_d              = x_out_q;
        y_out_d              = y_out_q;
        ts_out_d             = ts_out_q;
        event_valid_d        = 1'b0;
        weight_event_valid_d = 1'b0;
        thresh_event_valid_d = 1'b0;
        evt_reads_done_d     = 1'b0;
        thresh_reg_d         = thresh_reg_q;
        weight_addr_d        = weight_addr_q;
        weight_data_d        = weight_data_q;
        weight_sram_addr_d   = weight_sram_addr_q;
        thresh_data_d        = thresh_data_q;
        thresh_addr_d        = thresh_addr_q;
        debug_page_sel_d     = debug_page_sel_q;
        boot_req_d           = 1'b0;
        reload_req_d         = 1'b0;
        debug_req_d          = 1'b0;
        bin_length_us_d      = bin_length_us_q;
        bin_length_valid_d   = 1'b0;
        bin_length_reg_d     = bin_length_reg_q;
        for (int i = 0; i < 16; i++) begin
            xbound_d[i] = xbound_q[i];
            ybound_d[i] = ybound_q[i];
        end

        if (data_valid && data_ready) begin
            case (pkt_type)
                EVT_TIME_HIGH: begin
                    have_time_high_d = 1'b1;
                    time_high_reg_d  = evt_word[27:0];
                end

                EVT_CD_OFF,
                EVT_CD_ON: begin
                    if (!REQUIRE_TIME_HIGH || have_time_high_q) begin
                        x_out_d       = x_grid;
                        y_out_d       = y_grid;
                        ts_out_d      = {time_high_reg_q, evt_word[27:22]};
                        event_valid_d = 1'b1;
                    end
                end

                EVT_WEIGHT: begin
                    if (evt_ld_en) begin
                        weight_data_d        = evt_word[27:20];
                        weight_addr_d        = evt_word[19:8];
                        weight_sram_addr_d   = evt_word[7:2];
                        weight_event_valid_d = 1'b1;
                    end
                end

                EVT_THRESH_U: begin
                    if (evt_ld_en) begin
                        thresh_reg_d = evt_word[27:9];
                    end
                end

                EVT_THRESH_L: begin
                    if (evt_ld_en) begin
                        thresh_data_d       = {thresh_reg_q, evt_word[27:10]};
                        thresh_addr_d       = evt_word[9:7];
                        thresh_event_valid_d = 1'b1;
                    end
                end

                BIN_LENGTH_U: begin
                    if (evt_ld_en) begin
                        bin_length_reg_d = evt_word[16:0];
                    end
                end

                BIN_LENGTH_L: begin
                    if (evt_ld_en) begin
                        bin_length_us_d    = {bin_length_reg_q, evt_word[16:0]};
                        bin_length_valid_d = 1'b1;
                    end
                end

                VOXEL_DIMS: begin
                    if (evt_ld_en) begin
                        xbound_d[evt_word[27:24]] = evt_word[23:13];
                        ybound_d[evt_word[27:24]] = evt_word[12:2];
                    end
                end

                EVT_READS_DONE: begin
                    evt_reads_done_d = 1'b1;
                end

                DEBUG_REQ: begin
                    debug_req_d = 1'b1;
                end

                RELOAD_REQ: begin
                    reload_req_d = 1'b1;
                end

                BOOT_REQ: begin
                    boot_req_d = 1'b1;
                end

                DEBUG_PAGE: begin
                    debug_page_sel_d = evt_word[27:24];
                end

                default: begin
                    event_valid_d        = 1'b0;
                    thresh_event_valid_d = 1'b0;
                    weight_event_valid_d = 1'b0;
                end
            endcase
        end
    end

    // Register block: _d -> _q on clock edge
    always_ff @(posedge clk) begin
        if (rst) begin
            have_time_high_q    <= 1'b0;
            time_high_reg_q     <= '0;
            x_out_q             <= '0;
            y_out_q             <= '0;
            ts_out_q            <= '0;
            event_valid_q       <= 1'b0;
            weight_event_valid_q <= 1'b0;
            thresh_event_valid_q <= 1'b0;
            evt_reads_done_q    <= 1'b0;
            thresh_reg_q        <= '0;
            weight_addr_q       <= '0;
            weight_data_q       <= '0;
            weight_sram_addr_q  <= '0;
            thresh_data_q       <= '0;
            thresh_addr_q       <= '0;
            debug_page_sel_q    <= '0;
            boot_req_q          <= 1'b0;
            reload_req_q        <= 1'b0;
            debug_req_q         <= 1'b0;
            bin_length_us_q     <= '0;
            bin_length_valid_q  <= 1'b0;
            bin_length_reg_q    <= '0;
            for (int i = 0; i < 16; i++) begin
                xbound_q[i] <= (i * 20 + 19);
                ybound_q[i] <= (i * 20 + 19);
            end
        end else begin
            have_time_high_q    <= have_time_high_d;
            time_high_reg_q     <= time_high_reg_d;
            x_out_q             <= x_out_d;
            y_out_q             <= y_out_d;
            ts_out_q            <= ts_out_d;
            event_valid_q       <= event_valid_d;
            weight_event_valid_q <= weight_event_valid_d;
            thresh_event_valid_q <= thresh_event_valid_d;
            evt_reads_done_q    <= evt_reads_done_d;
            thresh_reg_q        <= thresh_reg_d;
            weight_addr_q       <= weight_addr_d;
            weight_data_q       <= weight_data_d;
            weight_sram_addr_q  <= weight_sram_addr_d;
            thresh_data_q       <= thresh_data_d;
            thresh_addr_q       <= thresh_addr_d;
            debug_page_sel_q    <= debug_page_sel_d;
            boot_req_q          <= boot_req_d;
            reload_req_q        <= reload_req_d;
            debug_req_q         <= debug_req_d;
            bin_length_us_q     <= bin_length_us_d;
            bin_length_valid_q  <= bin_length_valid_d;
            bin_length_reg_q    <= bin_length_reg_d;
            for (int i = 0; i < 16; i++) begin
                xbound_q[i] <= xbound_d[i];
                ybound_q[i] <= ybound_d[i];
            end
        end
    end

    // Output assignments: _q -> module outputs
    assign x_out              = x_out_q;
    assign y_out              = y_out_q;
    assign ts_out             = ts_out_q;
    assign event_valid        = event_valid_q;
    assign weight_event_valid = weight_event_valid_q;
    assign thresh_event_valid = thresh_event_valid_q;
    assign evt_reads_done     = evt_reads_done_q;
    assign weight_addr_o      = weight_addr_q;
    assign weight_data_o      = weight_data_q;
    assign weight_sram_addr_o = weight_sram_addr_q;
    assign thresh_data_o      = thresh_data_q;
    assign thresh_addr_o      = thresh_addr_q;
    assign debug_page_sel     = debug_page_sel_q;
    assign boot_req_o         = boot_req_q;
    assign reload_req_o       = reload_req_q;
    assign debug_req_o        = debug_req_q;
    assign bin_length_us      = bin_length_us_q;
    assign bin_length_valid   = bin_length_valid_q;

    assign decoder_dbg        = {event_ready_i, data_valid, y_out_q, x_out_q, event_valid_q, data_ready};
    assign decoder_output_dbg = ts_out_q[31:0];

endmodule
