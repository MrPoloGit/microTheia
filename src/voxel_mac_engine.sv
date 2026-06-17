// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
`timescale 1ns/1ps

// Weights are SIGNED two's-complement int8 (range -128..127). Features are the
// unsigned event counters from the feature RAM. The MAC therefore performs a
// signed×unsigned multiply and accumulates into a signed score, so a class can
// learn negative evidence ("this gesture is NOT active here"). Downstream the
// gesture classifier compares the resulting scores as signed quantities.
module voxel_mac_engine #(
    parameter int FEATURE_COUNT = 4096,
    parameter int COUNTER_BITS  = 16,
    parameter int WEIGHT_BITS   = 8,
    parameter int NUM_CLASSES   = 4,

    // added this because the RAM read latency is no longer fixed at 1 cycle
    // after enabling PIPELINE_READ in the SRAM wrappers. Set this to the number
    // of cycles from rd_en/rd_addr being issued to feature_data/weight_data_flat
    // being valid at this module's inputs.
    //
    // Old non-pipelined wrapper behavior: READ_LATENCY = 1
    // Wrapper with PIPELINE_READ=1:     READ_LATENCY = 2
    parameter int READ_LATENCY  = 4,

    parameter int SCORE_BITS    = COUNTER_BITS + WEIGHT_BITS + $clog2(FEATURE_COUNT) + 1 // localparam
) (
    input  logic clk,
    input  logic rst,

    input  logic start,
    output logic busy,

    output logic                               rd_en,
    output logic [$clog2(FEATURE_COUNT)-1:0]   rd_addr,

    input  logic [COUNTER_BITS-1:0]            feature_data,
    input  logic [NUM_CLASSES*WEIGHT_BITS-1:0] weight_data_flat,

    output logic [NUM_CLASSES*SCORE_BITS-1:0]  scores_flat,
    output logic                               scores_valid,

    // output ports for debug
    output logic [14:0]                        mac_dbg,
    output logic [31:0]                        score_A, score_B, score_C, score_D //scores are all truncated to 32 bits
);

    localparam int ADDR_BITS   = $clog2(FEATURE_COUNT);
    localparam int STREAM_BITS = $clog2(FEATURE_COUNT + 1);

    typedef enum logic [1:0] {
        ST_IDLE    = 2'd0,
        ST_STREAM  = 2'd1,
        ST_PUBLISH = 2'd2
    } state_t;

    state_t                 state;
    logic [STREAM_BITS-1:0] stream_idx;

    // ------------------------------------------------------------------
    // RAM read-latency alignment.
    //
    // changed this from the old single-cycle mac_valid/mac_last registers.
    // the old version assumed synchronous RAM data came back exactly one cycle
    // after rd_en. after adding read pipeline stages, that is no longer true.
    //
    // issue_valid marks the cycle where we request a feature/weight address.
    // issue_last marks the final requested address.
    //
    // valid_pipe/last_pipe delay those markers until the returned RAM data is
    // actually lined up with feature_data and weight_data_flat.
    // ------------------------------------------------------------------
    logic issue_valid;
    logic issue_last;

    logic [READ_LATENCY-1:0] valid_pipe;
    logic [READ_LATENCY-1:0] last_pipe;

    logic mac_valid; // delayed to match returned feature/weight data
    logic mac_last;  // delayed marker for the final accumulation

    assign issue_valid = (state == ST_STREAM) &&
                         (stream_idx < STREAM_BITS'(FEATURE_COUNT));

    assign issue_last  = (state == ST_STREAM) &&
                         (stream_idx == STREAM_BITS'(FEATURE_COUNT - 1));

    assign mac_valid = valid_pipe[READ_LATENCY-1];
    assign mac_last  = last_pipe[READ_LATENCY-1];

    logic signed [SCORE_BITS-1:0] score_acc [0:NUM_CLASSES-1];

    // rd_addr held at 0 when idle to avoid X propagation in simulation.
    assign rd_en   = issue_valid;
    assign rd_addr = rd_en ? stream_idx[ADDR_BITS-1:0] : '0;
    assign busy    = (state != ST_IDLE);

    always_comb begin
        for (int g = 0; g < NUM_CLASSES; g++)
            scores_flat[g*SCORE_BITS +: SCORE_BITS] = score_acc[g];
    end

    // score busses for debug pins
    // only taking lower 32 bits from each score
    assign score_A = scores_flat[31:0];       // from [36:0]
    assign score_B = scores_flat[68:37];      // from [73:37]
    assign score_C = scores_flat[105:74];     // from [110:74]
    assign score_D = scores_flat[142:111];    // from [147:111]

    always_ff @(posedge clk) begin
        if (rst) begin
            state        <= ST_IDLE;
            stream_idx   <= '0;
            valid_pipe   <= '0;
            last_pipe    <= '0;
            scores_valid <= 1'b0;

            for (int g = 0; g < NUM_CLASSES; g++)
                score_acc[g] <= '0;
        end else begin
            scores_valid <= 1'b0;

            // keep shifting while in ST_STREAM so the MAC can drain the final
            // RAM responses after the last read request has already been issued
            if (state == ST_STREAM) begin
                valid_pipe <= {valid_pipe[READ_LATENCY-2:0], issue_valid};
                last_pipe  <= {last_pipe[READ_LATENCY-2:0],  issue_last};
            end else begin
                valid_pipe <= '0;
                last_pipe  <= '0;
            end

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        stream_idx <= '0;
                        valid_pipe <= '0;
                        last_pipe  <= '0;

                        for (int g = 0; g < NUM_CLASSES; g++)
                            score_acc[g] <= '0;

                        state <= ST_STREAM;
                    end
                end

                ST_STREAM: begin
                    // Issue one read per cycle until all FEATURE_COUNT addresses
                    // have been requested. After that, stay in ST_STREAM while
                    // valid_pipe/last_pipe drain the remaining RAM responses.
                    if (stream_idx < STREAM_BITS'(FEATURE_COUNT))
                        stream_idx <= stream_idx + 1'b1;

                    if (mac_valid) begin
                        for (int g = 0; g < NUM_CLASSES; g++) begin
                            // Signed MAC: the feature is an unsigned event counter
                            // (always >= 0), so zero-extend it by one bit into a
                            // signed container; the weight is signed int8. The
                            // 17-bit × 8-bit product is a signed 25-bit value that
                            // sign-extends to SCORE_BITS in the signed accumulate.
                            logic signed [COUNTER_BITS:0]             feat_s;
                            logic signed [WEIGHT_BITS-1:0]            weight_s;
                            logic signed [COUNTER_BITS+WEIGHT_BITS:0] product;

                            feat_s   = $signed({1'b0, feature_data});
                            weight_s = $signed(weight_data_flat[g*WEIGHT_BITS +: WEIGHT_BITS]);
                            product  = feat_s * weight_s;

                            score_acc[g] <= score_acc[g] + product;
                        end
                    end

                    // changed this from issue-side mac_last to delayed mac_last.
                    // We only publish after the final returned RAM word has
                    // actually been accumulated.
                    if (mac_last)
                        state <= ST_PUBLISH;
                end

                ST_PUBLISH: begin
                    scores_valid <= 1'b1;
                    state        <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

    // debug bus connections
    assign mac_dbg[0]  = start;
    assign mac_dbg[1]  = busy;
    assign mac_dbg[2]  = rd_en;
    assign mac_dbg[3]  = scores_valid;
    assign mac_dbg[4]  = rd_addr[0];
    assign mac_dbg[5]  = rd_addr[1];
    assign mac_dbg[6]  = rd_addr[2];
    assign mac_dbg[7]  = rd_addr[3];
    assign mac_dbg[8]  = rd_addr[4];
    assign mac_dbg[9]  = rd_addr[5];
    assign mac_dbg[10] = rd_addr[6];
    assign mac_dbg[11] = rd_addr[7];
    assign mac_dbg[12] = rd_addr[8];
    assign mac_dbg[13] = rd_addr[9];
    assign mac_dbg[14] = rd_addr[10];

endmodule