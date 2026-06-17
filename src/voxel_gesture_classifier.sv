// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
`timescale 1ns/1ps

// Scores arrive as SIGNED two's-complement values (the MAC accumulates signed
// weight×feature products). Every reduction, argmax and threshold comparison in
// this module is therefore a SIGNED operation — a class with a negative winning
// score must still be ordered correctly against the others and against its
// (possibly negative) class threshold.
module voxel_gesture_classifier #(
    parameter int NUM_CLASSES = 4,
    parameter int SCORE_BITS  = 37
)(
    input  logic                              clk,
    input  logic                              rst,
    input  logic [NUM_CLASSES*SCORE_BITS-1:0] scores_flat,
    input  logic                              scores_valid,

    output logic                  thresh_rd_valid,
    output logic [2:0]            thresh_rd_addr,
    input  logic [SCORE_BITS-1:0] thresh_data,

    output logic [1:0] class_gesture,
    output logic       class_valid,
    output logic       class_pass,

    output logic [1:0] gesture,
    output logic       gesture_valid,
    output logic       gesture_confidence,

    // debug bus output
    output logic [10:0] class_dbg
);

    localparam int ADDR_BITS = $clog2(NUM_CLASSES);

    generate
        if (NUM_CLASSES != 4) begin : gen_invalid
            initial $error("gesture_classifier requires NUM_CLASSES=4 (got %0d)", NUM_CLASSES);
        end
    endgenerate

    // ------------------------------------------------------------------
    // Stage 0: Register incoming scores.
    // ------------------------------------------------------------------
    logic [NUM_CLASSES*SCORE_BITS-1:0] scores_flat_r;
    logic                              scores_valid_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            scores_flat_r  <= '0;
            scores_valid_r <= 1'b0;
        end else begin
            scores_flat_r  <= scores_flat;
            scores_valid_r <= scores_valid;
        end
    end

    logic signed [SCORE_BITS-1:0] s0, s1, s2, s3;

    assign s0 = scores_flat_r[0*SCORE_BITS +: SCORE_BITS];
    assign s1 = scores_flat_r[1*SCORE_BITS +: SCORE_BITS];
    assign s2 = scores_flat_r[2*SCORE_BITS +: SCORE_BITS];
    assign s3 = scores_flat_r[3*SCORE_BITS +: SCORE_BITS];

    // ------------------------------------------------------------------
    // Stage 1 combinational: Pairwise compare.
    //
    // Pair 0: class 0 vs class 1
    // Pair 1: class 2 vs class 3
    // ------------------------------------------------------------------
    logic signed [SCORE_BITS-1:0] pair0_max_c, pair0_min_c;
    logic signed [SCORE_BITS-1:0] pair1_max_c, pair1_min_c;
    logic [1:0]                   pair0_cls_c, pair1_cls_c;

    always_comb begin
        if (s0 >= s1) begin
            pair0_max_c = s0;
            pair0_min_c = s1;
            pair0_cls_c = 2'd0;
        end else begin
            pair0_max_c = s1;
            pair0_min_c = s0;
            pair0_cls_c = 2'd1;
        end

        if (s2 >= s3) begin
            pair1_max_c = s2;
            pair1_min_c = s3;
            pair1_cls_c = 2'd2;
        end else begin
            pair1_max_c = s3;
            pair1_min_c = s2;
            pair1_cls_c = 2'd3;
        end
    end

    // ------------------------------------------------------------------
    // Stage 1 register: Store pairwise max/min results.
    // ------------------------------------------------------------------
    logic                         pair_valid_r;
    logic signed [SCORE_BITS-1:0] pair0_max_r, pair0_min_r;
    logic signed [SCORE_BITS-1:0] pair1_max_r, pair1_min_r;
    logic [1:0]                   pair0_cls_r, pair1_cls_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            pair_valid_r <= 1'b0;
            pair0_max_r  <= '0;
            pair0_min_r  <= '0;
            pair1_max_r  <= '0;
            pair1_min_r  <= '0;
            pair0_cls_r  <= '0;
            pair1_cls_r  <= '0;
        end else begin
            pair_valid_r <= scores_valid_r;

            if (scores_valid_r) begin
                pair0_max_r <= pair0_max_c;
                pair0_min_r <= pair0_min_c;
                pair1_max_r <= pair1_max_c;
                pair1_min_r <= pair1_min_c;
                pair0_cls_r <= pair0_cls_c;
                pair1_cls_r <= pair1_cls_c;
            end
        end
    end

    // ------------------------------------------------------------------
    // Stage 2a combinational: Choose the global winner and the two
    // candidates for second-best score.
    //
    // changed this from the previous version because timing moved back here.
    // before this stage did:
    //   pair max regs -> choose max -> choose second candidates
    //                 -> compare second candidates -> rank regs
    //
    // that left a pretty long cone from pair0_max_r / pair1_max_r into the
    // rank regs. now this stage only chooses the winner and the two possible
    // second-best values. the actual second-best compare is one cycle later.
    // ------------------------------------------------------------------
    logic                         pair0_wins_b;
    logic signed [SCORE_BITS-1:0] max_score_sel_b;
    logic signed [SCORE_BITS-1:0] second_a_sel_b;
    logic signed [SCORE_BITS-1:0] second_b_sel_b;
    logic [1:0]                   max_class_sel_b;

    assign pair0_wins_b    = (pair0_max_r >= pair1_max_r);

    assign max_score_sel_b = pair0_wins_b ? pair0_max_r : pair1_max_r;
    assign max_class_sel_b = pair0_wins_b ? pair0_cls_r : pair1_cls_r;

    // second_a is the losing pair's max.
    // second_b is the winning pair's min.
    // the real second-best score is max(second_a, second_b), but that compare
    // is now registered into the next stage instead of being done here.
    assign second_a_sel_b  = pair0_wins_b ? pair1_max_r : pair0_max_r;
    assign second_b_sel_b  = pair0_wins_b ? pair0_min_r : pair1_min_r;

    // ------------------------------------------------------------------
    // NEW Stage 2a register: candidate-result pipeline stage.
    //
    // added this stage to cut the path that was showing up from pair0_max_r
    // into the old rank registers. this costs one extra classifier cycle,
    // but it keeps the max score, max class, and second-score candidates
    // aligned with each other.
    // ------------------------------------------------------------------
    logic                         cand_valid_r;
    logic [1:0]                   max_class_cand_r;
    logic signed [SCORE_BITS-1:0] max_score_cand_r;
    logic signed [SCORE_BITS-1:0] second_a_cand_r;
    logic signed [SCORE_BITS-1:0] second_b_cand_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            cand_valid_r     <= 1'b0;
            max_class_cand_r <= '0;
            max_score_cand_r <= '0;
            second_a_cand_r  <= '0;
            second_b_cand_r  <= '0;
        end else begin
            cand_valid_r <= pair_valid_r;

            if (pair_valid_r) begin
                max_class_cand_r <= max_class_sel_b;
                max_score_cand_r <= max_score_sel_b;
                second_a_cand_r  <= second_a_sel_b;
                second_b_cand_r  <= second_b_sel_b;
            end
        end
    end

    // ------------------------------------------------------------------
    // Stage 2b combinational: Compute second-best score only.
    //
    // this used to be part of the pair regs -> rank regs path. now it starts
    // from second_a_cand_r / second_b_cand_r, so the previous pair max regs
    // do not have to drive this compare in the same cycle.
    // ------------------------------------------------------------------
    logic signed [SCORE_BITS-1:0] second_score_from_cand_c;

    assign second_score_from_cand_c = (second_a_cand_r >= second_b_cand_r)
                                      ? second_a_cand_r
                                      : second_b_cand_r;

    // ------------------------------------------------------------------
    // Stage 2b register: Rank-result pipeline stage.
    //
    // changed from the previous version: rank_valid_r now follows
    // cand_valid_r instead of pair_valid_r. this is the extra cycle added
    // to split the classifier ranking path.
    // ------------------------------------------------------------------
    logic                         rank_valid_r;
    logic [1:0]                   max_class_rank_r;
    logic signed [SCORE_BITS-1:0] max_score_rank_r;
    logic signed [SCORE_BITS-1:0] second_score_rank_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            rank_valid_r        <= 1'b0;
            max_class_rank_r    <= '0;
            max_score_rank_r    <= '0;
            second_score_rank_r <= '0;
        end else begin
            rank_valid_r <= cand_valid_r;

            if (cand_valid_r) begin
                max_class_rank_r    <= max_class_cand_r;
                max_score_rank_r    <= max_score_cand_r;
                second_score_rank_r <= second_score_from_cand_c;
            end
        end
    end

    // ------------------------------------------------------------------
    // Stage 2c combinational: Compute score difference.
    //
    // diff is still computed from registered rank-stage values. this part
    // was already split out earlier and is left structurally the same.
    // ------------------------------------------------------------------
    logic signed [SCORE_BITS-1:0] diff_from_rank_c;

    assign diff_from_rank_c = max_score_rank_r - second_score_rank_r;

    // ------------------------------------------------------------------
    // Stage 2c register: Store final max class, max score, and diff.
    // ------------------------------------------------------------------
    logic                         decision_valid_r;
    logic [1:0]                   max_class_r;
    logic signed [SCORE_BITS-1:0] max_score_r;
    logic signed [SCORE_BITS-1:0] diff_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            decision_valid_r <= 1'b0;
            max_class_r      <= '0;
            max_score_r      <= '0;
            diff_r           <= '0;
        end else begin
            decision_valid_r <= rank_valid_r;

            if (rank_valid_r) begin
                max_class_r <= max_class_rank_r;
                max_score_r <= max_score_rank_r;
                diff_r      <= diff_from_rank_c;
            end
        end
    end

    // ------------------------------------------------------------------
    // Threshold SRAM read scheduling.
    //
    // this still works with the extra candidate stage because rank_valid_r
    // only goes high when max_class_rank_r is valid. the whole threshold read
    // sequence is just one classifier cycle later now.
    //
    // Read sequence:
    //   rank_valid_r      -> read class threshold at {1'b0, max_class}
    //   decision_valid_r  -> read diff threshold  at {1'b1, max_class}
    // ------------------------------------------------------------------
    assign thresh_rd_valid = rank_valid_r | decision_valid_r;

    assign thresh_rd_addr  = rank_valid_r
                             ? {1'b0, max_class_rank_r[ADDR_BITS-1:0]}
                             : {1'b1, max_class_r[ADDR_BITS-1:0]};

    // ------------------------------------------------------------------
    // Stage 3: Capture class threshold and delay decision fields.
    //
    // The class-threshold SRAM data is captured while decision_valid_r is
    // high. The diff-threshold read is issued during that same cycle and is
    // available for the final comparison one cycle later.
    // ------------------------------------------------------------------
    logic                         decision_valid_r2;
    logic [1:0]                   max_class_r2;
    logic signed [SCORE_BITS-1:0] max_score_r2;
    logic signed [SCORE_BITS-1:0] diff_r2;
    logic signed [SCORE_BITS-1:0] class_thresh_r;

    // Signed view of the threshold SRAM read data for the diff comparison below.
    logic signed [SCORE_BITS-1:0] thresh_data_s;
    assign thresh_data_s = thresh_data;

    always_ff @(posedge clk) begin
        if (rst) begin
            decision_valid_r2 <= 1'b0;
            max_class_r2      <= '0;
            max_score_r2      <= '0;
            diff_r2           <= '0;
            class_thresh_r    <= '0;
        end else begin
            decision_valid_r2 <= decision_valid_r;

            if (decision_valid_r) begin
                max_class_r2   <= max_class_r;
                max_score_r2   <= max_score_r;
                diff_r2        <= diff_r;
                class_thresh_r <= thresh_data;
            end
        end
    end

    // ------------------------------------------------------------------
    // Stage 4: Final class/diff threshold decisions.
    //
    // class_thresh_r was captured in the previous stage.
    // thresh_data_s holds the diff threshold from the staggered SRAM read.
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            class_gesture      <= 2'd0;
            class_valid        <= 1'b0;
            class_pass         <= 1'b0;
            gesture            <= 2'd0;
            gesture_valid      <= 1'b0;
            gesture_confidence <= 1'b0;
        end else begin
            class_valid        <= 1'b0;
            class_pass         <= 1'b0;
            gesture_valid      <= 1'b0;
            gesture_confidence <= 1'b0;

            if (decision_valid_r2) begin
                class_gesture <= max_class_r2;
                class_valid   <= 1'b1;

                if (max_score_r2 > class_thresh_r) begin
                    class_pass         <= 1'b1;
                    gesture            <= max_class_r2;
                    gesture_valid      <= 1'b1;

                    // Diff threshold arrives from the staggered threshold SRAM read.
                    gesture_confidence <= (diff_r2 > thresh_data_s);
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Debug bus connections.
    // ------------------------------------------------------------------
    assign class_dbg[0]  = thresh_rd_valid;
    assign class_dbg[1]  = thresh_rd_addr[0];
    assign class_dbg[2]  = thresh_rd_addr[1];
    assign class_dbg[3]  = class_gesture[0];
    assign class_dbg[4]  = class_gesture[1];
    assign class_dbg[5]  = class_valid;
    assign class_dbg[6]  = class_pass;
    assign class_dbg[7]  = gesture[0];
    assign class_dbg[8]  = gesture[1];
    assign class_dbg[9]  = gesture_valid;
    assign class_dbg[10] = gesture_confidence;

endmodule