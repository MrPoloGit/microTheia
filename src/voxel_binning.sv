// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Group G Contributors
`timescale 1ns/1ps

// Features emitted oldest->newest bins, row-major within each bin (y major, x minor).
//
// Counter memory is a GF180MCU SRAM. This version accounts for the extra timing
// pipeline added around the counter SRAM:
//
//   logical read request
//     -> rd_valid_s / rd_addr_s register
//     -> SRAM synchronous read
//     -> wrapper PIPELINE_READ output stage
//     -> sram_rd_data <= rd_data_s register
//
// So read response metadata must be delayed to match the returned data.
//
// Accumulate (ST_ACCUM) uses a delayed read-modify-write pipeline:
//   Cycle N       : issue logical SRAM read for event address
//   Cycle N+L     : matching SRAM Q is available as sram_rd_data
//   Cycle N+L+... : writeback is staged through wr_valid_s/wr_addr_s/wr_data_s
//
// Timing update:
//   Incoming accepted events are first captured into event_stage_* registers.
//   Rollover compare and RMW issue use those registered fields instead of
//   directly using pending_event_valid/event_valid muxed inputs. This cuts the
//   old critical path from pending_event_valid through event select,
//   timestamp compare, and FSM control.

module voxel_binning #(
    parameter  int GRID_SIZE      = 16,
    parameter  int NUM_BINS       = 16,
    parameter  int READOUT_BINS   = 16,
    parameter  int COUNTER_BITS   = 16,
    localparam int RO_INDEX_WIDTH = READOUT_BINS*GRID_SIZE*GRID_SIZE
)(
`ifdef USE_POWER_PINS
    inout  wire                               VDD,
    inout  wire                               VSS,
`endif

    input  logic                              clk,
    input  logic                              rst,
    input  logic                              event_valid,
    input  logic [$clog2(GRID_SIZE)-1:0]      event_x,
    input  logic [$clog2(GRID_SIZE)-1:0]      event_y,
    input  logic [33:0]                       ts_in,
    input  logic                              force_rollover_i,
    input  logic [33:0]                       bin_length_us,
    input  logic                              bin_length_valid,
    output logic                              event_ready,
    input  logic                              readout_ready,
    output logic                              readout_start,
    output logic                              readout_valid,
    output logic [COUNTER_BITS-1:0]           readout_data,
    output logic [$clog2(RO_INDEX_WIDTH)-1:0] readout_index,
    output logic                              readout_last,
    output logic [30:0]                       vox_bin_dbg
);

    localparam int CELLS_PER_BIN       = GRID_SIZE * GRID_SIZE;
    localparam int TOTAL_CELLS         = NUM_BINS * CELLS_PER_BIN;
    localparam int FEATURE_COUNT       = READOUT_BINS * CELLS_PER_BIN;
    localparam int BIN_BITS            = (NUM_BINS > 1) ? $clog2(NUM_BINS) : 1;
    localparam int CELL_BITS           = (CELLS_PER_BIN > 1) ? $clog2(CELLS_PER_BIN) : 1;
    localparam int BIN_COUNT_BITS      = $clog2(NUM_BINS + 1);
    localparam int MEM_ADDR_BITS       = $clog2(TOTAL_CELLS > 1 ? TOTAL_CELLS : 2);
    localparam int RO_ADDR_BITS        = $clog2(RO_INDEX_WIDTH);
    localparam logic [33:0] DEFAULT_BIN_LENGTH = 34'd62500;

    // Effective latency from logical sram_rd_valid/sram_rd_addr to matching
    // sram_rd_data being valid.
    //
    // This assumes:
    //   1 cycle: rd_valid_s / rd_addr_s
    //   1 cycle: synchronous SRAM read
    //   1 cycle: wrapper PIPELINE_READ
    //   1 cycle: local sram_rd_data <= rd_data_s
    localparam int COUNTER_READ_LATENCY = 4;

    // ------------------------------------------------------------------
    // Programmable bin length
    // ------------------------------------------------------------------
    logic [33:0] bin_duration_ts;

    always_ff @(posedge clk) begin
        if (rst) begin
            bin_duration_ts <= DEFAULT_BIN_LENGTH;
        end else if (bin_length_valid && (bin_length_us != 34'd0)) begin
            bin_duration_ts <= bin_length_us;
        end
    end

    initial begin
        if (READOUT_BINS > NUM_BINS)
            $error("voxel_binning: READOUT_BINS (%0d) must be <= NUM_BINS (%0d)",
                   READOUT_BINS, NUM_BINS);
    end

    // ------------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------------
    typedef enum logic [1:0] {
        ST_ACCUM   = 2'd0,
        ST_WAIT_RD = 2'd1,
        ST_READOUT = 2'd2,
        ST_CLEAR   = 2'd3
    } state_t;

    state_t state;

    // ------------------------------------------------------------------
    // SRAM interface wires
    // ------------------------------------------------------------------
    logic                     sram_wr_valid, wr_valid_s;
    logic [MEM_ADDR_BITS-1:0] sram_wr_addr,  wr_addr_s;
    logic [COUNTER_BITS-1:0]  sram_wr_data,  wr_data_s;

    logic                     sram_rd_valid;
    logic [MEM_ADDR_BITS-1:0] sram_rd_addr;
    logic [COUNTER_BITS-1:0]  sram_rd_data, rd_data_s;

    // Registered SRAM request stage to break timing into the counter SRAM.
    logic                     rd_valid_s;
    logic [MEM_ADDR_BITS-1:0] rd_addr_s;

    // ------------------------------------------------------------------
    // Control registers
    // ------------------------------------------------------------------
    logic [33:0]               bin_start_ts;
    logic                      ts_initialized;
    logic [BIN_BITS-1:0]       wr_bin_idx;
    logic [BIN_BITS-1:0]       clear_bin_idx;
    logic [BIN_BITS-1:0]       snapshot_start_bin;
    logic [BIN_COUNT_BITS-1:0] completed_bins;

    logic [BIN_BITS-1:0]  rd_bin_off;
    logic [CELL_BITS-1:0] rd_cell_idx;
    logic [CELL_BITS-1:0] clear_cell_idx;

    // ------------------------------------------------------------------
    // Registered selected-event stage.
    //
    // This replaces the old pending_event_valid -> mux -> timestamp compare
    // same-cycle path. An event is accepted only when event_ready is high,
    // captured here, and then processed from these registers.
    //
    // If the staged event is beyond the current bin boundary, it stays staged
    // while the binner rolls/clears/reads out. When the binner returns to
    // ST_ACCUM and the timestamp has caught up, the same staged event is finally
    // written into the correct new bin.
    // ------------------------------------------------------------------
    logic                         event_stage_valid;
    logic [$clog2(GRID_SIZE)-1:0] event_stage_x;
    logic [$clog2(GRID_SIZE)-1:0] event_stage_y;
    logic [33:0]                  event_stage_ts;

    // Readout drain state: set after final address is issued, cleared once
    // the delayed final read response has been observed.
    logic rd_draining;

    // ------------------------------------------------------------------
    // Read-response metadata pipe
    //
    // Every logical read request pushes metadata into this pipe.
    // The output of this pipe lines up with sram_rd_data.
    //
    // rd_pipe_is_rmw = 1 => returned data belongs to an accumulate RMW read.
    // rd_pipe_is_rmw = 0 => returned data belongs to feature-window readout.
    // ------------------------------------------------------------------
    logic [COUNTER_READ_LATENCY-1:0] rd_pipe_valid;
    logic [COUNTER_READ_LATENCY-1:0] rd_pipe_is_rmw;
    logic [COUNTER_READ_LATENCY-1:0] rd_pipe_last;

    logic [MEM_ADDR_BITS-1:0] rd_pipe_addr  [0:COUNTER_READ_LATENCY-1];
    logic [RO_ADDR_BITS-1:0]  rd_pipe_index [0:COUNTER_READ_LATENCY-1];

    logic rd_resp_valid;
    logic rd_resp_is_rmw;
    logic rd_resp_is_readout;
    logic rd_resp_last;
    logic [MEM_ADDR_BITS-1:0] rd_resp_addr;
    logic [RO_ADDR_BITS-1:0]  rd_resp_index;

    assign rd_resp_valid      = rd_pipe_valid[COUNTER_READ_LATENCY-1];
    assign rd_resp_is_rmw     = rd_pipe_is_rmw[COUNTER_READ_LATENCY-1];
    assign rd_resp_is_readout = rd_resp_valid && !rd_resp_is_rmw;
    assign rd_resp_last       = rd_pipe_last[COUNTER_READ_LATENCY-1];
    assign rd_resp_addr       = rd_pipe_addr[COUNTER_READ_LATENCY-1];
    assign rd_resp_index      = rd_pipe_index[COUNTER_READ_LATENCY-1];

    logic rmw_read_inflight;
    logic rmw_busy;

    always_comb begin
        rmw_read_inflight = 1'b0;
        for (int i = 0; i < COUNTER_READ_LATENCY; i++) begin
            if (rd_pipe_valid[i] && rd_pipe_is_rmw)
                rmw_read_inflight = 1'b1;
        end
    end

    // Block a new event while an RMW read response is in flight, while the
    // response is being converted into a writeback, or while the registered
    // write request is still staged into the SRAM wrapper.
    assign rmw_busy = rmw_read_inflight || wr_valid_s;

    always_ff @(posedge clk) begin
        if (rst) begin
            wr_valid_s   <= 1'b0;
            wr_addr_s    <= '0;
            wr_data_s    <= '0;

            rd_valid_s   <= 1'b0;
            rd_addr_s    <= '0;
            sram_rd_data <= '0;

            rd_pipe_valid  <= '0;
            rd_pipe_is_rmw <= '0;
            rd_pipe_last   <= '0;

            for (int i = 0; i < COUNTER_READ_LATENCY; i++) begin
                rd_pipe_addr[i]  <= '0;
                rd_pipe_index[i] <= '0;
            end
        end else begin
            // Stage writes into SRAM wrapper.
            wr_valid_s <= sram_wr_valid;
            wr_addr_s  <= sram_wr_addr;
            wr_data_s  <= sram_wr_data;

            // Stage reads into SRAM wrapper.
            rd_valid_s <= sram_rd_valid;
            rd_addr_s  <= sram_rd_addr;

            // Stage SRAM returned data.
            sram_rd_data <= rd_data_s;

            // Metadata pipe for the corresponding read response.
            rd_pipe_valid[0]  <= sram_rd_valid;
            rd_pipe_is_rmw[0] <= (state == ST_ACCUM);
            rd_pipe_addr[0]   <= sram_rd_addr;
            rd_pipe_index[0]  <= RO_ADDR_BITS'(rd_bin_off * CELLS_PER_BIN + rd_cell_idx);
            rd_pipe_last[0]   <= (state == ST_READOUT) &&
                                 (rd_bin_off  == BIN_BITS'(READOUT_BINS - 1)) &&
                                 (rd_cell_idx == CELL_BITS'(CELLS_PER_BIN - 1));

            for (int i = 1; i < COUNTER_READ_LATENCY; i++) begin
                rd_pipe_valid[i]  <= rd_pipe_valid[i-1];
                rd_pipe_is_rmw[i] <= rd_pipe_is_rmw[i-1];
                rd_pipe_addr[i]   <= rd_pipe_addr[i-1];
                rd_pipe_index[i]  <= rd_pipe_index[i-1];
                rd_pipe_last[i]   <= rd_pipe_last[i-1];
            end
        end
    end

    sram_wrapper #(
        .width_p       (COUNTER_BITS),
        .depth_p       (TOTAL_CELLS),
        .PIPELINE_READ (1'b1)
    ) u_counter_mem (
`ifdef USE_POWER_PINS
        .VDD        (VDD),
        .VSS        (VSS),
`endif
        .clk_i      (clk),
        .reset_i    (rst),
        .wr_valid_i (wr_valid_s),
        .wr_data_i  (wr_data_s),
        .wr_addr_i  (wr_addr_s),
        .rd_valid_i (rd_valid_s),
        .rd_addr_i  (rd_addr_s),
        .rd_data_o  (rd_data_s)
    );

    // ------------------------------------------------------------------
    // Combinational address helpers
    // ------------------------------------------------------------------
    logic [BIN_BITS:0]          wr_bin_plus_1;
    logic [BIN_BITS-1:0]        next_wr_bin;
    logic [BIN_BITS:0]          start_calc;
    logic [BIN_COUNT_BITS-1:0]  completed_bins_next;
    logic [BIN_BITS:0]          rd_bin_calc;
    logic [BIN_BITS-1:0]        rd_bin_idx;
    logic [CELL_BITS-1:0]       event_cell_idx;

    always_comb begin
        wr_bin_plus_1 = wr_bin_idx + 1'b1;
        next_wr_bin   = (wr_bin_plus_1 >= (BIN_BITS+1)'(NUM_BINS))
                        ? BIN_BITS'(wr_bin_plus_1 - NUM_BINS)
                        : BIN_BITS'(wr_bin_plus_1);

        start_calc         = (BIN_BITS+1)'(wr_bin_idx) + (BIN_BITS+1)'(NUM_BINS) -
                             (BIN_BITS+1)'((READOUT_BINS - 1));
        snapshot_start_bin = (start_calc >= (BIN_BITS+1)'(NUM_BINS))
                             ? BIN_BITS'(start_calc - NUM_BINS)
                             : BIN_BITS'(start_calc);

        completed_bins_next = (completed_bins < BIN_COUNT_BITS'(NUM_BINS))
                              ? BIN_COUNT_BITS'(completed_bins + 1)
                              : completed_bins;

        rd_bin_calc = snapshot_start_bin + rd_bin_off;
        rd_bin_idx  = (rd_bin_calc >= (BIN_BITS+1)'(NUM_BINS))
                      ? BIN_BITS'(rd_bin_calc - NUM_BINS)
                      : BIN_BITS'(rd_bin_calc);

        event_cell_idx = CELL_BITS'(event_stage_y * GRID_SIZE + event_stage_x);
    end

    // ------------------------------------------------------------------
    // Rollover trigger: timestamp crossed a bin boundary, or explicit force.
    //
    // Uses only registered event_stage_* fields. This is the timing cut.
    // ------------------------------------------------------------------
    logic do_rollover;

    always_comb begin
        do_rollover = !rmw_busy &&
                      (force_rollover_i ||
                       (event_stage_valid && ts_initialized &&
                        ((event_stage_ts - bin_start_ts) >= bin_duration_ts)));
    end

    // ------------------------------------------------------------------
    // SRAM control mux
    //   ST_ACCUM  : RD on staged event, WR when delayed RMW read returns
    //   ST_READOUT: RD each cycle until final address has been issued
    //   ST_CLEAR  : WR each cycle zero-filling the next bin
    // ------------------------------------------------------------------
    logic [MEM_ADDR_BITS-1:0] rd_addr_current;
    logic [MEM_ADDR_BITS-1:0] cl_addr_current;
    logic                     issue_rmw_read;

    assign rd_addr_current = MEM_ADDR_BITS'(rd_bin_idx    * CELLS_PER_BIN + rd_cell_idx);
    assign cl_addr_current = MEM_ADDR_BITS'(clear_bin_idx * CELLS_PER_BIN + clear_cell_idx);

    assign issue_rmw_read = (state == ST_ACCUM) &&
                            event_stage_valid &&
                            !rmw_busy &&
                            !do_rollover;

    always_comb begin
        sram_rd_valid = 1'b0;
        sram_rd_addr  = '0;
        sram_wr_valid = 1'b0;
        sram_wr_addr  = '0;
        sram_wr_data  = '0;

        case (state)
            ST_ACCUM: begin
                // RMW read request for the staged event.
                sram_rd_valid = issue_rmw_read;
                sram_rd_addr  = MEM_ADDR_BITS'(wr_bin_idx * CELLS_PER_BIN + event_cell_idx);

                // RMW writeback when the delayed read response returns.
                sram_wr_valid = rd_resp_valid && rd_resp_is_rmw;
                sram_wr_addr  = rd_resp_addr;
                sram_wr_data  = (sram_rd_data == {COUNTER_BITS{1'b1}})
                                ? sram_rd_data
                                : sram_rd_data + 1'b1;
            end

            ST_READOUT: begin
                // Sequential readout; stop issuing new reads after final address.
                sram_rd_valid = !rd_draining;
                sram_rd_addr  = rd_addr_current;
            end

            ST_CLEAR: begin
                sram_wr_valid = 1'b1;
                sram_wr_addr  = cl_addr_current;
                sram_wr_data  = '0;
            end

            default: ;
        endcase
    end

    // ------------------------------------------------------------------
    // event_ready:
    //
    // Accept one new event only when:
    //   - in accumulate state
    //   - no staged event waiting
    //   - no RMW transaction in flight
    //
    // Once accepted, the event is processed from event_stage_* registers.
    // ------------------------------------------------------------------
    assign event_ready = (state == ST_ACCUM) && !rmw_busy && !event_stage_valid;

    // ------------------------------------------------------------------
    // Readout outputs aligned with delayed SRAM data
    // ------------------------------------------------------------------
    assign readout_valid = rd_resp_is_readout;
    assign readout_data  = sram_rd_data;
    assign readout_index = rd_resp_index;
    assign readout_last  = rd_resp_is_readout && rd_resp_last;

    // ------------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state          <= ST_CLEAR;
            bin_start_ts   <= '0;
            ts_initialized <= 1'b0;
            wr_bin_idx     <= '0;
            clear_bin_idx  <= '0;
            completed_bins <= '0;
            rd_bin_off     <= '0;
            rd_cell_idx    <= '0;
            clear_cell_idx <= '0;
            readout_start  <= 1'b0;

            event_stage_valid <= 1'b0;
            event_stage_x     <= '0;
            event_stage_y     <= '0;
            event_stage_ts    <= '0;

            rd_draining <= 1'b0;
        end else begin
            readout_start <= 1'b0;

            // Capture a newly accepted event into the timing-isolating stage.
            // This is based on the old-cycle event_ready value, so it captures
            // exactly when the upstream handshake succeeds.
            if (event_ready && event_valid) begin
                event_stage_valid <= 1'b1;
                event_stage_x     <= event_x;
                event_stage_y     <= event_y;
                event_stage_ts    <= ts_in;
            end

            case (state)
                // ----------------------------------------------------------
                ST_ACCUM: begin
                    // If this cycle issued an RMW read for the staged event,
                    // that event has now been consumed. The RMW writeback is
                    // protected by rmw_busy until the delayed read response
                    // returns and the staged write enters the wrapper.
                    if (issue_rmw_read)
                        event_stage_valid <= 1'b0;

                    // Latch bin start on the first staged CD event.
                    // The first event is also allowed to issue an RMW read in
                    // the same cycle because do_rollover is false until
                    // ts_initialized is set.
                    if (event_stage_valid && !ts_initialized) begin
                        ts_initialized <= 1'b1;
                        bin_start_ts   <= event_stage_ts;
                    end else if (do_rollover) begin
                        if (ts_initialized)
                            bin_start_ts <= bin_start_ts + bin_duration_ts;

                        clear_bin_idx  <= next_wr_bin;
                        completed_bins <= completed_bins_next;

                        if (completed_bins_next >= BIN_COUNT_BITS'(READOUT_BINS)) begin
                            if (readout_ready) begin
                                state         <= ST_READOUT;
                                rd_bin_off    <= '0;
                                rd_cell_idx   <= '0;
                                rd_draining   <= 1'b0;
                                readout_start <= 1'b1;
                            end else begin
                                state <= ST_WAIT_RD;
                            end
                        end else begin
                            state          <= ST_CLEAR;
                            clear_cell_idx <= '0;
                        end
                    end
                end

                // ----------------------------------------------------------
                ST_WAIT_RD: begin
                    if (readout_ready) begin
                        state         <= ST_READOUT;
                        rd_bin_off    <= '0;
                        rd_cell_idx   <= '0;
                        rd_draining   <= 1'b0;
                        readout_start <= 1'b1;
                    end
                end

                // ----------------------------------------------------------
                ST_READOUT: begin
                    if (rd_draining) begin
                        // Stay here until the delayed final read response appears.
                        if (rd_resp_is_readout && rd_resp_last) begin
                            rd_draining    <= 1'b0;
                            state          <= ST_CLEAR;
                            clear_cell_idx <= '0;
                        end
                    end else begin
                        // Determine whether this is the last address to issue.
                        if ((rd_bin_off  == BIN_BITS'(READOUT_BINS - 1)) &&
                            (rd_cell_idx == CELL_BITS'(CELLS_PER_BIN - 1))) begin
                            rd_draining <= 1'b1;
                        end else if (rd_cell_idx == CELL_BITS'(CELLS_PER_BIN - 1)) begin
                            rd_cell_idx <= '0;
                            rd_bin_off  <= rd_bin_off + 1'b1;
                        end else begin
                            rd_cell_idx <= rd_cell_idx + 1'b1;
                        end
                    end
                end

                // ----------------------------------------------------------
                ST_CLEAR: begin
                    // SRAM clear write issued combinationally above.
                    if (clear_cell_idx == CELL_BITS'(CELLS_PER_BIN - 1)) begin
                        clear_cell_idx <= '0;
                        wr_bin_idx     <= clear_bin_idx;
                        state          <= ST_ACCUM;
                    end else begin
                        clear_cell_idx <= clear_cell_idx + 1'b1;
                    end
                end

                default: begin
                    state <= ST_ACCUM;
                end
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Debug bus connections
    // ------------------------------------------------------------------
    assign vox_bin_dbg[0]     = event_ready;
    assign vox_bin_dbg[1]     = readout_start;
    assign vox_bin_dbg[2]     = readout_valid;
    assign vox_bin_dbg[3]     = readout_last;
    assign vox_bin_dbg[14:4]  = readout_index;
    assign vox_bin_dbg[30:15] = readout_data;

endmodule