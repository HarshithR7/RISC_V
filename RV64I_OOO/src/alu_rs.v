`timescale 1ns / 1ps
// ALU reservation-station bank + the (combinational, 1-cycle) ALU itself,
// combined in one module -- this project prefers concrete, purpose-built
// logic over a maximally generic "res_station_bank" reused unchanged
// across wildly different functional-unit latencies/behaviors (see
// RV64I_OOO's own commit-order and LSQ design notes for the same
// instinct). ALU logic is lifted verbatim from RV64I/src/execute.v's
// full/word-result case statements.
//
// Allocation picks the lowest free index (order doesn't affect
// correctness or fairness there -- every free slot is equally "new").
// Issue arbitration among *ready* entries, however, is oldest-ROB-tag-
// first (the same age()-relative-to-rob_head_tag policy the CDB arbiter
// and lsq.v's disambiguation already use) -- this used to be fixed
// lowest-slot-index-first instead, documented at the time as a "never
// actually starves anything in practice" simplification, until Phase 3's
// 2-wide dispatch (see riscv64_ooo_proc.v) started allocating in bursts
// of two: a burstier fill pattern measurably increased the odds of an
// older entry landing in a high-index slot while a stream of newer,
// faster-to-ready entries kept re-filling low-index slots and winning
// issue priority every cycle -- a real, measured slowdown (found via a
// Phase 4 benchmark showing dual-issue running *slower* than single-issue
// on fully independent work, which should never happen), not a
// theoretical concern anymore.
module alu_rs #(
    parameter DEPTH = 4,
    parameter TAG_BITS = 3
)(
    input clk,
    input reset,

    // Dispatch: up to two entries per cycle (Phase 3, 2-wide dispatch) --
    // lane 0 via alloc_req/alloc_*, lane 1 via alloc2_req/alloc2_* below.
    // has_2_free tells the caller whether *both* could be accepted this
    // cycle (used to decide whether lane 1 may dual-issue into this same
    // bank alongside lane 0); full (=not even one free) is unchanged from
    // Phase 1/2's meaning.
    input alloc_req,
    input [3:0] alloc_op,
    input alloc_word_op,
    input alloc_src1_ready,
    input [63:0] alloc_src1_val,
    input [TAG_BITS-1:0] alloc_src1_tag,
    input alloc_src2_ready,
    input [63:0] alloc_src2_val,
    input [TAG_BITS-1:0] alloc_src2_tag,
    input [TAG_BITS-1:0] alloc_dest_tag,
    output full,

    input alloc2_req,
    input [3:0] alloc2_op,
    input alloc2_word_op,
    input alloc2_src1_ready,
    input [63:0] alloc2_src1_val,
    input [TAG_BITS-1:0] alloc2_src1_tag,
    input alloc2_src2_ready,
    input [63:0] alloc2_src2_val,
    input [TAG_BITS-1:0] alloc2_src2_tag,
    input [TAG_BITS-1:0] alloc2_dest_tag,
    output has_2_free,

    // CDB snoop: capture a broadcast operand this bank is waiting on.
    input cdb_valid,
    input [TAG_BITS-1:0] cdb_tag,
    input [63:0] cdb_value,

    // CDB request: this bank has a fully-ready entry it wants to
    // broadcast the ALU result of. The arbiter grants at most one
    // requester per cycle across all banks; only on req_grant does this
    // bank actually free the entry.
    output req_valid,
    output [TAG_BITS-1:0] req_tag,
    output [63:0] req_value,
    input req_grant,

    // Phase 2 misprediction squash: clear any entry strictly younger than
    // squash_tag (a mispredicted branch's own tag), using the same
    // wraparound-safe age-relative-to-rob_head_tag comparison as the CDB
    // arbiter and lsq.v's own disambiguation. squash_tag itself (the
    // branch) is never an entry in this bank, so no age==0 case to worry
    // about excluding.
    input [TAG_BITS-1:0] rob_head_tag,
    input squash_valid,
    input [TAG_BITS-1:0] squash_tag
);
    localparam ALU_ADD  = 4'b0010;
    localparam ALU_SUB  = 4'b1010;
    localparam ALU_AND  = 4'b0100;
    localparam ALU_OR   = 4'b0101;
    localparam ALU_XOR  = 4'b0011;
    localparam ALU_SLL  = 4'b0110;
    localparam ALU_SRL  = 4'b0111;
    localparam ALU_SRA  = 4'b1000;
    localparam ALU_SLT  = 4'b1011;
    localparam ALU_SLTU = 4'b1100;

    reg busy       [0:DEPTH-1];
    reg [3:0] op    [0:DEPTH-1];
    reg word_op_arr[0:DEPTH-1];
    reg s1_ready   [0:DEPTH-1];
    reg [63:0] s1_val[0:DEPTH-1];
    reg [TAG_BITS-1:0] s1_tag[0:DEPTH-1];
    reg s2_ready   [0:DEPTH-1];
    reg [63:0] s2_val[0:DEPTH-1];
    reg [TAG_BITS-1:0] s2_tag[0:DEPTH-1];
    reg [TAG_BITS-1:0] dest_tag[0:DEPTH-1];

    // ---- Allocation: first free slot for lane 0, second-lowest (distinct
    // from lane 0's own pick) for lane 1 ----
    integer fi;
    reg [DEPTH-1:0] free_mask;
    reg have_free;
    reg [31:0] free_idx;
    reg have_free2;
    reg [31:0] free_idx2;
    always @(*) begin
        have_free = 1'b0;
        free_idx = 0;
        for (fi = 0; fi < DEPTH; fi = fi + 1)
            free_mask[fi] = !busy[fi];
        for (fi = DEPTH - 1; fi >= 0; fi = fi - 1)
            if (free_mask[fi]) begin
                have_free = 1'b1;
                free_idx = fi;
            end

        // Only reserve free_idx away from lane 1 if lane 0 is actually
        // going to consume it this cycle (alloc_req, not just have_free)
        // -- otherwise, when lane 0 doesn't want this bank at all (e.g.
        // it's a branch and this is alu_rs), free_idx is a perfectly
        // available slot for lane 1 alone, not something to skip past.
        // Getting this wrong silently drops lane 1's allocation (its
        // ROB/RAT entries still get created at the top level, believing
        // dispatch succeeded) while creating no reservation-station entry
        // for it anywhere -- a permanent deadlock, found by tracing a
        // ROB head stuck on a tag nothing was ever going to broadcast.
        have_free2 = 1'b0;
        free_idx2 = 0;
        for (fi = DEPTH - 1; fi >= 0; fi = fi - 1)
            if (free_mask[fi] && !(alloc_req && have_free && fi == free_idx)) begin
                have_free2 = 1'b1;
                free_idx2 = fi;
            end
    end
    assign full = !have_free;
    assign has_2_free = have_free2;

    wire do_alloc1 = alloc_req  && have_free;
    wire do_alloc2 = alloc2_req && have_free2;

    // ---- Issue: oldest ready busy entry (see header for why this is
    // age-ordered, not fixed-index, as of Phase 3) ----
    integer ri;
    reg have_ready;
    reg [31:0] ready_idx;
    reg [TAG_BITS-1:0] ready_best_age;
    always @(*) begin
        have_ready = 1'b0;
        ready_idx = 0;
        ready_best_age = 0;
        for (ri = 0; ri < DEPTH; ri = ri + 1)
            if (busy[ri] && s1_ready[ri] && s2_ready[ri] &&
                (!have_ready || age(dest_tag[ri]) < ready_best_age)) begin
                have_ready = 1'b1;
                ready_idx = ri;
                ready_best_age = age(dest_tag[ri]);
            end
    end

    reg [63:0] alu_full_result, alu_word_result_ext;
    reg [31:0] alu_word_result;
    always @(*) begin
        case (op[ready_idx])
            ALU_ADD:  alu_full_result = s1_val[ready_idx] + s2_val[ready_idx];
            ALU_SUB:  alu_full_result = s1_val[ready_idx] - s2_val[ready_idx];
            ALU_AND:  alu_full_result = s1_val[ready_idx] & s2_val[ready_idx];
            ALU_OR:   alu_full_result = s1_val[ready_idx] | s2_val[ready_idx];
            ALU_XOR:  alu_full_result = s1_val[ready_idx] ^ s2_val[ready_idx];
            ALU_SLL:  alu_full_result = s1_val[ready_idx] << s2_val[ready_idx][5:0];
            ALU_SRL:  alu_full_result = s1_val[ready_idx] >> s2_val[ready_idx][5:0];
            ALU_SRA:  alu_full_result = $signed(s1_val[ready_idx]) >>> s2_val[ready_idx][5:0];
            ALU_SLT:  alu_full_result = ($signed(s1_val[ready_idx]) < $signed(s2_val[ready_idx])) ? 64'd1 : 64'd0;
            ALU_SLTU: alu_full_result = (s1_val[ready_idx] < s2_val[ready_idx]) ? 64'd1 : 64'd0;
            default:  alu_full_result = 64'b0;
        endcase
        case (op[ready_idx])
            ALU_ADD:  alu_word_result = s1_val[ready_idx][31:0] + s2_val[ready_idx][31:0];
            ALU_SUB:  alu_word_result = s1_val[ready_idx][31:0] - s2_val[ready_idx][31:0];
            ALU_SLL:  alu_word_result = s1_val[ready_idx][31:0] << s2_val[ready_idx][4:0];
            ALU_SRL:  alu_word_result = s1_val[ready_idx][31:0] >> s2_val[ready_idx][4:0];
            ALU_SRA:  alu_word_result = $signed(s1_val[ready_idx][31:0]) >>> s2_val[ready_idx][4:0];
            default:  alu_word_result = 32'b0;
        endcase
        alu_word_result_ext = {{32{alu_word_result[31]}}, alu_word_result};
    end

    assign req_valid = have_ready;
    assign req_tag   = dest_tag[ready_idx];
    assign req_value = word_op_arr[ready_idx] ? alu_word_result_ext : alu_full_result;

    function [TAG_BITS-1:0] age;
        input [TAG_BITS-1:0] t;
        begin
            age = t - rob_head_tag;
        end
    endfunction

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (fi = 0; fi < DEPTH; fi = fi + 1)
                busy[fi] <= 1'b0;
        end else begin
            // CDB snoop: any waiting entry captures a matching broadcast.
            // Written before allocation below so a same-cycle "dispatch a
            // new entry whose source is exactly this cycle's CDB result"
            // still sees the *old* (not-yet-allocated) entries correctly;
            // the new entry's own ready bits are set directly from
            // alloc_src*_ready at allocation instead of via snooping its
            // own not-yet-existing tag.
            if (cdb_valid) begin
                for (fi = 0; fi < DEPTH; fi = fi + 1) begin
                    if (busy[fi] && !s1_ready[fi] && s1_tag[fi] == cdb_tag) begin
                        s1_ready[fi] <= 1'b1;
                        s1_val[fi]   <= cdb_value;
                    end
                    if (busy[fi] && !s2_ready[fi] && s2_tag[fi] == cdb_tag) begin
                        s2_ready[fi] <= 1'b1;
                        s2_val[fi]   <= cdb_value;
                    end
                end
            end

            if (do_alloc1) begin
                busy[free_idx]     <= 1'b1;
                op[free_idx]       <= alloc_op;
                word_op_arr[free_idx] <= alloc_word_op;
                s1_ready[free_idx] <= alloc_src1_ready;
                s1_val[free_idx]   <= alloc_src1_val;
                s1_tag[free_idx]   <= alloc_src1_tag;
                s2_ready[free_idx] <= alloc_src2_ready;
                s2_val[free_idx]   <= alloc_src2_val;
                s2_tag[free_idx]   <= alloc_src2_tag;
                dest_tag[free_idx] <= alloc_dest_tag;
            end
            if (do_alloc2) begin
                busy[free_idx2]     <= 1'b1;
                op[free_idx2]       <= alloc2_op;
                word_op_arr[free_idx2] <= alloc2_word_op;
                s1_ready[free_idx2] <= alloc2_src1_ready;
                s1_val[free_idx2]   <= alloc2_src1_val;
                s1_tag[free_idx2]   <= alloc2_src1_tag;
                s2_ready[free_idx2] <= alloc2_src2_ready;
                s2_val[free_idx2]   <= alloc2_src2_val;
                s2_tag[free_idx2]   <= alloc2_src2_tag;
                dest_tag[free_idx2] <= alloc2_dest_tag;
            end

            if (req_valid && req_grant) begin
                busy[ready_idx] <= 1'b0;
            end

            if (squash_valid) begin
                for (fi = 0; fi < DEPTH; fi = fi + 1)
                    if (busy[fi] && age(dest_tag[fi]) > age(squash_tag))
                        busy[fi] <= 1'b0;
            end
        end
    end
endmodule
