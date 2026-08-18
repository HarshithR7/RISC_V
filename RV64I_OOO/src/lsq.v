`timescale 1ns / 1ps
// Load-Store Queue: DEPTH entries, holding both loads and stores in
// program order (age tracked via each entry's captured ROB tag, same
// wraparound-correct age() comparison the top level's CDB arbiter uses).
//
// Loads execute out-of-order once their address is known, *if* it's safe:
// for a candidate load L, every older (still-resident) store S blocks it
// unless S's own address is already known AND provably doesn't overlap
// L's (doubleword-granularity compare, per the original sizing plan --
// Phase 1 does no finer-grained byte-range disambiguation). An older
// store with an unresolved address always blocks, since aliasing can't be
// ruled out. This project's Phase 1 does no store-to-load forwarding: an
// aliasing older store simply blocks the load until that store leaves the
// LSQ, which only happens at *commit* (see below) -- so "block until S
// commits" falls out for free from "S is still resident."
//
// Stores never broadcast a value (no destination register); once both
// their address and data operands are ready, they become commit-eligible
// via `store_ready`/`store_tag` (an array, since -- unlike branch_rs's
// single entry -- multiple stores can legitimately become ready the same
// cycle; the top level fans these into rob.v's extra_mark ports, which
// tolerates exactly this "several simultaneous, always-distinct tags"
// case by construction). The actual memory write happens only at commit
// (driven by the top level from this module's commit_addr/data/func3),
// preserving precise state exactly like the architectural register file.
//
// Only one data-memory access (the shared read/write address port) can be
// used per cycle: `mem_port_busy` (asserted by the top level whenever a
// store is committing its write this cycle) stalls load issue for
// exactly that one cycle rather than racing the two accesses -- a real,
// deliberately simple structural-hazard resolution.
module lsq #(
    parameter DEPTH = 4,
    parameter TAG_BITS = 3
)(
    input clk,
    input reset,

    input alloc_req,
    input alloc_is_store,
    input [2:0] alloc_func3,
    input [63:0] alloc_imm,
    input alloc_base_ready, input [63:0] alloc_base_val, input [TAG_BITS-1:0] alloc_base_tag,
    input alloc_data_ready, input [63:0] alloc_data_val, input [TAG_BITS-1:0] alloc_data_tag,
    input [TAG_BITS-1:0] alloc_dest_tag,
    output full,

    // Phase 3: lane-1 allocation port, same convention as alu_rs.v.
    input alloc2_req,
    input alloc2_is_store,
    input [2:0] alloc2_func3,
    input [63:0] alloc2_imm,
    input alloc2_base_ready, input [63:0] alloc2_base_val, input [TAG_BITS-1:0] alloc2_base_tag,
    input alloc2_data_ready, input [63:0] alloc2_data_val, input [TAG_BITS-1:0] alloc2_data_tag,
    input [TAG_BITS-1:0] alloc2_dest_tag,
    output has_2_free,

    input cdb_valid,
    input [TAG_BITS-1:0] cdb_tag,
    input [63:0] cdb_value,

    input [TAG_BITS-1:0] rob_head_tag,   // age() reference, same convention as the top-level CDB arbiter

    // Load broadcast request (CDB) -- same req/grant convention as
    // alu_rs.v/mul_rs.v.
    output req_valid,
    output [TAG_BITS-1:0] req_tag,
    output [63:0] req_value,
    input req_grant,

    // Shared data-memory read port (combinational).
    input mem_port_busy,
    output mem_read_req,
    output [63:0] mem_read_addr,
    output [2:0] mem_read_func3,
    input [63:0] mem_read_data,

    // Per-slot store-readiness, fanned into rob.v's extra_mark ports by
    // the top level -- see header for why this is an array, not a single
    // pulse.
    output [DEPTH-1:0] store_ready,
    output [DEPTH*TAG_BITS-1:0] store_ready_tag_flat,

    // Commit-time store writeback: valid combinationally whenever some
    // resident store's dest_tag matches rob_head_tag (at most one, by the
    // ROB's own tag-uniqueness invariant). commit_fire (=rob_commit_req
    // && rob_head_is_store, supplied by the top level) tells this module
    // to vacate that matched entry this edge.
    output commit_match,
    output [63:0] commit_addr,
    output [63:0] commit_data,
    output [2:0] commit_func3,
    input commit_fire,

    // Phase 2 misprediction squash -- same convention as alu_rs.v.
    input squash_valid,
    input [TAG_BITS-1:0] squash_tag
);
    reg busy        [0:DEPTH-1];
    reg is_store_arr[0:DEPTH-1];
    reg [2:0] func3  [0:DEPTH-1];
    reg [63:0] imm   [0:DEPTH-1];
    reg base_ready   [0:DEPTH-1];
    reg [63:0] base_val[0:DEPTH-1];
    reg [TAG_BITS-1:0] base_tag[0:DEPTH-1];
    reg data_ready   [0:DEPTH-1];
    reg [63:0] data_val[0:DEPTH-1];
    reg [TAG_BITS-1:0] data_tag[0:DEPTH-1];
    reg [TAG_BITS-1:0] dest_tag[0:DEPTH-1];

    function [TAG_BITS-1:0] age;
        input [TAG_BITS-1:0] tag;
        begin
            age = tag - rob_head_tag;
        end
    endfunction

    // ---- Allocation: first free slot for lane 0, second-lowest (distinct
    // from lane 0's own pick) for lane 1 -- same pattern as alu_rs.v ----
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
        // going to consume it (alloc_req, not just have_free) -- see
        // alu_rs.v's identical fix for the deadlock this caused when
        // omitted.
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

    // ---- Per-slot address (valid only where base_ready[i]) ----
    reg [63:0] entry_addr [0:DEPTH-1];
    integer ai;
    always @(*) begin
        for (ai = 0; ai < DEPTH; ai = ai + 1)
            entry_addr[ai] = base_val[ai] + imm[ai];
    end

    // ---- Disambiguation: which load slots are blocked by an older,
    // still-resident store this cycle ----
    reg blocked [0:DEPTH-1];
    integer li, si;
    always @(*) begin
        for (li = 0; li < DEPTH; li = li + 1) begin
            blocked[li] = 1'b0;
            if (busy[li] && !is_store_arr[li]) begin
                for (si = 0; si < DEPTH; si = si + 1) begin
                    if (si != li && busy[si] && is_store_arr[si] && (age(dest_tag[si]) < age(dest_tag[li]))) begin
                        if (!base_ready[si])
                            blocked[li] = 1'b1;
                        else if (entry_addr[si][63:3] == entry_addr[li][63:3])
                            blocked[li] = 1'b1;
                    end
                end
            end
        end
    end

    // ---- Issue: first ready, unblocked load (fixed lowest-index priority) ----
    integer ri;
    reg have_ready;
    reg [31:0] ready_idx;
    always @(*) begin
        have_ready = 1'b0;
        ready_idx = 0;
        if (!mem_port_busy) begin
            for (ri = DEPTH - 1; ri >= 0; ri = ri - 1)
                if (busy[ri] && !is_store_arr[ri] && base_ready[ri] && !blocked[ri]) begin
                    have_ready = 1'b1;
                    ready_idx = ri;
                end
        end
    end

    assign mem_read_req   = have_ready;
    assign mem_read_addr  = entry_addr[ready_idx];
    assign mem_read_func3 = func3[ready_idx];

    assign req_valid = have_ready;
    assign req_tag   = dest_tag[ready_idx];
    assign req_value = mem_read_data;

    // ---- Per-slot store readiness (address + data both known) ----
    genvar gi;
    generate
        for (gi = 0; gi < DEPTH; gi = gi + 1) begin : store_ready_gen
            assign store_ready[gi] = busy[gi] && is_store_arr[gi] && base_ready[gi] && data_ready[gi];
            assign store_ready_tag_flat[(gi+1)*TAG_BITS-1 -: TAG_BITS] = dest_tag[gi];
        end
    endgenerate

    // ---- Commit-time store lookup (at most one match, by construction) ----
    reg commit_match_r;
    reg [63:0] commit_addr_r, commit_data_r;
    reg [2:0] commit_func3_r;
    integer ci;
    always @(*) begin
        commit_match_r = 1'b0;
        commit_addr_r = 64'b0;
        commit_data_r = 64'b0;
        commit_func3_r = 3'b0;
        for (ci = 0; ci < DEPTH; ci = ci + 1) begin
            if (busy[ci] && is_store_arr[ci] && dest_tag[ci] == rob_head_tag) begin
                commit_match_r = 1'b1;
                commit_addr_r = entry_addr[ci];
                commit_data_r = data_val[ci];
                commit_func3_r = func3[ci];
            end
        end
    end
    assign commit_match = commit_match_r;
    assign commit_addr  = commit_addr_r;
    assign commit_data  = commit_data_r;
    assign commit_func3 = commit_func3_r;

    integer vi;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (vi = 0; vi < DEPTH; vi = vi + 1)
                busy[vi] <= 1'b0;
        end else begin
            if (cdb_valid) begin
                for (vi = 0; vi < DEPTH; vi = vi + 1) begin
                    if (busy[vi] && !base_ready[vi] && base_tag[vi] == cdb_tag) begin
                        base_ready[vi] <= 1'b1;
                        base_val[vi]   <= cdb_value;
                    end
                    if (busy[vi] && !data_ready[vi] && data_tag[vi] == cdb_tag) begin
                        data_ready[vi] <= 1'b1;
                        data_val[vi]   <= cdb_value;
                    end
                end
            end

            if (do_alloc1) begin
                busy[free_idx]        <= 1'b1;
                is_store_arr[free_idx]<= alloc_is_store;
                func3[free_idx]       <= alloc_func3;
                imm[free_idx]         <= alloc_imm;
                base_ready[free_idx]  <= alloc_base_ready;
                base_val[free_idx]    <= alloc_base_val;
                base_tag[free_idx]    <= alloc_base_tag;
                data_ready[free_idx]  <= alloc_data_ready;
                data_val[free_idx]    <= alloc_data_val;
                data_tag[free_idx]    <= alloc_data_tag;
                dest_tag[free_idx]    <= alloc_dest_tag;
            end
            if (do_alloc2) begin
                busy[free_idx2]        <= 1'b1;
                is_store_arr[free_idx2]<= alloc2_is_store;
                func3[free_idx2]       <= alloc2_func3;
                imm[free_idx2]         <= alloc2_imm;
                base_ready[free_idx2]  <= alloc2_base_ready;
                base_val[free_idx2]    <= alloc2_base_val;
                base_tag[free_idx2]    <= alloc2_base_tag;
                data_ready[free_idx2]  <= alloc2_data_ready;
                data_val[free_idx2]    <= alloc2_data_val;
                data_tag[free_idx2]    <= alloc2_data_tag;
                dest_tag[free_idx2]    <= alloc2_dest_tag;
            end

            if (req_valid && req_grant) begin
                busy[ready_idx] <= 1'b0;
            end

            if (commit_fire) begin
                for (vi = 0; vi < DEPTH; vi = vi + 1)
                    if (busy[vi] && is_store_arr[vi] && dest_tag[vi] == rob_head_tag)
                        busy[vi] <= 1'b0;
            end

            if (squash_valid) begin
                for (vi = 0; vi < DEPTH; vi = vi + 1)
                    if (busy[vi] && age(dest_tag[vi]) > age(squash_tag))
                        busy[vi] <= 1'b0;
            end
        end
    end
endmodule
