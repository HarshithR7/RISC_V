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
//
// Phase 7 (SMT, 2 threads): this bank is *shared* across both threads --
// every entry carries a 1-bit thread ID alongside its ROB tag, since ROB
// tags are only unique *within* a thread (each thread has its own ROB
// instance -- see riscv64_ooo_proc.v's header). This breaks the single
// global "oldest ROB tag" ordering Phase 1-6 relied on: there is no
// meaningful cross-thread notion of age between two independent
// programs' instructions. Issue priority is redefined accordingly:
// an entry whose tag is *exactly* its own thread's current ROB head
// (checked against whichever of rob_head_tag0/rob_head_tag1 matches its
// stored tid) always wins over any entry that isn't -- this is what
// actually matters for correctness (a thread's own head, once ready,
// must never be needlessly blocked from broadcasting, or that thread's
// commit stalls forever), not a precise cross-thread age ranking, which
// wouldn't mean anything anyway. Among entries of the same priority
// class (both "at head" or both "not"), ties break by fixed lowest-index
// -- the same bounded, documented-starvation-risk simplification this
// bank already used before Phase 3 exposed why *unbounded* age blindness
// was a real problem (see the git history / README for that finding);
// unlike that case, there's no unbounded-stream scenario here since
// "at head" entries are drained with real priority.
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
    // Phase 1/2's meaning. alloc_tid/alloc2_tid: which thread this entry
    // belongs to (Phase 7).
    input alloc_req,
    input alloc_tid,
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
    input alloc2_tid,
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
    // Two independent buses (widened-CDB support, see
    // riscv64_ooo_proc.v's 2-wide arbiter) -- the arbiter guarantees
    // cdbA and cdbB always carry different (tid,tag) pairs in the same
    // cycle, so a given waiting operand can match at most one of the
    // two. Phase 7 correctness fix: ROB tags are only unique *within* a
    // thread (two independent ROBs both number their entries 0..DEPTH-1),
    // so matching on tag alone is a real, not theoretical, bug -- a
    // waiting entry on thread 0 can and will occasionally share its
    // numeric tag with an unrelated thread-1 producer's tag, and without
    // a tid check would wrongly capture thread 1's value as its own
    // operand. cdbA_tid/cdbB_tid (each requester's own tid, already
    // needed for ROB mark routing at the top level) close this: every
    // snoop compares the FULL (tid, tag) pair, not the tag alone.
    input cdbA_valid,
    input cdbA_tid,
    input [TAG_BITS-1:0] cdbA_tag,
    input [63:0] cdbA_value,
    input cdbB_valid,
    input cdbB_tid,
    input [TAG_BITS-1:0] cdbB_tag,
    input [63:0] cdbB_value,

    // CDB request: this bank has a fully-ready entry it wants to
    // broadcast the ALU result of. The arbiter can grant this bank
    // either bus per cycle; only on req_grant does this bank actually
    // free the entry (which bus doesn't matter to this module). req_tid
    // tells the top-level arbiter which thread's ROB to route the mark
    // to.
    output req_valid,
    output req_tid,
    output [TAG_BITS-1:0] req_tag,
    output [63:0] req_value,
    input req_grant,

    // Phase 7: one head tag per thread, for the is-head issue-priority
    // check above.
    input [TAG_BITS-1:0] rob_head_tag0,
    input [TAG_BITS-1:0] rob_head_tag1,

    // Phase 2 misprediction squash, now thread-aware (Phase 7): two fully
    // independent ports, one per thread, rather than one port muxed by a
    // tid selector -- branch resolution (and so misprediction detection)
    // is asynchronous to which thread is currently dispatching, so both
    // threads' outstanding branches can resolve, and both mispredict, in
    // the exact same cycle. A single muxed port could only ever squash
    // one of them that cycle, leaving the other thread's wrong-path
    // entries stale in a shared bank -- exactly the kind of "rare but
    // real" gap this project has already found and fixed more than once
    // (see the README's Bugs found section), so it's fixed here before
    // ever shipping, not after.
    input squash0_valid,
    input [TAG_BITS-1:0] squash0_tag,
    input squash1_valid,
    input [TAG_BITS-1:0] squash1_tag
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
    reg tid_arr    [0:DEPTH-1];
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

    // ---- Issue: is-own-thread-head-first, else fixed lowest-index
    // (Phase 7 -- see module header) ----
    function entry_is_head;
        input t;          // tid_arr[i]
        input [TAG_BITS-1:0] tg; // dest_tag[i]
        begin
            entry_is_head = t ? (tg == rob_head_tag1) : (tg == rob_head_tag0);
        end
    endfunction

    integer ri;
    reg have_ready;
    reg [31:0] ready_idx;
    reg ready_is_head;
    always @(*) begin
        have_ready = 1'b0;
        ready_idx = 0;
        ready_is_head = 1'b0;
        for (ri = 0; ri < DEPTH; ri = ri + 1)
            if (busy[ri] && s1_ready[ri] && s2_ready[ri]) begin
                if (!have_ready || (entry_is_head(tid_arr[ri], dest_tag[ri]) && !ready_is_head)) begin
                    have_ready = 1'b1;
                    ready_idx = ri;
                    ready_is_head = entry_is_head(tid_arr[ri], dest_tag[ri]);
                end
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
    assign req_tid   = tid_arr[ready_idx];
    assign req_tag   = dest_tag[ready_idx];
    assign req_value = word_op_arr[ready_idx] ? alu_word_result_ext : alu_full_result;

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
            if (cdbA_valid || cdbB_valid) begin
                for (fi = 0; fi < DEPTH; fi = fi + 1) begin
                    if (busy[fi] && !s1_ready[fi] && cdbA_valid && tid_arr[fi] == cdbA_tid && s1_tag[fi] == cdbA_tag) begin
                        s1_ready[fi] <= 1'b1;
                        s1_val[fi]   <= cdbA_value;
                    end else if (busy[fi] && !s1_ready[fi] && cdbB_valid && tid_arr[fi] == cdbB_tid && s1_tag[fi] == cdbB_tag) begin
                        s1_ready[fi] <= 1'b1;
                        s1_val[fi]   <= cdbB_value;
                    end
                    if (busy[fi] && !s2_ready[fi] && cdbA_valid && tid_arr[fi] == cdbA_tid && s2_tag[fi] == cdbA_tag) begin
                        s2_ready[fi] <= 1'b1;
                        s2_val[fi]   <= cdbA_value;
                    end else if (busy[fi] && !s2_ready[fi] && cdbB_valid && tid_arr[fi] == cdbB_tid && s2_tag[fi] == cdbB_tag) begin
                        s2_ready[fi] <= 1'b1;
                        s2_val[fi]   <= cdbB_value;
                    end
                end
            end

            if (do_alloc1) begin
                busy[free_idx]     <= 1'b1;
                tid_arr[free_idx]  <= alloc_tid;
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
                tid_arr[free_idx2]  <= alloc2_tid;
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

            if (squash0_valid) begin
                for (fi = 0; fi < DEPTH; fi = fi + 1)
                    if (busy[fi] && !tid_arr[fi] &&
                        (dest_tag[fi] - rob_head_tag0) > (squash0_tag - rob_head_tag0))
                        busy[fi] <= 1'b0;
            end
            if (squash1_valid) begin
                for (fi = 0; fi < DEPTH; fi = fi + 1)
                    if (busy[fi] && tid_arr[fi] &&
                        (dest_tag[fi] - rob_head_tag1) > (squash1_tag - rob_head_tag1))
                        busy[fi] <= 1'b0;
            end
        end
    end
endmodule
