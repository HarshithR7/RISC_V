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

    // Phase 16: a third allocation port, lane 2 of 3-wide dispatch. Same
    // convention as alloc2/has_2_free, generalized one step further --
    // free_idx3 is the next-lowest free slot distinct from whichever of
    // free_idx/free_idx2 were actually consumed this cycle.
    input alloc3_req,
    input alloc3_tid,
    input [3:0] alloc3_op,
    input alloc3_word_op,
    input alloc3_src1_ready,
    input [63:0] alloc3_src1_val,
    input [TAG_BITS-1:0] alloc3_src1_tag,
    input alloc3_src2_ready,
    input [63:0] alloc3_src2_val,
    input [TAG_BITS-1:0] alloc3_src2_tag,
    input [TAG_BITS-1:0] alloc3_dest_tag,
    output has_3_free,

    // CDB snoop: capture a broadcast operand this bank is waiting on.
    // Three independent buses (Phase 16 widens this from two to three --
    // see riscv64_ooo_proc.v's 3-wide arbiter) -- the arbiter guarantees
    // cdbA/cdbB/cdbC always carry different (tid,tag) pairs in the same
    // cycle, so a given waiting operand can match at most one of the
    // three. Phase 7 correctness fix: ROB tags are only unique *within* a
    // thread (two independent ROBs both number their entries 0..DEPTH-1),
    // so matching on tag alone is a real, not theoretical, bug -- a
    // waiting entry on thread 0 can and will occasionally share its
    // numeric tag with an unrelated thread-1 producer's tag, and without
    // a tid check would wrongly capture thread 1's value as its own
    // operand. cdbA_tid/cdbB_tid/cdbC_tid (each requester's own tid,
    // already needed for ROB mark routing at the top level) close this:
    // every snoop compares the FULL (tid, tag) pair, not the tag alone.
    input cdbA_valid,
    input cdbA_tid,
    input [TAG_BITS-1:0] cdbA_tag,
    input [63:0] cdbA_value,
    input cdbB_valid,
    input cdbB_tid,
    input [TAG_BITS-1:0] cdbB_tag,
    input [63:0] cdbB_value,
    input cdbC_valid,
    input cdbC_tid,
    input [TAG_BITS-1:0] cdbC_tag,
    input [63:0] cdbC_value,

    // CDB request: this bank has a fully-ready entry it wants to
    // broadcast the ALU result of. The arbiter can grant this bank
    // any bus per cycle; only on req_grant does this bank actually
    // free the entry (which bus doesn't matter to this module). req_tid
    // tells the top-level arbiter which thread's ROB to route the mark
    // to.
    //
    // Phase 16: two more issue ports (req2/req3), each picking the next-
    // oldest ready entry after masking out the previous pick(s) -- the
    // same "find oldest, mask, repeat" reduction the top-level CDB
    // arbiter already uses. ALU compute is purely combinational, so this
    // is a real, not artificial, throughput increase: up to 3 independent
    // ALU results can now be produced (and broadcast, if the CDB grants
    // all three) in a single cycle, instead of Phase 1-15's fixed 1/cycle
    // regardless of dispatch/CDB width -- this is specifically what
    // cashes in the wider CDB/commit path for ALU-heavy code (see
    // README's Phase 16 section for the Phase 4/5 lesson this addresses).
    output req_valid,
    output req_tid,
    output [TAG_BITS-1:0] req_tag,
    output [63:0] req_value,
    input req_grant,
    output req2_valid,
    output req2_tid,
    output [TAG_BITS-1:0] req2_tag,
    output [63:0] req2_value,
    input req2_grant,
    output req3_valid,
    output req3_tid,
    output [TAG_BITS-1:0] req3_tag,
    output [63:0] req3_value,
    input req3_grant,

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
    // from lane 0's own pick) for lane 1, third-lowest (distinct from
    // whichever of lane 0/1's picks are actually consumed) for lane 2 ----
    integer fi;
    reg [DEPTH-1:0] free_mask;
    reg have_free;
    reg [31:0] free_idx;
    reg have_free2;
    reg [31:0] free_idx2;
    reg have_free3;
    reg [31:0] free_idx3;
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

        // Phase 16: same reasoning one level further -- lane 2's search
        // excludes free_idx only if lane 0 will actually consume it, and
        // excludes free_idx2 only if lane 1 will actually consume it (its
        // own do_alloc2 already folds in the same "actually consumed, not
        // just computed" gating one level up).
        have_free3 = 1'b0;
        free_idx3 = 0;
        for (fi = DEPTH - 1; fi >= 0; fi = fi - 1)
            if (free_mask[fi] && !(alloc_req && have_free && fi == free_idx) &&
                !(alloc2_req && have_free2 && fi == free_idx2)) begin
                have_free3 = 1'b1;
                free_idx3 = fi;
            end
    end
    assign full = !have_free;
    assign has_2_free = have_free2;
    assign has_3_free = have_free3;

    wire do_alloc1 = alloc_req  && have_free;
    wire do_alloc2 = alloc2_req && have_free2;
    wire do_alloc3 = alloc3_req && have_free3;

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

    // Phase 16: two more issue picks, each re-running the identical
    // reduction with the previous pick(s) masked out of contention -- the
    // same "find best of what's left" pattern the top-level CDB arbiter
    // already uses for its own 3 picks. Up to 3 independent ALU results
    // can now be produced in one cycle (see this module's own header for
    // why this, not just a wider CDB, is what actually raises sustained
    // ALU throughput).
    integer ri2;
    reg have_ready2;
    reg [31:0] ready_idx2;
    reg ready2_is_head;
    always @(*) begin
        have_ready2 = 1'b0;
        ready_idx2 = 0;
        ready2_is_head = 1'b0;
        for (ri2 = 0; ri2 < DEPTH; ri2 = ri2 + 1)
            if (busy[ri2] && s1_ready[ri2] && s2_ready[ri2] && !(have_ready && ri2 == ready_idx)) begin
                if (!have_ready2 || (entry_is_head(tid_arr[ri2], dest_tag[ri2]) && !ready2_is_head)) begin
                    have_ready2 = 1'b1;
                    ready_idx2 = ri2;
                    ready2_is_head = entry_is_head(tid_arr[ri2], dest_tag[ri2]);
                end
            end
    end

    integer ri3;
    reg have_ready3;
    reg [31:0] ready_idx3;
    reg ready3_is_head;
    always @(*) begin
        have_ready3 = 1'b0;
        ready_idx3 = 0;
        ready3_is_head = 1'b0;
        for (ri3 = 0; ri3 < DEPTH; ri3 = ri3 + 1)
            if (busy[ri3] && s1_ready[ri3] && s2_ready[ri3] &&
                !(have_ready && ri3 == ready_idx) && !(have_ready2 && ri3 == ready_idx2)) begin
                if (!have_ready3 || (entry_is_head(tid_arr[ri3], dest_tag[ri3]) && !ready3_is_head)) begin
                    have_ready3 = 1'b1;
                    ready_idx3 = ri3;
                    ready3_is_head = entry_is_head(tid_arr[ri3], dest_tag[ri3]);
                end
            end
    end

    // ALU compute, factored into a function (this file already uses one
    // for entry_is_head above) so the 3 issue ports share one definition
    // instead of tripling the case-statement pair by copy-paste -- each
    // call is still independent combinational logic, one per issue port.
    function [63:0] alu_exec;
        input [3:0] alu_op_in;
        input alu_word_op_in;
        input [63:0] a, b;
        reg [63:0] full_r;
        reg [31:0] word_r;
        begin
            case (alu_op_in)
                ALU_ADD:  full_r = a + b;
                ALU_SUB:  full_r = a - b;
                ALU_AND:  full_r = a & b;
                ALU_OR:   full_r = a | b;
                ALU_XOR:  full_r = a ^ b;
                ALU_SLL:  full_r = a << b[5:0];
                ALU_SRL:  full_r = a >> b[5:0];
                ALU_SRA:  full_r = $signed(a) >>> b[5:0];
                ALU_SLT:  full_r = ($signed(a) < $signed(b)) ? 64'd1 : 64'd0;
                ALU_SLTU: full_r = (a < b) ? 64'd1 : 64'd0;
                default:  full_r = 64'b0;
            endcase
            case (alu_op_in)
                ALU_ADD:  word_r = a[31:0] + b[31:0];
                ALU_SUB:  word_r = a[31:0] - b[31:0];
                ALU_SLL:  word_r = a[31:0] << b[4:0];
                ALU_SRL:  word_r = a[31:0] >> b[4:0];
                ALU_SRA:  word_r = $signed(a[31:0]) >>> b[4:0];
                default:  word_r = 32'b0;
            endcase
            alu_exec = alu_word_op_in ? {{32{word_r[31]}}, word_r} : full_r;
        end
    endfunction

    assign req_valid = have_ready;
    assign req_tid   = tid_arr[ready_idx];
    assign req_tag   = dest_tag[ready_idx];
    assign req_value = alu_exec(op[ready_idx], word_op_arr[ready_idx], s1_val[ready_idx], s2_val[ready_idx]);

    assign req2_valid = have_ready2;
    assign req2_tid   = tid_arr[ready_idx2];
    assign req2_tag   = dest_tag[ready_idx2];
    assign req2_value = alu_exec(op[ready_idx2], word_op_arr[ready_idx2], s1_val[ready_idx2], s2_val[ready_idx2]);

    assign req3_valid = have_ready3;
    assign req3_tid   = tid_arr[ready_idx3];
    assign req3_tag   = dest_tag[ready_idx3];
    assign req3_value = alu_exec(op[ready_idx3], word_op_arr[ready_idx3], s1_val[ready_idx3], s2_val[ready_idx3]);

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
            if (cdbA_valid || cdbB_valid || cdbC_valid) begin
                for (fi = 0; fi < DEPTH; fi = fi + 1) begin
                    if (busy[fi] && !s1_ready[fi] && cdbA_valid && tid_arr[fi] == cdbA_tid && s1_tag[fi] == cdbA_tag) begin
                        s1_ready[fi] <= 1'b1;
                        s1_val[fi]   <= cdbA_value;
                    end else if (busy[fi] && !s1_ready[fi] && cdbB_valid && tid_arr[fi] == cdbB_tid && s1_tag[fi] == cdbB_tag) begin
                        s1_ready[fi] <= 1'b1;
                        s1_val[fi]   <= cdbB_value;
                    end else if (busy[fi] && !s1_ready[fi] && cdbC_valid && tid_arr[fi] == cdbC_tid && s1_tag[fi] == cdbC_tag) begin
                        s1_ready[fi] <= 1'b1;
                        s1_val[fi]   <= cdbC_value;
                    end
                    if (busy[fi] && !s2_ready[fi] && cdbA_valid && tid_arr[fi] == cdbA_tid && s2_tag[fi] == cdbA_tag) begin
                        s2_ready[fi] <= 1'b1;
                        s2_val[fi]   <= cdbA_value;
                    end else if (busy[fi] && !s2_ready[fi] && cdbB_valid && tid_arr[fi] == cdbB_tid && s2_tag[fi] == cdbB_tag) begin
                        s2_ready[fi] <= 1'b1;
                        s2_val[fi]   <= cdbB_value;
                    end else if (busy[fi] && !s2_ready[fi] && cdbC_valid && tid_arr[fi] == cdbC_tid && s2_tag[fi] == cdbC_tag) begin
                        s2_ready[fi] <= 1'b1;
                        s2_val[fi]   <= cdbC_value;
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
            if (do_alloc3) begin
                busy[free_idx3]     <= 1'b1;
                tid_arr[free_idx3]  <= alloc3_tid;
                op[free_idx3]       <= alloc3_op;
                word_op_arr[free_idx3] <= alloc3_word_op;
                s1_ready[free_idx3] <= alloc3_src1_ready;
                s1_val[free_idx3]   <= alloc3_src1_val;
                s1_tag[free_idx3]   <= alloc3_src1_tag;
                s2_ready[free_idx3] <= alloc3_src2_ready;
                s2_val[free_idx3]   <= alloc3_src2_val;
                s2_tag[free_idx3]   <= alloc3_src2_tag;
                dest_tag[free_idx3] <= alloc3_dest_tag;
            end

            // ready_idx/ready_idx2/ready_idx3 are always distinct by
            // construction (each pass masks out the previous pick(s)), so
            // these three vacate conditions can never target the same
            // index in the same cycle.
            if (req_valid && req_grant) begin
                busy[ready_idx] <= 1'b0;
            end
            if (req2_valid && req2_grant) begin
                busy[ready_idx2] <= 1'b0;
            end
            if (req3_valid && req3_grant) begin
                busy[ready_idx3] <= 1'b0;
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
