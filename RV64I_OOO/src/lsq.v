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
// LSQ, which used to mean "until commit" when memory access was instant
// (Phase 1-6); see the store-buffer note below for what changed in
// Phase 8.
//
// Stores never broadcast a value (no destination register); once both
// their address and data operands are ready, they become commit-eligible
// via `store_ready`/`store_tag` (an array, since -- unlike branch_rs's
// single entry -- multiple stores can legitimately become ready the same
// cycle; the top level fans these into rob.v's extra_mark ports, which
// tolerates exactly this "several simultaneous, always-distinct tags"
// case by construction).
//
// Phase 8 (real L1/L2 cache + MESI): the flat, same-cycle data_memory.v
// interface Phase 1-7 used is gone -- l1_cache.v is a genuinely multi-
// cycle, blocking resource (a miss, or a coherency transaction, can take
// many cycles; see its own header). Two consequences:
//
//   - Loads become a real outstanding-request operation, structurally
//     the same category of thing div_rs.v already does for a multi-cycle
//     divide: issue to l1_cache.v, wait for l1_read_valid, THEN broadcast
//     on the CDB. Unlike a divide, though, a load's ROB entry can be
//     squashed *while* the request is still outstanding at the cache
//     (the cache itself has no notion of speculation) -- if that happens,
//     the eventual response must be silently discarded, not broadcast to
//     whatever unrelated instruction has since reused that same LSQ slot
//     (see outstanding_squashed below).
//
//   - Store commit can no longer be "instant": actually acquiring
//     ownership of a cache line (a coherency transaction) can take many
//     cycles, but ROB retirement itself must stay exactly as fast as
//     Phase 1-7 (nothing about *architectural* commit should get slower
//     just because the memory subsystem got more realistic). The fix is
//     a small store buffer: at the same moment a store used to fire its
//     instant memory write, it now gets pushed into this buffer instead
//     (its address is already fully resolved by commit time, so this is
//     just a data-and-address handoff) and the LSQ slot vacates
//     immediately, exactly like before. The buffer drains into l1_cache.v
//     asynchronously, at whatever pace the cache can actually accept
//     writes. A store buffer entry, having already committed, can never
//     be squashed -- once pushed, it is unconditionally going to happen.
//     Younger loads must additionally check the store buffer (not just
//     LSQ-resident stores) for an address match before issuing; every
//     buffered entry is, by construction, older than any still-resident
//     LSQ instruction (in-order commit already guarantees that), so no
//     age comparison is needed there, only an address check. If the
//     buffer is ever full, a new store simply can't be pushed yet --
//     store_buffer_full tells the top level to withhold that commit for
//     a cycle, the same kind of space-available backpressure Phase 5's
//     dual-commit already uses for the single store-memory-port conflict.
//
// Only one l1_cache.v access (its single blocking request port) can be
// in flight at a time; issuing a new load is given priority over
// draining a buffered store whenever both want it the same cycle -- a
// load feeds dependent instructions waiting on the CDB, a buffered
// store's only consequence (beyond eventually landing in the cache) is
// filling up its own buffer, a slower-forming problem.
//
// Phase 10 (hit-under-miss): l1_cache.v's primary port above is still
// exactly this single-outstanding resource. What's new is an entirely
// separate, same-cycle, hit-or-reject probe (l1_read2_*) tried
// opportunistically whenever the primary port is occupied by something
// else (an outstanding load miss, an outstanding store drain, or a
// background prefetch) -- see l1_cache.v's own header for why a plain
// hit check against its live state/tag arrays is always safe regardless
// of what the primary port is doing. Because it's a live combinational
// read with no request/response state of its own, trying it and missing
// costs nothing: the same candidate (or a different one, if this one is
// still blocked) is simply retried next cycle. A won port-2 hit
// completes in the exact same cycle it's tried -- there is no
// "outstanding" bookkeeping needed for it the way the primary load path
// needs load_outstanding/outstanding_idx, since nothing about it spans
// more than one cycle. It shares the single req_valid/tag/value CDB
// request output with the primary load path; the primary path always
// wins that arbitration when both want it the same cycle, since
// l1_read_valid is a one-shot pulse from l1_cache.v's FSM that would be
// lost forever if not consumed this exact cycle, whereas a port-2 hit's
// underlying data is still sitting safely in the cache either way.
//
// Phase 10 (merging write buffer): a new store committing to the exact
// same doubleword address as one already resident in the buffer
// overwrites that entry in place instead of allocating a new slot --
// the newer value supersedes the older one for that address regardless,
// so this is a free reduction in both buffer pressure and the number of
// l1_cache.v write transactions eventually issued. This is deliberately
// narrower than a textbook merging write buffer's usual scope (combining
// several *different* offsets within the same cache line into one wide
// bus transaction): that would need l1_cache.v's write port widened to
// accept a masked whole-line write, a bigger change than this project's
// current write-one-doubleword-at-a-time protocol supports. Documented
// scope line, not an oversight.
module lsq #(
    parameter DEPTH = 4,
    parameter TAG_BITS = 3,
    parameter SBUF_DEPTH = 4
)(
    input clk,
    input reset,

    input alloc_req,
    input alloc_tid,
    input alloc_is_store,
    input [2:0] alloc_func3,
    input [63:0] alloc_imm,
    input alloc_base_ready, input [63:0] alloc_base_val, input [TAG_BITS-1:0] alloc_base_tag,
    input alloc_data_ready, input [63:0] alloc_data_val, input [TAG_BITS-1:0] alloc_data_tag,
    input [TAG_BITS-1:0] alloc_dest_tag,
    output full,

    // Phase 3: lane-1 allocation port, same convention as alu_rs.v.
    input alloc2_req,
    input alloc2_tid,
    input alloc2_is_store,
    input [2:0] alloc2_func3,
    input [63:0] alloc2_imm,
    input alloc2_base_ready, input [63:0] alloc2_base_val, input [TAG_BITS-1:0] alloc2_base_tag,
    input alloc2_data_ready, input [63:0] alloc2_data_val, input [TAG_BITS-1:0] alloc2_data_tag,
    input [TAG_BITS-1:0] alloc2_dest_tag,
    output has_2_free,

    // Two independent CDB snoop buses -- see alu_rs.v's identical port,
    // including cdbA_tid/cdbB_tid (Phase 7 fix: ROB tags are only unique
    // per-thread, so snoop matching must compare the full (tid,tag) pair,
    // not the tag alone).
    input cdbA_valid,
    input cdbA_tid,
    input [TAG_BITS-1:0] cdbA_tag,
    input [63:0] cdbA_value,
    input cdbB_valid,
    input cdbB_tid,
    input [TAG_BITS-1:0] cdbB_tag,
    input [63:0] cdbB_value,

    // Phase 7 (SMT): one head tag per thread, for thread-aware age
    // comparison in disambiguation/squash -- see alu_rs.v's header.
    input [TAG_BITS-1:0] rob_head_tag0,
    input [TAG_BITS-1:0] rob_head_tag1,

    // Widened commit: which (thread, tag) is the *actual* committing
    // store this cycle -- head's if the head itself is a store, or
    // head2's if the head isn't a store but head+1 is and it's also
    // committing (only one store buffer push is accepted per cycle --
    // see the top level for the single-store-commit-per-cycle
    // arbitration this mirrors from Phase 5/7). commit_lookup_tid was
    // added for Phase 7: tags alone are no longer unique once two
    // threads' ROBs can both be issuing numerically identical tags.
    input commit_lookup_tid,
    input [TAG_BITS-1:0] commit_lookup_tag,

    // Load broadcast request (CDB) -- same req/grant convention as
    // alu_rs.v/mul_rs.v.
    output req_valid,
    output req_tid,
    output [TAG_BITS-1:0] req_tag,
    output [63:0] req_value,
    input req_grant,

    // ---- l1_cache.v CPU-side port (Phase 8) -- shared by load-issue and
    // store-buffer-drain, one outstanding request at a time. ----
    output l1_read_req,
    output [63:0] l1_read_addr,
    output [2:0] l1_read_func3,
    input l1_read_valid,
    input [63:0] l1_read_data,
    output l1_write_req,
    output [63:0] l1_write_addr,
    output [63:0] l1_write_data,
    output [2:0] l1_write_func3,
    input l1_write_done,
    input l1_busy,

    // Phase 10 (hit-under-miss): l1_cache.v's second, hit-only,
    // same-cycle read port -- see this module's own header addendum
    // below for how it's used.
    output l1_read2_req,
    output [63:0] l1_read2_addr,
    output [2:0] l1_read2_func3,
    input l1_read2_hit,
    input [63:0] l1_read2_data,

    // Per-slot store readiness -> committable via rob.v's extra_mark
    // ports, same as Phase 1-7 -- this doesn't change: it just means
    // "this store's operands are known," not "this store has committed."
    output [DEPTH-1:0] store_ready,
    output [DEPTH*TAG_BITS-1:0] store_ready_tag_flat,
    output [DEPTH-1:0] store_ready_tid_flat,

    // Push a just-committed store into the store buffer this cycle --
    // commit_fire replaces the old immediate-write trigger; the address
    // is already resolved (entry_addr), so nothing but a data handoff and
    // slot vacate happens here. store_buffer_full tells the top level to
    // withhold this commit for a cycle if there's nowhere to put it.
    output commit_match,
    input commit_fire,
    output store_buffer_full,

    // Phase 2 misprediction squash, now two independent per-thread ports
    // (Phase 7) -- see alu_rs.v's header for why a single muxed port can't
    // handle both threads mispredicting in the same cycle.
    input squash0_valid,
    input [TAG_BITS-1:0] squash0_tag,
    input squash1_valid,
    input [TAG_BITS-1:0] squash1_tag
);
    reg busy        [0:DEPTH-1];
    reg tid_arr     [0:DEPTH-1];
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

    // Phase 7: age relative to whichever thread's head tag applies --
    // used only for *same-thread* comparisons (disambiguation and
    // squash both first check tid_arr[i]==the other operand's tid before
    // ever calling this), so there's no cross-thread meaning implied.
    function [TAG_BITS-1:0] age;
        input t;
        input [TAG_BITS-1:0] tag;
        begin
            age = t ? (tag - rob_head_tag1) : (tag - rob_head_tag0);
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

    // ---- Store buffer (Phase 8) -----------------------------------------
    reg sbuf_valid [0:SBUF_DEPTH-1];
    reg sbuf_tid   [0:SBUF_DEPTH-1];
    reg [63:0] sbuf_addr [0:SBUF_DEPTH-1];
    reg [63:0] sbuf_data [0:SBUF_DEPTH-1];
    reg [2:0]  sbuf_func3[0:SBUF_DEPTH-1];

    integer sfi;
    reg sbuf_have_free;
    reg [31:0] sbuf_free_idx;
    always @(*) begin
        sbuf_have_free = 1'b0;
        sbuf_free_idx = 0;
        for (sfi = SBUF_DEPTH - 1; sfi >= 0; sfi = sfi - 1)
            if (!sbuf_valid[sfi]) begin
                sbuf_have_free = 1'b1;
                sbuf_free_idx = sfi;
            end
    end
    // ---- Commit-time store lookup: which resident store (if any) has
    // the exact (tid, tag) the top level says is committing this cycle
    // -- at most one match, by the ROB's own tag-uniqueness invariant.
    // This is purely predictive (depends only on commit_lookup_tid/tag,
    // never on commit_fire), which is what lets sbuf_merge_hit and
    // store_buffer_full below safely consume it before commit_fire is
    // even known this cycle.
    reg commit_match_r;
    reg [31:0] commit_idx_r;
    integer ci;
    always @(*) begin
        commit_match_r = 1'b0;
        commit_idx_r = 0;
        for (ci = 0; ci < DEPTH; ci = ci + 1) begin
            if (busy[ci] && is_store_arr[ci] && tid_arr[ci] == commit_lookup_tid && dest_tag[ci] == commit_lookup_tag) begin
                commit_match_r = 1'b1;
                commit_idx_r = ci;
            end
        end
    end
    assign commit_match = commit_match_r;

    // ---- Phase 10: merging write buffer -- does a resident, not-yet-
    // draining entry already cover the EXACT same (address, width) as
    // the store about to commit? Only an exact match is safe to collapse
    // in place (see this module's header addendum for why a width
    // mismatch can't just overwrite the old entry -- it could shrink the
    // covered byte range and silently lose part of the older write).
    // store_draining's own in-flight entry is deliberately excluded: it
    // may already be mid-handshake at l1_cache.v this exact cycle, so
    // mutating its data out from under that request is unsafe -- let it
    // finish; the new store simply gets its own fresh slot instead, same
    // as before this feature existed.
    reg sbuf_merge_hit;
    reg [31:0] sbuf_merge_idx;
    integer mi;
    always @(*) begin
        sbuf_merge_hit = 1'b0;
        sbuf_merge_idx = 0;
        for (mi = 0; mi < SBUF_DEPTH; mi = mi + 1) begin
            if (sbuf_valid[mi] && !(store_draining && draining_idx == mi) &&
                sbuf_tid[mi] == tid_arr[commit_idx_r] &&
                sbuf_addr[mi] == entry_addr[commit_idx_r] &&
                sbuf_func3[mi] == func3[commit_idx_r]) begin
                sbuf_merge_hit = 1'b1;
                sbuf_merge_idx = mi;
            end
        end
    end

    // A merge in place doesn't need a free slot at all -- it's reusing
    // an existing one -- so store_buffer_full (sbuf_have_free) must not
    // gate this path the way it gates a genuinely new allocation.
    assign store_buffer_full = !sbuf_have_free && !(commit_match_r && sbuf_merge_hit);
    wire do_commit_push = commit_fire && commit_match_r && (sbuf_have_free || sbuf_merge_hit);

    // ---- Disambiguation: which load slots are blocked by an older,
    // still-resident store this cycle, *or* by a same-thread store
    // buffer entry (Phase 8 -- see module header: every buffered entry
    // is unconditionally older than any still-resident LSQ instruction,
    // and always has a fully-resolved address, so this is a plain
    // address compare, no readiness/age check needed the way resident
    // stores need). Phase 7 (SMT): only same-thread stores are ever
    // considered -- cross-thread memory ordering/consistency is
    // explicitly out of scope for this integration (the same category
    // of simplification as this whole project having no MMU/virtual
    // memory), and there is no meaningful cross-thread notion of "older"
    // to compare against anyway (each thread has its own ROB tag space).
    reg blocked [0:DEPTH-1];
    integer li, si, sbi;
    always @(*) begin
        for (li = 0; li < DEPTH; li = li + 1) begin
            blocked[li] = 1'b0;
            if (busy[li] && !is_store_arr[li]) begin
                for (si = 0; si < DEPTH; si = si + 1) begin
                    if (si != li && busy[si] && is_store_arr[si] && tid_arr[si] == tid_arr[li] &&
                        (age(tid_arr[si], dest_tag[si]) < age(tid_arr[li], dest_tag[li]))) begin
                        if (!base_ready[si])
                            blocked[li] = 1'b1;
                        else if (entry_addr[si][63:3] == entry_addr[li][63:3])
                            blocked[li] = 1'b1;
                    end
                end
                for (sbi = 0; sbi < SBUF_DEPTH; sbi = sbi + 1) begin
                    if (sbuf_valid[sbi] && sbuf_tid[sbi] == tid_arr[li] &&
                        sbuf_addr[sbi][63:3] == entry_addr[li][63:3])
                        blocked[li] = 1'b1;
                end
            end
        end
    end

    // ---- l1_cache.v arbitration + outstanding-operation tracking -------
    // Load issue wins over store-buffer drain when both want the port;
    // neither may issue while l1_busy (one outstanding request at a
    // time) or while a load is already outstanding from this module's
    // own last issue (l1_busy alone already covers that, but tracking it
    // explicitly here is what lets a squash-while-outstanding be handled
    // safely -- see below).
    integer ri;
    reg have_ready;
    reg [31:0] ready_idx;
    always @(*) begin
        have_ready = 1'b0;
        ready_idx = 0;
        if (!l1_busy && !load_outstanding) begin
            for (ri = DEPTH - 1; ri >= 0; ri = ri - 1)
                if (busy[ri] && !is_store_arr[ri] && base_ready[ri] && !blocked[ri]) begin
                    have_ready = 1'b1;
                    ready_idx = ri;
                end
        end
    end

    reg load_outstanding;
    reg outstanding_squashed;
    reg [31:0] outstanding_idx;
    reg [TAG_BITS-1:0] outstanding_tag;
    reg outstanding_tid;

    // Same category of bug this module's own load path already had to
    // get right (see outstanding_idx above): l1_write_done pulses *while*
    // l1_busy is still 1 (ST_DONE_W is itself a non-idle state in
    // l1_cache.v -- see its own header), so a clear condition built from
    // "!l1_busy && ...==the entry I'd pick right now" can never actually
    // fire -- by the time done pulses, l1_busy blocks the very
    // "currently draining" signal the clear was gated on, and
    // sbuf_drain_idx is a live combinational pick that isn't guaranteed
    // to still point at the right entry anyway. store_draining/
    // draining_idx latch which entry is actually in flight the same way
    // load_outstanding/outstanding_idx already do.
    reg store_draining;
    reg [31:0] draining_idx;

    // ---- Phase 10: opportunistic second-load candidate (hit-under-
    // miss). Tried only while the primary port is occupied by something
    // else this cycle -- when it's free, everything goes through the
    // primary issue_load path below as always, since there's no benefit
    // to routing through the same-cycle hit-only probe instead of the
    // primary path's own fully general one. Excludes whichever slot is
    // already the primary path's own in-flight load (outstanding_idx),
    // and reuses the same blocked[]/base_ready[] readiness check issue_load
    // uses -- l1_read2_hit (from l1_cache.v, purely combinational) is
    // this cycle's actual pass/fail verdict; if it misses, nothing is
    // committed and the same or a different candidate is simply retried
    // next cycle.
    wire primary_occupied = l1_busy || load_outstanding || store_draining;
    integer r2;
    reg have_ready2;
    reg [31:0] ready_idx2;
    always @(*) begin
        have_ready2 = 1'b0;
        ready_idx2 = 0;
        if (primary_occupied) begin
            for (r2 = DEPTH - 1; r2 >= 0; r2 = r2 - 1)
                if (busy[r2] && !is_store_arr[r2] && base_ready[r2] && !blocked[r2] &&
                    !(load_outstanding && outstanding_idx == r2)) begin
                    have_ready2 = 1'b1;
                    ready_idx2 = r2;
                end
        end
    end

    assign l1_read2_req   = have_ready2;
    assign l1_read2_addr  = entry_addr[ready_idx2];
    assign l1_read2_func3 = func3[ready_idx2];
    wire port2_fire = have_ready2 && l1_read2_hit;

    wire issue_load  = have_ready;
    wire drain_store = !l1_busy && !load_outstanding && !issue_load && !store_draining && sbuf_valid[sbuf_drain_idx];

    // Oldest-first drain: fixed lowest-index scan is enough here (a real
    // FIFO would need head/tail bookkeeping for no behavioral benefit --
    // every entry is already unconditionally safe to drain in any order,
    // since none of them can alias *each other* differently than program
    // order already guaranteed at commit time, and this buffer's only
    // job is eventually getting each one out, not preserving a specific
    // drain order beyond what correctness already requires).
    integer sdi;
    reg [31:0] sbuf_drain_idx;
    always @(*) begin
        sbuf_drain_idx = 0;
        for (sdi = SBUF_DEPTH - 1; sdi >= 0; sdi = sdi - 1)
            if (sbuf_valid[sdi])
                sbuf_drain_idx = sdi;
    end

    assign l1_read_req   = issue_load;
    assign l1_read_addr  = entry_addr[ready_idx];
    assign l1_read_func3 = func3[ready_idx];

    assign l1_write_req   = drain_store;
    assign l1_write_addr  = sbuf_addr[sbuf_drain_idx];
    assign l1_write_data  = sbuf_data[sbuf_drain_idx];
    assign l1_write_func3 = sbuf_func3[sbuf_drain_idx];

    // CDB request: only once the outstanding load's data has actually
    // come back from l1_cache.v, and only if it wasn't squashed away in
    // the meantime (outstanding_squashed) -- a squashed load's response
    // is silently discarded, never broadcast onto a tag that may since
    // have been reused by an unrelated instruction.
    //
    // Phase 10: the primary path's own pulse ALWAYS wins this single CDB
    // request port when both want it the same cycle -- l1_read_valid is
    // a one-shot pulse from l1_cache.v's FSM (fsm returns to ST_IDLE the
    // very next cycle either way) that would be lost forever if not
    // consumed exactly this cycle, whereas port2_fire's underlying data
    // is a live combinational read that's still sitting safely in the
    // cache if it has to wait one more cycle for the port.
    wire primary_fire = load_outstanding && l1_read_valid && !outstanding_squashed;
    assign req_valid = primary_fire || port2_fire;
    assign req_tid    = primary_fire ? outstanding_tid : tid_arr[ready_idx2];
    assign req_tag    = primary_fire ? outstanding_tag : dest_tag[ready_idx2];
    assign req_value  = primary_fire ? l1_read_data : l1_read2_data;

    // ---- Per-slot store readiness (address + data both known). tid is
    // exposed alongside the tag (Phase 7) so the top level can route each
    // ready store's mark to the correct thread's ROB. ----
    genvar gi;
    generate
        for (gi = 0; gi < DEPTH; gi = gi + 1) begin : store_ready_gen
            assign store_ready[gi] = busy[gi] && is_store_arr[gi] && base_ready[gi] && data_ready[gi];
            assign store_ready_tag_flat[(gi+1)*TAG_BITS-1 -: TAG_BITS] = dest_tag[gi];
            assign store_ready_tid_flat[gi] = tid_arr[gi];
        end
    endgenerate

    integer vi;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (vi = 0; vi < DEPTH; vi = vi + 1)
                busy[vi] <= 1'b0;
            for (vi = 0; vi < SBUF_DEPTH; vi = vi + 1)
                sbuf_valid[vi] <= 1'b0;
            load_outstanding <= 1'b0;
            store_draining <= 1'b0;
            outstanding_squashed <= 1'b0;
        end else begin
            for (vi = 0; vi < DEPTH; vi = vi + 1) begin
                if (busy[vi] && !base_ready[vi] && cdbA_valid && tid_arr[vi] == cdbA_tid && base_tag[vi] == cdbA_tag) begin
                    base_ready[vi] <= 1'b1;
                    base_val[vi]   <= cdbA_value;
                end else if (busy[vi] && !base_ready[vi] && cdbB_valid && tid_arr[vi] == cdbB_tid && base_tag[vi] == cdbB_tag) begin
                    base_ready[vi] <= 1'b1;
                    base_val[vi]   <= cdbB_value;
                end
                if (busy[vi] && !data_ready[vi] && cdbA_valid && tid_arr[vi] == cdbA_tid && data_tag[vi] == cdbA_tag) begin
                    data_ready[vi] <= 1'b1;
                    data_val[vi]   <= cdbA_value;
                end else if (busy[vi] && !data_ready[vi] && cdbB_valid && tid_arr[vi] == cdbB_tid && data_tag[vi] == cdbB_tag) begin
                    data_ready[vi] <= 1'b1;
                    data_val[vi]   <= cdbB_value;
                end
            end

            if (do_alloc1) begin
                busy[free_idx]        <= 1'b1;
                tid_arr[free_idx]     <= alloc_tid;
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
                tid_arr[free_idx2]     <= alloc2_tid;
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

            // Issue a new outstanding load: latch everything the eventual
            // response needs into dedicated regs (not re-read from the
            // per-slot arrays later -- see module header on why: this
            // slot could otherwise be squashed and reallocated to a
            // completely different instruction before the response
            // arrives).
            if (issue_load) begin
                load_outstanding <= 1'b1;
                outstanding_squashed <= 1'b0;
                outstanding_idx <= ready_idx;
                outstanding_tag <= dest_tag[ready_idx];
                outstanding_tid <= tid_arr[ready_idx];
            end
            if (load_outstanding && l1_read_valid) begin
                load_outstanding <= 1'b0;
                busy[outstanding_idx] <= 1'b0;
            end

            // Phase 10: a won port-2 hit completes in the same cycle it's
            // tried -- no "outstanding" state needed, just free the slot
            // the moment the CDB request actually wins arbitration. If it
            // ISN'T granted this cycle, busy[ready_idx2] stays set and
            // the same (or by-then a different) candidate is naturally
            // reconsidered next cycle -- no data was ever at risk, unlike
            // the primary path's one-shot l1_read_valid pulse.
            if (req_valid && req_grant && !primary_fire) begin
                busy[ready_idx2] <= 1'b0;
            end

            // Push a committing store into the buffer (address already
            // resolved -- entry_addr is combinational off base_val+imm,
            // both of which store_ready already required to be known).
            // Phase 10: an exact (addr, width) match against an existing
            // entry collapses into that entry's slot instead of consuming
            // a new one -- see sbuf_merge_hit's own comment for why this
            // is always safe (only a bit-for-bit identical footprint ever
            // merges, and a mid-drain entry is excluded).
            if (do_commit_push) begin
                busy[commit_idx_r] <= 1'b0;
                if (sbuf_merge_hit) begin
                    sbuf_data[sbuf_merge_idx] <= data_val[commit_idx_r];
                end else begin
                    sbuf_valid[sbuf_free_idx] <= 1'b1;
                    sbuf_tid[sbuf_free_idx]   <= tid_arr[commit_idx_r];
                    sbuf_addr[sbuf_free_idx]  <= entry_addr[commit_idx_r];
                    sbuf_data[sbuf_free_idx]  <= data_val[commit_idx_r];
                    sbuf_func3[sbuf_free_idx] <= func3[commit_idx_r];
                end
            end

            // Issue a new outstanding store-buffer drain, latching which
            // entry it is (see store_draining's own comment above for why
            // this can't just be re-derived from sbuf_drain_idx/drain_store
            // at the moment the response actually arrives).
            if (drain_store) begin
                store_draining <= 1'b1;
                draining_idx <= sbuf_drain_idx;
            end
            if (store_draining && l1_write_done) begin
                store_draining <= 1'b0;
                sbuf_valid[draining_idx] <= 1'b0;
            end

            if (squash0_valid) begin
                for (vi = 0; vi < DEPTH; vi = vi + 1)
                    if (busy[vi] && !tid_arr[vi] &&
                        age(1'b0, dest_tag[vi]) > age(1'b0, squash0_tag) &&
                        !(load_outstanding && outstanding_idx == vi))
                        busy[vi] <= 1'b0;
                // The outstanding load's own slot can't be freed yet (its
                // cache request is still in flight) -- just mark it to be
                // discarded, not broadcast, once the response finally
                // arrives (see req_valid above and the l1_read_valid
                // handler, which still frees the slot either way).
                if (load_outstanding && !outstanding_tid &&
                    age(1'b0, outstanding_tag) > age(1'b0, squash0_tag))
                    outstanding_squashed <= 1'b1;
            end
            if (squash1_valid) begin
                for (vi = 0; vi < DEPTH; vi = vi + 1)
                    if (busy[vi] && tid_arr[vi] &&
                        age(1'b1, dest_tag[vi]) > age(1'b1, squash1_tag) &&
                        !(load_outstanding && outstanding_idx == vi))
                        busy[vi] <= 1'b0;
                if (load_outstanding && outstanding_tid &&
                    age(1'b1, outstanding_tag) > age(1'b1, squash1_tag))
                    outstanding_squashed <= 1'b1;
            end
        end
    end
endmodule
