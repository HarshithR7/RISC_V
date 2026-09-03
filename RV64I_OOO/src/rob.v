`timescale 1ns / 1ps
// Reorder Buffer: DEPTH-entry circular buffer providing precise,
// in-order commit on top of out-of-order execution. This is the
// mechanism that actually resolves WAW for the *architectural* register
// file's final value -- the real register file (RV64I/src/register_file.v,
// reused unmodified for this core) is written only at commit, never at
// CDB-broadcast time, so an out-of-order-completing older producer can
// never clobber a younger one's already-committed value.
//
// DEPTH must be a power of 2 so head_ptr/tail_ptr wrap correctly via
// plain unsigned overflow on a $clog2(DEPTH)-bit register.
module rob #(
    parameter DEPTH = 8,
    // Width of the extra_mark bus below -- sized to the LSQ's own DEPTH,
    // since that's the only other source of "no CDB value, just done"
    // marks besides mark2's single conditional-branch case, and unlike
    // branch_rs's single entry, several LSQ stores can legitimately
    // become commit-eligible the same cycle.
    parameter EXTRA_MARK_N = 4,
    // Vector (Phase 6, DLP) support: width of the parallel vec_value_arr
    // below, carrying VLEN-bit vector results alongside every entry's
    // ordinary 64-bit scalar value_arr slot. Kept as a genuinely separate
    // array (not a widened value_arr) so scalar-only programs pay nothing
    // extra, and so a vector result's width can differ from VLEN's
    // relationship to 64 in either direction without truncation risk.
    parameter VLEN = 128
)(
    input clk,
    input reset,

    // Dispatch: allocate a new entry for the next (lane 0, older)
    // instruction. alloc_tag is valid combinationally whenever !full
    // (dispatch logic samples it the same cycle it asserts alloc_req).
    input alloc_req,
    input alloc_has_dest,
    input [4:0] alloc_rd,
    input alloc_is_store,
    input alloc_is_ecall,
    // Phase 6: does this entry's destination register live in the vector
    // register file (vd, written from vec_value_arr at commit) rather
    // than the scalar one (rd, written from value_arr)? Mutually
    // exclusive with alloc_has_dest in practice (an entry has at most one
    // destination), but tracked as its own bit rather than overloading
    // alloc_has_dest's meaning, so the commit-time top level can cleanly
    // pick which register file (and which value array) to write from.
    input alloc_is_vec_dest,
    output [$clog2(DEPTH)-1:0] alloc_tag,
    output full,
    output empty,

    // Phase 3: a second, lane-1 (younger) allocation port for 2-wide
    // dispatch. alloc2_tag = alloc_tag+1 combinationally -- always valid
    // to *read*, but only actually consumed (tail_ptr/count advance past
    // it) if alloc2_req fires and free_count is >=2 this cycle. The
    // caller is expected to only ever assert alloc2_req alongside
    // alloc_req (lane 1 can't dispatch without lane 0 -- dispatch stays
    // in-order), but this module doesn't itself depend on that for
    // safety, only for the *tag numbering* to make sense (alloc2_tag is
    // always alloc_tag+1, regardless).
    input alloc2_req,
    input alloc2_has_dest,
    input [4:0] alloc2_rd,
    input alloc2_is_store,
    input alloc2_is_ecall,
    input alloc2_is_vec_dest,
    output [$clog2(DEPTH)-1:0] alloc2_tag,
    output [$clog2(DEPTH):0] free_count,   // 0..DEPTH

    // Phase 16: a third, lane-2 (youngest) allocation port for 3-wide
    // dispatch, same convention as alloc2 -- alloc3_tag = alloc_tag+2
    // combinationally, always valid to read, only consumed if alloc3_req
    // fires and free_count is >=3 this cycle.
    input alloc3_req,
    input alloc3_has_dest,
    input [4:0] alloc3_rd,
    input alloc3_is_store,
    input alloc3_is_ecall,
    input alloc3_is_vec_dest,
    output [$clog2(DEPTH)-1:0] alloc3_tag,

    // Mark an entry's result ready: a CDB broadcast for a register-
    // producing entry (mark_value is the result), or the LSQ signaling a
    // store's address+data are both known (mark_value don't-care --
    // stores never broadcast a value, they only become commit-eligible).
    input mark_valid,
    input [$clog2(DEPTH)-1:0] mark_tag,
    input [63:0] mark_value,

    // Second, unarbitrated mark port for entries that need to become
    // commit-eligible *without* ever producing a CDB value at all --
    // concretely, a resolved conditional branch (no destination
    // register, so it never has anything to broadcast, but still must
    // unblock commit once it resolves). Kept genuinely separate from the
    // port above, not merged behind arbitration, because unlike a real
    // register value (a shared, one-broadcast-per-cycle resource), "this
    // tag has no result and is simply done" carries no data to contend
    // over -- marking two different tags done in the same cycle here is
    // always safe by construction (ROB tags are always distinct), so
    // there's nothing to arbitrate.
    input mark2_valid,
    input [$clog2(DEPTH)-1:0] mark2_tag,

    // Widened-CDB support: a second CDB-sourced mark port, exactly
    // mirroring mark_valid/mark_tag/mark_value, for when the top-level
    // arbiter grants a second, simultaneous broadcast this cycle (see
    // riscv64_ooo_proc.v's 2-wide CDB arbiter). Safe without arbitration
    // for the same reason mark2/extra_mark already are: the two winning
    // requesters are always different reservation-station entries with
    // always-distinct ROB tags.
    input mark_b_valid,
    input [$clog2(DEPTH)-1:0] mark_b_tag,
    input [63:0] mark_b_value,

    // Phase 16: a third CDB-sourced mark port, exactly mirroring
    // mark_valid/mark_b_valid, for the top level's 3-wide CDB arbiter's
    // third simultaneous winner. Safe without arbitration for the same
    // reason mark_b already is: the three winning requesters always carry
    // distinct ROB tags.
    input mark_c_valid,
    input [$clog2(DEPTH)-1:0] mark_c_tag,
    input [63:0] mark_c_value,

    // Phase 6: vector-result mark port -- exactly mark_valid's role, but
    // writing vec_value_arr (VLEN bits) instead of value_arr (64 bits).
    // Genuinely separate rather than widening mark_valid/mark_value to
    // VLEN, since vec_rs.v's own broadcast is entirely independent of the
    // scalar CDB arbiter (vector operands in this scoped core only ever
    // come from other vector instructions, never from scalar ones -- see
    // riscv64_ooo_proc.v's header -- so there's no cross-domain
    // arbitration to unify).
    input vec_mark_valid,
    input [$clog2(DEPTH)-1:0] vec_mark_tag,
    input [VLEN-1:0] vec_mark_value,

    // Third mark bus, EXTRA_MARK_N-wide: the LSQ's per-store-slot
    // readiness (address+data both known -> commit-eligible). Multiple
    // lanes can be valid the same cycle, each targeting a different tag
    // -- safe by the same "ROB tags are always distinct" invariant as
    // mark2, just generalized from one lane to EXTRA_MARK_N lanes since
    // the LSQ (unlike branch_rs) can have several entries become ready
    // simultaneously. A lane's tag field is don't-care when its valid
    // bit is 0.
    input [EXTRA_MARK_N-1:0] extra_mark_valid,
    input [EXTRA_MARK_N*$clog2(DEPTH)-1:0] extra_mark_tag_flat,

    // Misprediction squash (Phase 2): roll tail_ptr back to just past
    // squash_tag (a mispredicted conditional branch's own tag), discarding
    // every younger, speculatively-allocated entry. squash_tag is always
    // a currently-resident (allocated, not-yet-committed) tag -- the
    // branch that just resolved -- so it's always within
    // [head_ptr, head_ptr+count) and the age-based count recomputation
    // below can't underflow. Independent of, and safe to fire the same
    // cycle as, a completely unrelated older entry's commit_req (head_ptr
    // advances on its own normal path; only tail_ptr/count are touched
    // here).
    input squash_valid,
    input [$clog2(DEPTH)-1:0] squash_tag,

    // Combinational value lookups by tag, for dispatch's operand-capture
    // logic. A tag whose entry already broadcast on the CDB (done_arr=1)
    // but hasn't committed yet is otherwise invisible to a dispatching
    // consumer: the CDB bypass only covers the exact cycle of the
    // broadcast itself (one-shot pulse, not latched), and the RAT's
    // busy/tag mapping for the destination register isn't cleared until
    // commit. Without this lookup, a consumer dispatching in that gap
    // parks itself waiting to snoop a CDB tag that will never broadcast
    // again -- a real deadlock found by tracing a backward-branch loop
    // where `addi x6,x6,1` dispatched exactly one cycle after x6's
    // producer had already broadcast. Safe for any tag the caller passes
    // while the corresponding RAT entry is still busy, since RAT's
    // busy/tag is kept in sync with an actual outstanding (allocated,
    // not-yet-committed) ROB entry by construction.
    input [$clog2(DEPTH)-1:0] lookup1_tag,
    output lookup1_done,
    output [63:0] lookup1_value,
    input [$clog2(DEPTH)-1:0] lookup2_tag,
    output lookup2_done,
    output [63:0] lookup2_value,

    // Phase 3: two more lookup ports, same convention, for lane 1's own
    // rs1/rs2 (a second, independently-dispatching instruction in the
    // same cycle needs the exact same three-way readiness determination
    // lane 0 already does).
    input [$clog2(DEPTH)-1:0] lookup3_tag,
    output lookup3_done,
    output [63:0] lookup3_value,
    input [$clog2(DEPTH)-1:0] lookup4_tag,
    output lookup4_done,
    output [63:0] lookup4_value,

    // Phase 16: two more lookup ports, same convention, for lane 2's own
    // rs1/rs2 readiness determination (3-wide dispatch).
    input [$clog2(DEPTH)-1:0] lookup5_tag,
    output lookup5_done,
    output [63:0] lookup5_value,
    input [$clog2(DEPTH)-1:0] lookup6_tag,
    output lookup6_done,
    output [63:0] lookup6_value,

    // Phase 6: two more lookup ports, same convention, reading
    // vec_value_arr (VLEN bits) instead of value_arr -- for vec_rat's
    // own dispatch-time operand readiness (vs1/vs2), which needs the
    // exact same three-way determination (not-busy / same-cycle
    // broadcast bypass / already-done-in-the-ROB) as scalar dispatch,
    // just against the vector value array.
    input [$clog2(DEPTH)-1:0] vec_lookup1_tag,
    output vec_lookup1_done,
    output [VLEN-1:0] vec_lookup1_value,
    input [$clog2(DEPTH)-1:0] vec_lookup2_tag,
    output vec_lookup2_done,
    output [VLEN-1:0] vec_lookup2_value,

    // Commit: retire the head entry, strictly in program order.
    output head_ready,   // head entry exists and is marked done
    output [$clog2(DEPTH)-1:0] head_tag,
    output head_has_dest,
    output [4:0] head_rd,
    output [63:0] head_value,
    output [VLEN-1:0] head_vec_value,
    output head_is_vec_dest,
    output head_is_store,
    output head_is_ecall,
    input commit_req,    // pulse to actually retire the head this cycle

    // Widened commit: a second port for the head+1 entry, retiring
    // alongside the head in the same cycle. head2_ready additionally
    // requires count>=2 (head+1 must actually be a valid, allocated
    // entry, not stale leftover state past the tail). The caller decides
    // commit_req2 (e.g. gating out a second same-cycle store commit, since
    // data_memory.v has only one write port -- see riscv64_ooo_proc.v);
    // this module also defensively requires do_commit1 (head itself
    // committing) before ever acting on commit_req2, since head+1 can
    // never retire ahead of head -- in-order commit stays enforced here,
    // not just trusted from the caller.
    output head2_ready,
    output [$clog2(DEPTH)-1:0] head2_tag,
    output head2_has_dest,
    output [4:0] head2_rd,
    output [63:0] head2_value,
    output [VLEN-1:0] head2_vec_value,
    output head2_is_vec_dest,
    output head2_is_store,
    output head2_is_ecall,
    input commit_req2,

    // Phase 16: a third commit port for the head+2 entry, retiring
    // alongside head and head2 in the same cycle (3-wide commit). Same
    // defensive convention as head2_ready: additionally requires count>=3,
    // and this module requires do_commit2 (head+1 itself retiring) before
    // ever acting on commit_req3 -- head+2 can never retire ahead of
    // head+1, same in-order enforcement head2 already gives head.
    output head3_ready,
    output [$clog2(DEPTH)-1:0] head3_tag,
    output head3_has_dest,
    output [4:0] head3_rd,
    output [63:0] head3_value,
    output [VLEN-1:0] head3_vec_value,
    output head3_is_vec_dest,
    output head3_is_store,
    output head3_is_ecall,
    input commit_req3,

    // Phase 9 (ECC): protects only value_arr[] (the scalar payload that
    // ultimately becomes committed architectural register state), not
    // vec_value_arr[] -- consistent with vector's already-narrower,
    // thread-0-only scope everywhere else in this project -- and not the
    // small per-entry control fields (valid/done/has_dest/rd/etc.), same
    // "bulk data, not small control metadata" scope as l1_cache.v/
    // l2_cache.v/ecc_register_file.v. See the ECC section below for why
    // fault *correction* applies to every read port but fault *counting*
    // is scoped to the commit-time (head/head2) reads only.
    output ecc_rob_sbe_fault,
    output ecc_rob_dbe_fault
);
    localparam TB = $clog2(DEPTH);

    reg [TB:0] count;              // 0..DEPTH, one extra bit of headroom
    reg [TB-1:0] head_ptr, tail_ptr;

    reg valid_arr    [0:DEPTH-1];
    reg done_arr      [0:DEPTH-1];
    reg has_dest_arr  [0:DEPTH-1];
    reg [4:0] rd_arr   [0:DEPTH-1];
    reg [63:0] value_arr[0:DEPTH-1];
    reg [7:0] value_check[0:DEPTH-1];  // Phase 9 (ECC): SECDED check bits for value_arr
    reg [VLEN-1:0] vec_value_arr[0:DEPTH-1];
    reg is_vec_dest_arr[0:DEPTH-1];
    reg is_store_arr [0:DEPTH-1];
    reg is_ecall_arr [0:DEPTH-1];

    assign full  = (count == DEPTH);
    assign empty = (count == 0);
    assign alloc_tag  = tail_ptr;
    assign alloc2_tag = tail_ptr + 1'b1;
    assign alloc3_tag = tail_ptr + 2'd2;
    assign free_count = DEPTH - count;

    assign head_ready     = !empty && done_arr[head_ptr];
    assign head_tag        = head_ptr;
    assign head_has_dest   = has_dest_arr[head_ptr];
    assign head_rd         = rd_arr[head_ptr];
    assign head_vec_value  = vec_value_arr[head_ptr];
    assign head_is_vec_dest = is_vec_dest_arr[head_ptr];
    assign head_is_store   = is_store_arr[head_ptr];
    assign head_is_ecall   = is_ecall_arr[head_ptr];

    wire [TB-1:0] head2_ptr = head_ptr + 1'b1;
    assign head2_ready     = (count >= 2) && done_arr[head2_ptr];
    assign head2_tag        = head2_ptr;
    assign head2_has_dest   = has_dest_arr[head2_ptr];
    assign head2_rd         = rd_arr[head2_ptr];
    assign head2_vec_value  = vec_value_arr[head2_ptr];
    assign head2_is_vec_dest = is_vec_dest_arr[head2_ptr];
    assign head2_is_store   = is_store_arr[head2_ptr];
    assign head2_is_ecall   = is_ecall_arr[head2_ptr];

    wire [TB-1:0] head3_ptr = head_ptr + 2'd2;
    assign head3_ready     = (count >= 3) && done_arr[head3_ptr];
    assign head3_tag        = head3_ptr;
    assign head3_has_dest   = has_dest_arr[head3_ptr];
    assign head3_rd         = rd_arr[head3_ptr];
    assign head3_vec_value  = vec_value_arr[head3_ptr];
    assign head3_is_vec_dest = is_vec_dest_arr[head3_ptr];
    assign head3_is_store   = is_store_arr[head3_ptr];
    assign head3_is_ecall   = is_ecall_arr[head3_ptr];

    assign lookup1_done  = done_arr[lookup1_tag];
    assign lookup2_done  = done_arr[lookup2_tag];
    assign lookup3_done  = done_arr[lookup3_tag];
    assign lookup4_done  = done_arr[lookup4_tag];
    assign lookup5_done  = done_arr[lookup5_tag];
    assign lookup6_done  = done_arr[lookup6_tag];

    // ---- Phase 9 (ECC): one decode instance per read port. Correction
    // is applied on every one of them (lookup1-4 included -- a
    // dispatching consumer capturing a bad operand value straight out of
    // the ROB would be a real, if quieter, failure mode than a corrupted
    // commit), but *counting* a fault into ecc_rob_sbe_fault/dbe_fault
    // below is deliberately scoped to just the commit-time (head/head2)
    // reads: head_ready/head2_ready already cleanly answer "is this read
    // meaningful right now", whereas a lookup port's tag input is simply
    // always driven to *something* by the caller's dispatch-stage wires,
    // valid or not, even on a cycle nothing is really consulting that
    // port -- there's no equally clean "is the caller actually using this
    // lookup right now" signal on this module's own ports to gate on
    // (unlike ecc_register_file.v's read_reg1/2 != x0, or l1_cache.v's
    // request-valid lines), so counting lookup-port faults would risk
    // reporting noise from an untouched, don't-care-value tag as if it
    // were a real fault.
    wire [63:0] lookup1_corrected, lookup2_corrected, lookup3_corrected, lookup4_corrected;
    wire [63:0] lookup5_corrected, lookup6_corrected;
    wire lookup1_sbe, lookup1_dbe, lookup2_sbe, lookup2_dbe;
    wire lookup3_sbe, lookup3_dbe, lookup4_sbe, lookup4_dbe;
    wire lookup5_sbe, lookup5_dbe, lookup6_sbe, lookup6_dbe;
    assign lookup1_value = lookup1_corrected;
    assign lookup2_value = lookup2_corrected;
    assign lookup3_value = lookup3_corrected;
    assign lookup4_value = lookup4_corrected;
    assign lookup5_value = lookup5_corrected;
    assign lookup6_value = lookup6_corrected;
    ecc64 ecc_lookup1_i (.wr_data(64'b0), .wr_check(),
        .rd_data(value_arr[lookup1_tag]), .rd_check(value_check[lookup1_tag]),
        .rd_data_corrected(lookup1_corrected), .rd_sbe(lookup1_sbe), .rd_dbe(lookup1_dbe));
    ecc64 ecc_lookup2_i (.wr_data(64'b0), .wr_check(),
        .rd_data(value_arr[lookup2_tag]), .rd_check(value_check[lookup2_tag]),
        .rd_data_corrected(lookup2_corrected), .rd_sbe(lookup2_sbe), .rd_dbe(lookup2_dbe));
    ecc64 ecc_lookup3_i (.wr_data(64'b0), .wr_check(),
        .rd_data(value_arr[lookup3_tag]), .rd_check(value_check[lookup3_tag]),
        .rd_data_corrected(lookup3_corrected), .rd_sbe(lookup3_sbe), .rd_dbe(lookup3_dbe));
    ecc64 ecc_lookup4_i (.wr_data(64'b0), .wr_check(),
        .rd_data(value_arr[lookup4_tag]), .rd_check(value_check[lookup4_tag]),
        .rd_data_corrected(lookup4_corrected), .rd_sbe(lookup4_sbe), .rd_dbe(lookup4_dbe));
    ecc64 ecc_lookup5_i (.wr_data(64'b0), .wr_check(),
        .rd_data(value_arr[lookup5_tag]), .rd_check(value_check[lookup5_tag]),
        .rd_data_corrected(lookup5_corrected), .rd_sbe(lookup5_sbe), .rd_dbe(lookup5_dbe));
    ecc64 ecc_lookup6_i (.wr_data(64'b0), .wr_check(),
        .rd_data(value_arr[lookup6_tag]), .rd_check(value_check[lookup6_tag]),
        .rd_data_corrected(lookup6_corrected), .rd_sbe(lookup6_sbe), .rd_dbe(lookup6_dbe));

    wire [63:0] head_corrected, head2_corrected, head3_corrected;
    wire head_val_sbe, head_val_dbe, head2_val_sbe, head2_val_dbe, head3_val_sbe, head3_val_dbe;
    assign head_value  = head_corrected;
    assign head2_value = head2_corrected;
    assign head3_value = head3_corrected;
    ecc64 ecc_head_i (.wr_data(64'b0), .wr_check(),
        .rd_data(value_arr[head_ptr]), .rd_check(value_check[head_ptr]),
        .rd_data_corrected(head_corrected), .rd_sbe(head_val_sbe), .rd_dbe(head_val_dbe));
    ecc64 ecc_head2_i (.wr_data(64'b0), .wr_check(),
        .rd_data(value_arr[head2_ptr]), .rd_check(value_check[head2_ptr]),
        .rd_data_corrected(head2_corrected), .rd_sbe(head2_val_sbe), .rd_dbe(head2_val_dbe));
    ecc64 ecc_head3_i (.wr_data(64'b0), .wr_check(),
        .rd_data(value_arr[head3_ptr]), .rd_check(value_check[head3_ptr]),
        .rd_data_corrected(head3_corrected), .rd_sbe(head3_val_sbe), .rd_dbe(head3_val_dbe));

    assign ecc_rob_sbe_fault = (head_ready && head_val_sbe) || (head2_ready && head2_val_sbe) || (head3_ready && head3_val_sbe);
    assign ecc_rob_dbe_fault = (head_ready && head_val_dbe) || (head2_ready && head2_val_dbe) || (head3_ready && head3_val_dbe);

    // ---- Phase 9 (ECC): encode instances for the scalar CDB mark ports
    // (mark_valid/mark_b_valid/mark_c_valid) -- the only writers of
    // value_arr.
    wire [7:0] mark_check, mark_b_check, mark_c_check;
    ecc64 ecc_mark_i   (.wr_data(mark_value),   .wr_check(mark_check),
        .rd_data(64'b0), .rd_check(8'b0), .rd_data_corrected(), .rd_sbe(), .rd_dbe());
    ecc64 ecc_mark_b_i (.wr_data(mark_b_value), .wr_check(mark_b_check),
        .rd_data(64'b0), .rd_check(8'b0), .rd_data_corrected(), .rd_sbe(), .rd_dbe());
    ecc64 ecc_mark_c_i (.wr_data(mark_c_value), .wr_check(mark_c_check),
        .rd_data(64'b0), .rd_check(8'b0), .rd_data_corrected(), .rd_sbe(), .rd_dbe());

    assign vec_lookup1_done  = done_arr[vec_lookup1_tag];
    assign vec_lookup1_value = vec_value_arr[vec_lookup1_tag];
    assign vec_lookup2_done  = done_arr[vec_lookup2_tag];
    assign vec_lookup2_value = vec_value_arr[vec_lookup2_tag];

    // Internally re-derived, defensive re-check against free_count --
    // mirrors the existing single-issue convention where the caller
    // already gates dispatch on !full/free_count but this module still
    // double-checks before actually writing.
    wire do_alloc1 = alloc_req  && (free_count >= 1);
    wire do_alloc2 = alloc2_req && (free_count >= 2);
    wire do_alloc3 = alloc3_req && (free_count >= 3);

    // Same defensive convention for commit: do_commit2 additionally
    // requires do_commit1 (see head2_ready's own port comment).
    wire do_commit1 = commit_req  && head_ready;
    wire do_commit2 = commit_req2 && head2_ready && do_commit1;
    wire do_commit3 = commit_req3 && head3_ready && do_commit2;

    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            count <= {(TB+1){1'b0}};
            head_ptr <= {TB{1'b0}};
            tail_ptr <= {TB{1'b0}};
            for (i = 0; i < DEPTH; i = i + 1) begin
                valid_arr[i] <= 1'b0;
                done_arr[i]  <= 1'b0;
                value_check[i] <= 8'b0;
            end
        end else begin
            // Allocate (up to 2, lane 0 then lane 1) and commit (up to 1)
            // can all fire this cycle; net occupancy change handles them
            // together. squash_valid takes priority over allocation (a
            // well-formed caller never asserts both -- dispatch is
            // suppressed the same cycle a misprediction is discovered --
            // but this ordering makes that the defined, safe behavior
            // even if it ever did), and composes independently with a
            // same-cycle, unrelated older commit (head_ptr's own advance
            // below is untouched by squash; only tail_ptr/count change).
            if (squash_valid) begin
                tail_ptr <= squash_tag + 1'b1;
            end else begin
                if (do_alloc1) begin
                    valid_arr[tail_ptr]     <= 1'b1;
                    done_arr[tail_ptr]      <= 1'b0;
                    has_dest_arr[tail_ptr]  <= alloc_has_dest;
                    rd_arr[tail_ptr]        <= alloc_rd;
                    is_vec_dest_arr[tail_ptr] <= alloc_is_vec_dest;
                    is_store_arr[tail_ptr]  <= alloc_is_store;
                    is_ecall_arr[tail_ptr]  <= alloc_is_ecall;
                end
                if (do_alloc2) begin
                    valid_arr[tail_ptr + 1'b1]     <= 1'b1;
                    done_arr[tail_ptr + 1'b1]      <= 1'b0;
                    has_dest_arr[tail_ptr + 1'b1]  <= alloc2_has_dest;
                    rd_arr[tail_ptr + 1'b1]        <= alloc2_rd;
                    is_vec_dest_arr[tail_ptr + 1'b1] <= alloc2_is_vec_dest;
                    is_store_arr[tail_ptr + 1'b1]  <= alloc2_is_store;
                    is_ecall_arr[tail_ptr + 1'b1]  <= alloc2_is_ecall;
                end
                if (do_alloc3) begin
                    valid_arr[tail_ptr + 2'd2]     <= 1'b1;
                    done_arr[tail_ptr + 2'd2]      <= 1'b0;
                    has_dest_arr[tail_ptr + 2'd2]  <= alloc3_has_dest;
                    rd_arr[tail_ptr + 2'd2]        <= alloc3_rd;
                    is_vec_dest_arr[tail_ptr + 2'd2] <= alloc3_is_vec_dest;
                    is_store_arr[tail_ptr + 2'd2]  <= alloc3_is_store;
                    is_ecall_arr[tail_ptr + 2'd2]  <= alloc3_is_ecall;
                end
                tail_ptr <= tail_ptr + (do_alloc1 ? 1'b1 : 1'b0) + (do_alloc2 ? 1'b1 : 1'b0) + (do_alloc3 ? 1'b1 : 1'b0);
            end
            if (do_commit1) begin
                valid_arr[head_ptr] <= 1'b0;
            end
            if (do_commit2) begin
                valid_arr[head2_ptr] <= 1'b0;
            end
            if (do_commit3) begin
                valid_arr[head3_ptr] <= 1'b0;
            end
            head_ptr <= head_ptr + (do_commit1 ? 1'b1 : 1'b0) + (do_commit2 ? 1'b1 : 1'b0) + (do_commit3 ? 1'b1 : 1'b0);
            if (squash_valid) begin
                // squash_tag is always within [head_ptr, head_ptr+count),
                // so (squash_tag - head_ptr) is a safe, non-negative
                // TB-bit distance -- +1 makes it inclusive of the branch's
                // own (kept) entry, then -1 more per unrelated older entry
                // *also* retiring the head(s) this same cycle.
                count <= {1'b0, (squash_tag - head_ptr)} + 1'b1
                               - (do_commit1 ? 1'b1 : 1'b0) - (do_commit2 ? 1'b1 : 1'b0) - (do_commit3 ? 1'b1 : 1'b0);
            end else begin
                count <= count + (do_alloc1 ? 1'b1 : 1'b0) + (do_alloc2 ? 1'b1 : 1'b0) + (do_alloc3 ? 1'b1 : 1'b0)
                               - (do_commit1 ? 1'b1 : 1'b0) - (do_commit2 ? 1'b1 : 1'b0) - (do_commit3 ? 1'b1 : 1'b0);
            end

            // mark_valid/mark2_valid are independent of alloc/commit and
            // never target the tag either of those touches this same
            // cycle in a well-formed caller (alloc's tag is brand new and
            // can't have a result the same cycle it's created; commit's
            // tag is being freed, not marked), so there's no same-cycle
            // ordering dependency to worry about here, unlike rat.v's
            // write_en vs commit_clear_en interaction. mark_valid,
            // mark_b_valid and mark_c_valid can fire in the same cycle for
            // three different tags safely (always distinct, by the ROB's
            // own allocation invariant) -- that's the whole reason mark_b/
            // mark_c exist as genuinely separate ports instead of
            // something arbitrated.
            if (mark_valid) begin
                done_arr[mark_tag]  <= 1'b1;
                value_arr[mark_tag] <= mark_value;
                value_check[mark_tag] <= mark_check;
            end
            if (mark_b_valid) begin
                done_arr[mark_b_tag]  <= 1'b1;
                value_arr[mark_b_tag] <= mark_b_value;
                value_check[mark_b_tag] <= mark_b_check;
            end
            if (mark_c_valid) begin
                done_arr[mark_c_tag]  <= 1'b1;
                value_arr[mark_c_tag] <= mark_c_value;
                value_check[mark_c_tag] <= mark_c_check;
            end
            if (vec_mark_valid) begin
                done_arr[vec_mark_tag]      <= 1'b1;
                vec_value_arr[vec_mark_tag] <= vec_mark_value;
            end
            if (mark2_valid) begin
                done_arr[mark2_tag] <= 1'b1;
            end
            for (i = 0; i < EXTRA_MARK_N; i = i + 1) begin
                if (extra_mark_valid[i])
                    done_arr[extra_mark_tag_flat[(i+1)*$clog2(DEPTH)-1 -: $clog2(DEPTH)]] <= 1'b1;
            end
        end
    end
endmodule
