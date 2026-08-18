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
    parameter EXTRA_MARK_N = 4
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
    output [$clog2(DEPTH)-1:0] alloc2_tag,
    output [$clog2(DEPTH):0] free_count,   // 0..DEPTH

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

    // Commit: retire the head entry, strictly in program order.
    output head_ready,   // head entry exists and is marked done
    output [$clog2(DEPTH)-1:0] head_tag,
    output head_has_dest,
    output [4:0] head_rd,
    output [63:0] head_value,
    output head_is_store,
    output head_is_ecall,
    input commit_req     // pulse to actually retire the head this cycle
);
    localparam TB = $clog2(DEPTH);

    reg [TB:0] count;              // 0..DEPTH, one extra bit of headroom
    reg [TB-1:0] head_ptr, tail_ptr;

    reg valid_arr    [0:DEPTH-1];
    reg done_arr      [0:DEPTH-1];
    reg has_dest_arr  [0:DEPTH-1];
    reg [4:0] rd_arr   [0:DEPTH-1];
    reg [63:0] value_arr[0:DEPTH-1];
    reg is_store_arr [0:DEPTH-1];
    reg is_ecall_arr [0:DEPTH-1];

    assign full  = (count == DEPTH);
    assign empty = (count == 0);
    assign alloc_tag  = tail_ptr;
    assign alloc2_tag = tail_ptr + 1'b1;
    assign free_count = DEPTH - count;

    assign head_ready     = !empty && done_arr[head_ptr];
    assign head_tag        = head_ptr;
    assign head_has_dest   = has_dest_arr[head_ptr];
    assign head_rd         = rd_arr[head_ptr];
    assign head_value      = value_arr[head_ptr];
    assign head_is_store   = is_store_arr[head_ptr];
    assign head_is_ecall   = is_ecall_arr[head_ptr];

    assign lookup1_done  = done_arr[lookup1_tag];
    assign lookup1_value = value_arr[lookup1_tag];
    assign lookup2_done  = done_arr[lookup2_tag];
    assign lookup2_value = value_arr[lookup2_tag];
    assign lookup3_done  = done_arr[lookup3_tag];
    assign lookup3_value = value_arr[lookup3_tag];
    assign lookup4_done  = done_arr[lookup4_tag];
    assign lookup4_value = value_arr[lookup4_tag];

    // Internally re-derived, defensive re-check against free_count --
    // mirrors the existing single-issue convention where the caller
    // already gates dispatch on !full/free_count but this module still
    // double-checks before actually writing.
    wire do_alloc1 = alloc_req  && (free_count >= 1);
    wire do_alloc2 = alloc2_req && (free_count >= 2);

    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            count <= {(TB+1){1'b0}};
            head_ptr <= {TB{1'b0}};
            tail_ptr <= {TB{1'b0}};
            for (i = 0; i < DEPTH; i = i + 1) begin
                valid_arr[i] <= 1'b0;
                done_arr[i]  <= 1'b0;
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
                    is_store_arr[tail_ptr]  <= alloc_is_store;
                    is_ecall_arr[tail_ptr]  <= alloc_is_ecall;
                end
                if (do_alloc2) begin
                    valid_arr[tail_ptr + 1'b1]     <= 1'b1;
                    done_arr[tail_ptr + 1'b1]      <= 1'b0;
                    has_dest_arr[tail_ptr + 1'b1]  <= alloc2_has_dest;
                    rd_arr[tail_ptr + 1'b1]        <= alloc2_rd;
                    is_store_arr[tail_ptr + 1'b1]  <= alloc2_is_store;
                    is_ecall_arr[tail_ptr + 1'b1]  <= alloc2_is_ecall;
                end
                tail_ptr <= tail_ptr + (do_alloc1 ? 1'b1 : 1'b0) + (do_alloc2 ? 1'b1 : 1'b0);
            end
            if (commit_req && head_ready) begin
                valid_arr[head_ptr] <= 1'b0;
                head_ptr <= head_ptr + 1'b1;
            end
            if (squash_valid) begin
                // squash_tag is always within [head_ptr, head_ptr+count),
                // so (squash_tag - head_ptr) is a safe, non-negative
                // TB-bit distance -- +1 makes it inclusive of the branch's
                // own (kept) entry, then -1 more if an unrelated older
                // commit is *also* retiring the head this same cycle.
                count <= {1'b0, (squash_tag - head_ptr)} + 1'b1 - ((commit_req && head_ready) ? 1'b1 : 1'b0);
            end else begin
                count <= count + (do_alloc1 ? 1'b1 : 1'b0) + (do_alloc2 ? 1'b1 : 1'b0)
                               - ((commit_req && head_ready) ? 1'b1 : 1'b0);
            end

            // mark_valid/mark2_valid are independent of alloc/commit and
            // never target the tag either of those touches this same
            // cycle in a well-formed caller (alloc's tag is brand new and
            // can't have a result the same cycle it's created; commit's
            // tag is being freed, not marked), so there's no same-cycle
            // ordering dependency to worry about here, unlike rat.v's
            // write_en vs commit_clear_en interaction. mark_valid and
            // mark2_valid can fire in the same cycle for two different
            // tags safely (always distinct, by the ROB's own allocation
            // invariant) -- that's the whole reason mark2 exists as a
            // genuinely separate port instead of something arbitrated.
            if (mark_valid) begin
                done_arr[mark_tag]  <= 1'b1;
                value_arr[mark_tag] <= mark_value;
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
