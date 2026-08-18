`timescale 1ns / 1ps
// Divide reservation station: wraps the already-unit-verified, multi-cycle
// div_fu.v with a single RS entry (DIV_RS is fixed at exactly 1 entry, per
// the original Phase 1 sizing plan -- same "correct by construction, not a
// knob worth exposing" reasoning as branch_rs.v, though here it's simply
// that this scoped core never needs more than one divide in flight at
// once, not a structural argument like branch_rs's stall-until-resolved
// policy). Unlike alu_rs.v/mul_rs.v (combinational, 1-cycle FUs), the
// wrapped FU takes ~34-66 cycles, so this module tracks an extra bit of
// state beyond "busy": whether div_fu has been started yet, and whether
// its result has come back and is waiting to broadcast.
module div_rs #(
    parameter TAG_BITS = 3
)(
    input clk,
    input reset,

    input alloc_req,
    input alloc_tid,
    input [2:0] alloc_op,        // muldiv_op: 100=DIV 101=DIVU 110=REM 111=REMU
    input alloc_word_op,
    input alloc_src1_ready,
    input [63:0] alloc_src1_val,
    input [TAG_BITS-1:0] alloc_src1_tag,
    input alloc_src2_ready,
    input [63:0] alloc_src2_val,
    input [TAG_BITS-1:0] alloc_src2_tag,
    input [TAG_BITS-1:0] alloc_dest_tag,
    output full,

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

    output req_valid,
    output req_tid,
    output [TAG_BITS-1:0] req_tag,
    output [63:0] req_value,
    input req_grant,

    // Phase 7 (SMT): one head tag per thread -- see alu_rs.v's header.
    // No issue-priority logic is needed here (single entry, nothing to
    // arbitrate among), just the tid tag itself and thread-aware squash.
    input [TAG_BITS-1:0] rob_head_tag0,
    input [TAG_BITS-1:0] rob_head_tag1,

    // Phase 2 misprediction squash, now two independent per-thread ports
    // (Phase 7) -- see alu_rs.v's header for why a single muxed port can't
    // handle both threads mispredicting in the same cycle. One extra
    // wrinkle unchanged from before: div_fu itself has no abort input, so
    // a squash while it's mid-computation can't stop it, only disown its
    // eventual result (see the `full` comment below).
    input squash0_valid,
    input [TAG_BITS-1:0] squash0_tag,
    input squash1_valid,
    input [TAG_BITS-1:0] squash1_tag
);
    reg busy;
    reg tid_r;
    reg [2:0] op;
    reg word_op_r;
    reg s1_ready; reg [63:0] s1_val; reg [TAG_BITS-1:0] s1_tag;
    reg s2_ready; reg [63:0] s2_val; reg [TAG_BITS-1:0] s2_tag;
    reg [TAG_BITS-1:0] dest_tag;
    reg started;       // div_fu's `start` has already been pulsed for this entry
    reg have_result;   // div_fu's `done` has fired; result_q/result_r valid

    reg [63:0] result_q, result_r;

    // Also full while div_fu is still finishing up a just-squashed
    // division: div_fu can't be aborted, and once it eventually pulses
    // `done`, the unconditional capture below would otherwise land on
    // whatever *new* entry got allocated into this now-"free" slot in the
    // meantime, corrupting an unrelated instruction's result. Keeping
    // `full` asserted until div_fu genuinely goes idle again blocks any
    // reallocation until that stale completion has nowhere left to land.
    assign full = busy || div_busy_w;

    wire is_signed_op  = !op[0];   // DIV/REM = signed, DIVU/REMU = unsigned
    wire is_remainder   = op[1];    // REM/REMU vs DIV/DIVU
    wire operands_ready = s1_ready && s2_ready;

    wire div_busy_w, div_done_w;
    wire [63:0] div_quotient_w, div_remainder_w;
    wire div_start = busy && operands_ready && !started && !div_busy_w;

    div_fu div_fu_i (
        .clk(clk), .reset(reset),
        .start(div_start),
        .is_signed(is_signed_op), .is_word(word_op_r),
        .dividend(s1_val), .divisor(s2_val),
        .busy(div_busy_w), .done(div_done_w),
        .quotient(div_quotient_w), .remainder(div_remainder_w)
    );

    assign req_valid = busy && have_result;
    assign req_tid   = tid_r;
    assign req_tag   = dest_tag;
    assign req_value = is_remainder ? result_r : result_q;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            busy <= 1'b0;
            started <= 1'b0;
            have_result <= 1'b0;
        end else begin
            if (busy && !s1_ready && cdbA_valid && tid_r == cdbA_tid && s1_tag == cdbA_tag) begin
                s1_ready <= 1'b1; s1_val <= cdbA_value;
            end else if (busy && !s1_ready && cdbB_valid && tid_r == cdbB_tid && s1_tag == cdbB_tag) begin
                s1_ready <= 1'b1; s1_val <= cdbB_value;
            end
            if (busy && !s2_ready && cdbA_valid && tid_r == cdbA_tid && s2_tag == cdbA_tag) begin
                s2_ready <= 1'b1; s2_val <= cdbA_value;
            end else if (busy && !s2_ready && cdbB_valid && tid_r == cdbB_tid && s2_tag == cdbB_tag) begin
                s2_ready <= 1'b1; s2_val <= cdbB_value;
            end

            if (alloc_req && !busy) begin
                busy       <= 1'b1;
                tid_r      <= alloc_tid;
                op         <= alloc_op;
                word_op_r  <= alloc_word_op;
                s1_ready   <= alloc_src1_ready;
                s1_val     <= alloc_src1_val;
                s1_tag     <= alloc_src1_tag;
                s2_ready   <= alloc_src2_ready;
                s2_val     <= alloc_src2_val;
                s2_tag     <= alloc_src2_tag;
                dest_tag   <= alloc_dest_tag;
                started     <= 1'b0;
                have_result <= 1'b0;
            end

            if (div_start) begin
                started <= 1'b1;
            end

            if (div_done_w) begin
                result_q    <= div_quotient_w;
                result_r    <= div_remainder_w;
                have_result <= 1'b1;
            end

            if (req_valid && req_grant) begin
                busy <= 1'b0;
            end

            if (squash0_valid && busy && !tid_r &&
                (dest_tag - rob_head_tag0) > (squash0_tag - rob_head_tag0))
                busy <= 1'b0;
            if (squash1_valid && busy && tid_r &&
                (dest_tag - rob_head_tag1) > (squash1_tag - rob_head_tag1))
                busy <= 1'b0;
        end
    end
endmodule
