`timescale 1ns / 1ps
// Return Address Stack: speculative target prediction for return-shaped
// JALR (`jalr x0, 0(ra)`-style -- rd is not a link register, rs1 is), the
// one class of indirect jump this design predicts at all. Every other
// JALR (indirect calls through a function pointer, jump-table dispatch,
// `jalr ra, 0(ra)`-style calls where rd IS a link register) still falls
// back to riscv64_ooo_proc.v's original Phase 1 stall-until-resolved path
// -- this module makes no attempt to predict those, since a plain LIFO has
// no way to guess an address it was never told.
//
// Call/return classification lives in riscv64_ooo_proc.v (ras_push_it /
// ras_pop_it), not here -- this module is a bare LIFO with a combinational
// top-of-stack read, nothing more.
//
// No checkpoint/restore port, unlike rat.v's speculation support: branch_rs
// (this design's branch-class reservation station) holds exactly one
// resident entry, and blocks any new branch-class dispatch -- push and pop
// both only ever happen from a branch-class instruction -- until the
// resident one fully resolves (see branch_rs.v's own header). That
// serialization means a push or pop here can never happen while an earlier
// branch/JALR's own misprediction is still undetected: by the time a new
// branch-class instruction (and therefore any RAS push/pop) reaches
// dispatch, any earlier speculative window has already resolved and, if it
// mispredicted, already squashed -- so there is never a "wrong-path" push
// to unwind. And a return-shaped JALR's own pop is never wrong to have
// issued regardless of whether the *target it predicted* turns out right:
// the instruction really is a return (that's fixed by its opcode/operand
// shape, not by prediction outcome), so the matching call's stack slot is
// genuinely consumed either way -- only the predicted *target* can be
// wrong, and that's corrected the same way a mispredicted branch is
// (riscv64_ooo_proc.v's existing squash/redirect path), with no RAS state
// to roll back.
module ras #(
    parameter DEPTH = 8
)(
    input clk,
    input reset,

    // Combinational top-of-stack read -- this cycle's return prediction,
    // valid only when top_valid.
    output [63:0] top_addr,
    output top_valid,

    // Call: push a return address. Silently dropped (stack held at DEPTH)
    // if already full -- a safe degrade to "this deep call's eventual
    // return just won't be predicted," not a correctness issue.
    input push_req,
    input [63:0] push_addr,

    // Return: pop the entry that top_addr/top_valid just supplied.
    // push_req and pop_req are always mutually exclusive from the caller
    // (a single instruction is classified as a call XOR a return, never
    // both -- see riscv64_ooo_proc.v's ras_push_it/ras_pop_it), so the
    // two need no combined-same-cycle handling here.
    input pop_req
);
    localparam SP_BITS = $clog2(DEPTH+1);

    reg [63:0] stack [0:DEPTH-1];
    reg [SP_BITS-1:0] sp;

    assign top_valid = (sp != {SP_BITS{1'b0}});
    assign top_addr  = stack[sp - 1'b1];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            sp <= {SP_BITS{1'b0}};
        end else if (pop_req && top_valid) begin
            sp <= sp - 1'b1;
        end else if (push_req && (sp != DEPTH)) begin
            stack[sp] <= push_addr;
            sp <= sp + 1'b1;
        end
    end
endmodule
