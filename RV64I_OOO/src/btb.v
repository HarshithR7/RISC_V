`timescale 1ns / 1ps
// Phase 17: Branch Target Buffer -- scoped specifically to non-return
// indirect JALR target prediction, not conditional-branch targets.
//
// RV64I conditional-branch targets are pc+imm -- already a cheap,
// immediate, same-cycle decode-time computation (see
// riscv64_ooo_proc.v's t0_next_pc: `t0_pc_latched + d0_imm`); a BTB adds
// nothing there. The real gap is non-return JALR (computed calls,
// virtual dispatch, switch/jump tables): today those always stall fetch
// completely (t0_jalr_stall_active) with zero prediction, since ras.v
// only predicts the return-shaped case. This table fills exactly that
// gap -- riscv64_ooo_proc.v feeds it whichever PC is at decode
// (`predict_pc = pc`, the same muxed, latched wire bht.v already reads),
// and on a hit for a non-return JALR, speculatively redirects to the
// stored target using the *same* checkpoint/mispredict/squash machinery
// already proven out for RAS-predicted returns (t0_spec_active/
// t0_predicted_target_reg/t0_mispredict are already generic over how the
// prediction was sourced -- see that file's header addendum).
//
// Simple direct-mapped table, no tags -- same aliasing-accepted
// simplicity convention bht.v already established (a 64-entry, no-tag,
// PC-indexed table): any misprediction, whichever table it came from, is
// caught by the same resolved-vs-predicted compare and recovered
// identically, so aliasing only costs prediction accuracy, never
// correctness. Single shared instance across both threads (again
// matching bht.v) means thread 0 and thread 1's independent call sites
// can alias and evict each other's entries -- flagged here as a real,
// deliberate scope cut (a target misprediction is costlier to recover
// from than a BHT direction misprediction), with `{tid, pc[...]}`
// indexing available later as a one-line fix if measured accuracy ever
// warrants it.
module btb #(
    parameter INDEX_BITS = 6  // 64 entries, matches bht.v
)(
    input clk,
    input reset,

    input [63:0] predict_pc,
    output predict_valid,       // combinational read
    output [63:0] predict_target,

    input update_valid,
    input [63:0] update_pc,
    input [63:0] update_target
);
    localparam ENTRIES = (1 << INDEX_BITS);

    reg valid [0:ENTRIES-1];
    reg [63:0] target [0:ENTRIES-1];

    wire [INDEX_BITS-1:0] predict_idx = predict_pc[INDEX_BITS+1:2];
    wire [INDEX_BITS-1:0] update_idx  = update_pc[INDEX_BITS+1:2];

    assign predict_valid  = valid[predict_idx];
    assign predict_target = target[predict_idx];

    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < ENTRIES; i = i + 1)
                valid[i] <= 1'b0;
        end else if (update_valid) begin
            // Unconditional overwrite -- always incorporate the latest
            // resolved outcome, same convention as bht.v's own saturating
            // counter always updating toward what just happened.
            valid[update_idx]  <= 1'b1;
            target[update_idx] <= update_target;
        end
    end
endmodule
