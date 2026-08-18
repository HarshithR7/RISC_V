`timescale 1ns / 1ps
// Branch History Table: direction prediction for conditional branches
// only (Phase 2's scope -- JAL never needs prediction since its target is
// exact at decode, and JALR still stalls rather than being predicted, see
// riscv64_ooo_proc.v's header for why). A simple, standard 2-bit
// saturating-counter table indexed by PC bits, initialized to "weakly not
// taken" (a real prediction, not a placeholder -- most conditional
// branches in typical code are loop-back edges that vastly outnumber
// forward branches, but backward-vs-forward isn't decoded here, so a
// uniform not-taken bias is the simplest defensible default that still
// gets better than 50/50 accuracy immediately and adapts within two
// resolutions either way).
module bht #(
    parameter INDEX_BITS = 6  // 64 entries
)(
    input clk,
    input reset,

    input [63:0] predict_pc,
    output predict_taken,      // combinational read

    input update_valid,
    input [63:0] update_pc,
    input update_taken
);
    localparam ENTRIES = (1 << INDEX_BITS);

    reg [1:0] counter [0:ENTRIES-1];

    wire [INDEX_BITS-1:0] predict_idx = predict_pc[INDEX_BITS+1:2];
    wire [INDEX_BITS-1:0] update_idx  = update_pc[INDEX_BITS+1:2];

    assign predict_taken = counter[predict_idx][1]; // 2,3 = taken; 0,1 = not-taken

    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < ENTRIES; i = i + 1)
                counter[i] <= 2'b01; // weakly not-taken
        end else if (update_valid) begin
            if (update_taken) begin
                if (counter[update_idx] != 2'b11)
                    counter[update_idx] <= counter[update_idx] + 1'b1;
            end else begin
                if (counter[update_idx] != 2'b00)
                    counter[update_idx] <= counter[update_idx] - 1'b1;
            end
        end
    end
endmodule
