`timescale 1ns / 1ps
// Vector Register Alias Table -- same role as rat.v, for v0-v31 instead
// of x1-x31. Simpler than rat.v in two ways, both direct consequences of
// this core's Phase 6 (DLP) scoping (see riscv64_ooo_proc.v's header):
// vector instructions are single-issue only (always lane 0, like
// branches), so there's no second read/write port or intra-group
// forwarding to build; and v0 is a real, ordinarily-renameable register
// here (RVV's v0.t masking is out of scope for this integration, so v0
// carries no special hardwired behavior the way x0 does in rat.v).
//
// Still needs the same checkpoint/restore machinery as rat.v: a vector
// instruction can dispatch during a speculative window opened by a
// predicted branch, and must be correctly rolled back on misprediction
// exactly like any scalar instruction would be -- driven by the same
// checkpoint_save/checkpoint_restore events from riscv64_ooo_proc.v.
module vec_rat #(
    parameter TAG_BITS = 3
)(
    input clk,
    input reset,

    input [4:0] vs1, vs2,
    output vs1_busy,
    output [TAG_BITS-1:0] vs1_tag,
    output vs2_busy,
    output [TAG_BITS-1:0] vs2_tag,

    input write_en,
    input [4:0] vd,
    input [TAG_BITS-1:0] new_tag,

    input commit_clear_en,
    input [4:0] commit_vd,
    input [TAG_BITS-1:0] commit_tag,

    input checkpoint_save,
    input checkpoint_restore
);
    reg busy [0:31];
    reg [TAG_BITS-1:0] tag [0:31];
    reg busy_cp [0:31];
    reg [TAG_BITS-1:0] tag_cp [0:31];

    assign vs1_busy = busy[vs1];
    assign vs1_tag  = tag[vs1];
    assign vs2_busy = busy[vs2];
    assign vs2_tag  = tag[vs2];

    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                busy[i] <= 1'b0;
                busy_cp[i] <= 1'b0;
            end
        end else begin
            if (commit_clear_en && tag_cp[commit_vd] == commit_tag) begin
                busy_cp[commit_vd] <= 1'b0;
            end

            if (commit_clear_en && tag[commit_vd] == commit_tag) begin
                busy[commit_vd] <= 1'b0;
            end
            if (write_en) begin
                busy[vd] <= 1'b1;
                tag[vd]  <= new_tag;
            end

            if (checkpoint_save) begin
                for (i = 0; i < 32; i = i + 1) begin
                    busy_cp[i] <= busy[i];
                    tag_cp[i]  <= tag[i];
                end
            end

            if (checkpoint_restore) begin
                for (i = 0; i < 32; i = i + 1) begin
                    busy[i] <= busy_cp[i];
                    tag[i]  <= tag_cp[i];
                end
            end
        end
    end
endmodule
