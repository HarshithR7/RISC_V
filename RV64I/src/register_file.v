`timescale 1ns / 1ps
// 32 x 64-bit register file. x0 hardwired to zero. Reads are asynchronous;
// writes are synchronous (same edge that advances the PC).
//
// Second write port (reg_write2/write_reg2/write_data2) added for
// RV64I_OOO's widened (2-wide) commit path -- see that project's README
// for why commit needed to widen at all. Backward-compatible with every
// existing single-write caller (this file's original RV64I single-cycle
// user): leaving the new port unconnected reads reg_write2 as 'x', and
// Verilog's `if` treats an X condition as false, so the second port is a
// safe no-op when unused -- same pattern already used throughout
// RV64I_OOO for extending shared modules without touching existing
// callers (e.g. rob.v's extra_mark ports). When both ports write the
// same register in the same cycle, port 2 wins (applied second in
// program order below, last-non-blocking-assignment-wins) -- the
// intended semantics for RV64I_OOO's dual in-order commit, where port 2
// is always the younger of the two simultaneously-committing
// instructions.
module register_file (
    input clk,
    input reset,
    input reg_write,
    input  [4:0] read_reg1,
    input  [4:0] read_reg2,
    input  [4:0] write_reg,
    input  [63:0] write_data,
    output reg [63:0] read_data1,
    output reg [63:0] read_data2,

    input reg_write2,
    input [4:0] write_reg2,
    input [63:0] write_data2
);
    reg [63:0] registers [0:31];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 64'b0;
        end
        else begin
            if (reg_write && write_reg != 5'd0) begin
                registers[write_reg] <= write_data;
            end
            if (reg_write2 && write_reg2 != 5'd0) begin
                registers[write_reg2] <= write_data2;
            end
        end
    end

    always @(*) begin
        read_data1 = (read_reg1 == 5'd0) ? 64'b0 : registers[read_reg1];
        read_data2 = (read_reg2 == 5'd0) ? 64'b0 : registers[read_reg2];
    end
endmodule
