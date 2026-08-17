`timescale 1ns / 1ps
// 32 x 64-bit register file. x0 hardwired to zero. Reads are asynchronous;
// writes are synchronous (same edge that advances the PC).
module register_file (
    input clk,
    input reset,
    input reg_write,
    input  [4:0] read_reg1,
    input  [4:0] read_reg2,
    input  [4:0] write_reg,
    input  [63:0] write_data,
    output reg [63:0] read_data1,
    output reg [63:0] read_data2
);
    reg [63:0] registers [0:31];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 64'b0;
        end
        else if (reg_write && write_reg != 5'd0) begin
            registers[write_reg] <= write_data;
        end
    end

    always @(*) begin
        read_data1 = (read_reg1 == 5'd0) ? 64'b0 : registers[read_reg1];
        read_data2 = (read_reg2 == 5'd0) ? 64'b0 : registers[read_reg2];
    end
endmodule
