`timescale 1ns / 1ps
// 32 x 64-bit floating-point register file (f0-f31). Unlike the integer
// file, f0 is an ordinary writable register -- F has no hardwired-zero
// register. Single-precision values are NaN-boxed into the lower 32 bits
// with all upper bits set to 1 (RISC-V's convention for storing a
// narrower-than-FLEN value, and what makes this register file already
// D-extension-ready without changes).
module fp_register_file (
    input clk,
    input reset,
    input fp_reg_write,
    input [4:0] read_reg1,
    input [4:0] read_reg2,
    input [4:0] read_reg3,   // rs3, only used by FMADD/FMSUB/FNMSUB/FNMADD
    input [4:0] write_reg,
    input [63:0] write_data,
    output reg [63:0] read_data1,
    output reg [63:0] read_data2,
    output reg [63:0] read_data3
);
    reg [63:0] registers [0:31];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 64'hFFFFFFFF00000000; // NaN-boxed +0.0f
        end
        else if (fp_reg_write) begin
            registers[write_reg] <= write_data;
        end
    end

    always @(*) begin
        read_data1 = registers[read_reg1];
        read_data2 = registers[read_reg2];
        read_data3 = registers[read_reg3];
    end
endmodule
