`timescale 1ns / 1ps
// 32 x 128-bit vector register file (v0-v31). VLEN=128, and this core only
// ever uses SEW=32 (4 elements/register) -- see RV64I/README.md for the
// scope decision. v0 has no hardwired meaning here since masking isn't
// implemented (real RVV reserves v0 as the default mask register).
module vector_register_file (
    input clk,
    input reset,
    input vreg_write,
    input [4:0] read_reg1,   // vs2 (or vs3 for vse32.v -- see riscv64_proc.v)
    input [4:0] read_reg2,   // vs1
    input [4:0] write_reg,   // vd
    input [127:0] write_data,
    output reg [127:0] read_data1,
    output reg [127:0] read_data2,
    output wire [127:0] v0_data   // fixed read of v0, for masking (v0.t)
);
    reg [127:0] registers [0:31];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 128'b0;
        end
        else if (vreg_write) begin
            registers[write_reg] <= write_data;
        end
    end

    always @(*) begin
        read_data1 = registers[read_reg1];
        read_data2 = registers[read_reg2];
    end

    assign v0_data = registers[0];
endmodule
