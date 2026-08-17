`timescale 1ns / 1ps
// 32 x VLEN-bit vector register file (v0-v31). SEW=32 always (see
// RV64I/README.md for the scope decision), so LANES = VLEN/32 is the
// number of 32-bit elements per register and VLMAX (since LMUL=1 always
// here too) -- LANES is the configurable-vector-width knob threaded down
// from riscv64_processor's own LANES parameter; VLEN is derived, not set
// directly, so it's never possible to end up with a VLEN that isn't an
// exact multiple of SEW=32. v0.t masking reads v0_data[LANES-1:0] (see
// riscv64_proc.v), so v0 keeps working as the mask register at any width.
module vector_register_file #(
    parameter LANES = 4,
    parameter VLEN = LANES * 32
)(
    input clk,
    input reset,
    input vreg_write,
    input [4:0] read_reg1,   // vs2 (or vs3 for vse32.v -- see riscv64_proc.v)
    input [4:0] read_reg2,   // vs1
    input [4:0] write_reg,   // vd
    input [VLEN-1:0] write_data,
    output reg [VLEN-1:0] read_data1,
    output reg [VLEN-1:0] read_data2,
    output wire [VLEN-1:0] v0_data   // fixed read of v0, for masking (v0.t)
);
    reg [VLEN-1:0] registers [0:31];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= {VLEN{1'b0}};
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
