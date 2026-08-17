`timescale 1ns / 1ps
// Blackbox stub for synthesis-only area estimation -- see
// instruction_fetch.v's synth copy for the full rationale. Same reasoning
// applies here: data memory is a real memory macro, and even though this
// module has a write port (so it's less exposed to full constant-folding
// than the write-port-less instruction ROM), an unconstrained blackbox
// output is still the correct way to keep the surrounding load/store,
// AMO, and vector-memory control logic honestly represented.
(* blackbox *)
module data_memory #(
    parameter DMEM_FILE  = "data.mem",
    parameter DMEM_WORDS = 4096
)(
    input clk,
    input mem_read,
    input mem_write,
    input [2:0] func3,
    input [63:0] mem_addr,
    input [63:0] write_data,
    output wire [63:0] read_data,

    input vmem_read,
    input vmem_write,
    input [63:0] vmem_addr,
    input [127:0] vmem_write_data,
    output wire [127:0] vmem_read_data
);
endmodule
