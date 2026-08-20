`timescale 1ns / 1ps
// FPGA bring-up (PS-side program loading): a registered-read, AXI-
// writable instruction memory -- the AXI-loadable counterpart to
// instruction_fetch_reg.v's simulation-only ($readmemh) path, selected
// by that module's own USE_AXI_MEM parameter. Not a wrapper around
// instruction_fetch.v (unlike instruction_fetch_reg.v's default path):
// that module's read is combinational and has no write port at all, so
// a clean, registered, AXI-writable reimplementation is simplest as its
// own small file -- same reasoning as data_memory_axi.v vs data_memory.v.
//
// Same halfword-pairing semantics as instruction_fetch.v (RV64C mixes
// 16-bit and 32-bit instructions, so a fetch always reads two
// consecutive halfwords), just with the read registered by one cycle
// and no $readmemh -- content arrives only through the AXI write port.
module instruction_fetch_axi #(
    parameter IMEM_WORDS = 8192   // halfwords, not 32-bit words
)(
    input clk,
    input [63:0] pc,
    output reg [31:0] instruction,

    input axi_wr_en,
    input [$clog2(IMEM_WORDS)-1:0] axi_wr_addr,
    input [15:0] axi_wr_data
);
    reg [15:0] memory [0:IMEM_WORDS-1];

    wire [$clog2(IMEM_WORDS)-1:0] hw_index = pc[$clog2(IMEM_WORDS):1];
    wire [31:0] instruction_comb = {memory[hw_index + 1'b1], memory[hw_index]};

    always @(posedge clk) begin
        instruction <= instruction_comb;
        if (axi_wr_en) memory[axi_wr_addr] <= axi_wr_data;
    end
endmodule
