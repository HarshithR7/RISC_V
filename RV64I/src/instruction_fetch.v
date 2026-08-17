`timescale 1ns / 1ps
// Halfword-addressable instruction memory: RV64C mixes 16-bit and 32-bit
// instructions in the same stream, so instructions are only guaranteed to
// be 2-byte aligned, not 4-byte aligned. Always fetch two consecutive
// halfwords ({next, current}) so a 32-bit instruction that straddles a
// non-4-byte-aligned boundary is still read correctly in one cycle;
// compressed_decoder uses only the low 16 bits when the current
// instruction turns out to be compressed.
//
// $readmemh loads one 16-bit token per line (RISC-V is little-endian: a
// 32-bit instruction's low halfword is stored at the lower address).
module instruction_fetch #(
    parameter IMEM_FILE  = "instructions.mem",
    parameter IMEM_WORDS = 8192   // halfwords, not 32-bit words
)(
    input clk,
    input [63:0] pc,
    output wire [31:0] instruction  // {halfword at pc+2, halfword at pc}
);
    reg [15:0] memory [0:IMEM_WORDS-1];

    initial begin
        $readmemh(IMEM_FILE, memory);
        $display("Instruction memory loaded successfully from %s.", IMEM_FILE);
    end

    wire [$clog2(IMEM_WORDS)-1:0] hw_index = pc[$clog2(IMEM_WORDS):1];
    assign instruction = {memory[hw_index + 1'b1], memory[hw_index]};
endmodule
