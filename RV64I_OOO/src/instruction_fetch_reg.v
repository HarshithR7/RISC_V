`timescale 1ns / 1ps
// FPGA bring-up (real hardware timing): a thin wrapper around the
// sibling single-cycle core's instruction_fetch.v (left completely
// unmodified -- it's also used, as-is, by RV64I's own single-cycle
// datapath, which genuinely needs a same-cycle combinational read) that
// adds exactly one register stage on the output.
//
// Real Xilinx Block RAM has no combinational-read mode at the sizes this
// design's instruction memory needs (thousands of words per thread) --
// distributed RAM (LUTRAM), the one primitive that CAN do a same-cycle
// read, doesn't scale to that capacity without burning a large fraction
// of a mid-size FPGA's fabric on storage alone. So a real hardware build
// needs a genuinely registered (1-cycle-latency) fetch; this module
// models exactly that latency in a way that's still driven by the same,
// already-verified instruction_fetch.v internals -- functional fetch
// behavior (halfword pairing, $readmemh content) is untouched, only WHEN
// the result becomes visible changes.
//
// riscv64_ooo_proc.v is the only caller, and pairs every use of this
// module with its own t0_pc_latched/t1_pc_latched register (see that
// file's header addendum) so the PC a downstream consumer sees always
// lines up with the instruction bits that same consumer sees, one cycle
// after the address was actually presented here.
module instruction_fetch_reg #(
    parameter IMEM_FILE  = "instructions.mem",
    parameter IMEM_WORDS = 8192
)(
    input clk,
    input [63:0] pc,
    output reg [31:0] instruction
);
    wire [31:0] instruction_comb;
    instruction_fetch #(.IMEM_FILE(IMEM_FILE), .IMEM_WORDS(IMEM_WORDS)) inner (
        .clk(clk), .pc(pc), .instruction(instruction_comb)
    );

    always @(posedge clk) instruction <= instruction_comb;
endmodule
