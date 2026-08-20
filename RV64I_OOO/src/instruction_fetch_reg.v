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
//
// Phase 12 (FPGA bring-up: AXI-loadable instruction memory): USE_AXI_MEM
// (default 0, every existing testbench) selects, via the same
// generate-if pattern l2_cache.v's USE_AXI_MEM already established,
// between this default $readmemh-loaded path and
// instruction_fetch_axi.v (registered read + a real AXI write port, no
// $readmemh). The axi_wr_* ports are only ever wired to anything inside
// the USE_AXI_MEM=1 branch, so -- like l2_cache.v's own axi_wr_* -- they
// need no tie-off updates in any pre-existing testbench: there's no
// logic path for an unconnected input to reach when unused.
module instruction_fetch_reg #(
    parameter IMEM_FILE  = "instructions.mem",
    parameter IMEM_WORDS = 8192,
    parameter USE_AXI_MEM = 0
)(
    input clk,
    input [63:0] pc,
    output wire [31:0] instruction,

    input axi_wr_en,
    input [$clog2(IMEM_WORDS)-1:0] axi_wr_addr,
    input [15:0] axi_wr_data
);
    generate
        if (USE_AXI_MEM) begin : axi_fetch
            instruction_fetch_axi #(.IMEM_WORDS(IMEM_WORDS)) inner (
                .clk(clk), .pc(pc), .instruction(instruction),
                .axi_wr_en(axi_wr_en), .axi_wr_addr(axi_wr_addr), .axi_wr_data(axi_wr_data)
            );
        end else begin : sim_fetch
            wire [31:0] instruction_comb;
            reg [31:0] instruction_r;
            instruction_fetch #(.IMEM_FILE(IMEM_FILE), .IMEM_WORDS(IMEM_WORDS)) inner (
                .clk(clk), .pc(pc), .instruction(instruction_comb)
            );
            always @(posedge clk) instruction_r <= instruction_comb;
            assign instruction = instruction_r;
        end
    endgenerate
endmodule
