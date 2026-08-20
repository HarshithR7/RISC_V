`timescale 1ns / 1ps
// 2-core system: two independent riscv64_ooo_proc instances (each already
// 2-thread SMT, so 4 hardware threads total), each with its own private
// L1 data cache, sharing one l2_cache.v as the real coherence point --
// see l1_cache.v/l2_cache.v's own headers for the MESI protocol this
// wiring drives, and riscv64_ooo_proc.v's header for why a single core no
// longer owns a backing memory directly.
//
// This is the genuine, full-pipeline version of what tb_cache_mesi.v
// already verified in isolation (two l1_cache.v instances + one
// l2_cache.v, driven directly): here the same three modules are driven
// by two real, independently-executing OoO cores instead of a scripted
// test sequence, so a store committed on core 0 is only ever visible to
// core 1 by actually going through a real coherency transaction, not by
// construction.
module dual_core_riscv64_ooo #(
    parameter C0_IMEM_FILE0 = "core0_t0.mem",
    parameter C0_IMEM_FILE1 = "core0_t1.mem",
    parameter C1_IMEM_FILE0 = "core1_t0.mem",
    parameter C1_IMEM_FILE1 = "core1_t1.mem",
    parameter DMEM_FILE = "data.mem",
    parameter IMEM_WORDS = 8192,
    parameter DMEM_WORDS = 4096,
    parameter ROB_DEPTH = 8,
    parameter ALU_RS_DEPTH = 4,
    parameter MUL_RS_DEPTH = 2,
    parameter LSQ_DEPTH = 4,
    parameter ENABLE_DUAL_ISSUE = 1,
    parameter LANES = 4,
    parameter VEC_RS_DEPTH = 2,
    parameter L1_LINES = 16,
    parameter L1_LINE_BYTES = 32,
    parameter SBUF_DEPTH = 4,
    parameter L2_LINES = 64,
    // Phase 12 (FPGA bring-up): 0 (default, every existing testbench)
    // keeps every instruction/backing memory on its $readmemh path; 1
    // selects the AXI-writable variants everywhere -- see
    // instruction_fetch_reg.v's and l2_cache.v's own USE_AXI_MEM.
    parameter USE_AXI_MEM = 0
)(
    input clk,
    input reset,
    // Naming: c<core>t<thread> -- e.g. c0t1_pc_out is core 0's thread 1.
    output wire [63:0] c0t0_pc_out, output wire [63:0] c0t1_pc_out,
    output wire c0t0_ecall_halt, output wire c0t1_ecall_halt,
    output wire [63:0] c1t0_pc_out, output wire [63:0] c1t1_pc_out,
    output wire c1t0_ecall_halt, output wire c1t1_ecall_halt,

    // Phase 12 (FPGA bring-up): only meaningful when USE_AXI_MEM=1. One
    // write port per thread's instruction memory (mirrored internally to
    // both of that thread's fetch instances -- see riscv64_ooo_proc.v's
    // own header), plus one for the single shared L2 backing memory.
    input c0t0_imem_axi_wr_en, input [$clog2(IMEM_WORDS)-1:0] c0t0_imem_axi_wr_addr, input [15:0] c0t0_imem_axi_wr_data,
    input c0t1_imem_axi_wr_en, input [$clog2(IMEM_WORDS)-1:0] c0t1_imem_axi_wr_addr, input [15:0] c0t1_imem_axi_wr_data,
    input c1t0_imem_axi_wr_en, input [$clog2(IMEM_WORDS)-1:0] c1t0_imem_axi_wr_addr, input [15:0] c1t0_imem_axi_wr_data,
    input c1t1_imem_axi_wr_en, input [$clog2(IMEM_WORDS)-1:0] c1t1_imem_axi_wr_addr, input [15:0] c1t1_imem_axi_wr_data,
    input dmem_axi_wr_en, input [$clog2(DMEM_WORDS)-1:0] dmem_axi_wr_addr, input [63:0] dmem_axi_wr_data
);
    localparam VBITS = L1_LINE_BYTES * 8;

    wire c0_l2_req_valid; wire [1:0] c0_l2_req_type; wire [63:0] c0_l2_req_addr;
    wire [VBITS-1:0] c0_l2_req_wb_data;
    wire c0_l2_resp_valid; wire [VBITS-1:0] c0_l2_resp_data; wire c0_l2_resp_exclusive;
    wire c0_snoop_req_valid; wire [1:0] c0_snoop_req_type; wire [63:0] c0_snoop_req_addr;
    wire c0_snoop_resp_hit, c0_snoop_resp_dirty; wire [VBITS-1:0] c0_snoop_resp_data;

    riscv64_ooo_proc #(
        .IMEM_FILE0(C0_IMEM_FILE0), .IMEM_FILE1(C0_IMEM_FILE1), .IMEM_WORDS(IMEM_WORDS),
        .ROB_DEPTH(ROB_DEPTH), .ALU_RS_DEPTH(ALU_RS_DEPTH), .MUL_RS_DEPTH(MUL_RS_DEPTH),
        .LSQ_DEPTH(LSQ_DEPTH), .ENABLE_DUAL_ISSUE(ENABLE_DUAL_ISSUE), .LANES(LANES),
        .VEC_RS_DEPTH(VEC_RS_DEPTH), .L1_LINES(L1_LINES), .L1_LINE_BYTES(L1_LINE_BYTES),
        .SBUF_DEPTH(SBUF_DEPTH), .USE_AXI_MEM(USE_AXI_MEM)
    ) core0 (
        .clk(clk), .reset(reset),
        .pc_out0(c0t0_pc_out), .pc_out1(c0t1_pc_out),
        .ecall_halt0(c0t0_ecall_halt), .ecall_halt1(c0t1_ecall_halt),
        .l2_req_valid(c0_l2_req_valid), .l2_req_type(c0_l2_req_type), .l2_req_addr(c0_l2_req_addr),
        .l2_req_wb_data(c0_l2_req_wb_data),
        .l2_resp_valid(c0_l2_resp_valid), .l2_resp_data(c0_l2_resp_data), .l2_resp_exclusive(c0_l2_resp_exclusive),
        .snoop_req_valid(c0_snoop_req_valid), .snoop_req_type(c0_snoop_req_type), .snoop_req_addr(c0_snoop_req_addr),
        .snoop_resp_hit(c0_snoop_resp_hit), .snoop_resp_dirty(c0_snoop_resp_dirty), .snoop_resp_data(c0_snoop_resp_data),
        .ecc_rf_sbe_fault(), .ecc_rf_dbe_fault(), .ecc_l1_sbe_fault(), .ecc_l1_dbe_fault(),
        .ecc_rob_sbe_fault(), .ecc_rob_dbe_fault(),
        .t0_imem_axi_wr_en(c0t0_imem_axi_wr_en), .t0_imem_axi_wr_addr(c0t0_imem_axi_wr_addr), .t0_imem_axi_wr_data(c0t0_imem_axi_wr_data),
        .t1_imem_axi_wr_en(c0t1_imem_axi_wr_en), .t1_imem_axi_wr_addr(c0t1_imem_axi_wr_addr), .t1_imem_axi_wr_data(c0t1_imem_axi_wr_data)
    );

    wire c1_l2_req_valid; wire [1:0] c1_l2_req_type; wire [63:0] c1_l2_req_addr;
    wire [VBITS-1:0] c1_l2_req_wb_data;
    wire c1_l2_resp_valid; wire [VBITS-1:0] c1_l2_resp_data; wire c1_l2_resp_exclusive;
    wire c1_snoop_req_valid; wire [1:0] c1_snoop_req_type; wire [63:0] c1_snoop_req_addr;
    wire c1_snoop_resp_hit, c1_snoop_resp_dirty; wire [VBITS-1:0] c1_snoop_resp_data;

    riscv64_ooo_proc #(
        .IMEM_FILE0(C1_IMEM_FILE0), .IMEM_FILE1(C1_IMEM_FILE1), .IMEM_WORDS(IMEM_WORDS),
        .ROB_DEPTH(ROB_DEPTH), .ALU_RS_DEPTH(ALU_RS_DEPTH), .MUL_RS_DEPTH(MUL_RS_DEPTH),
        .LSQ_DEPTH(LSQ_DEPTH), .ENABLE_DUAL_ISSUE(ENABLE_DUAL_ISSUE), .LANES(LANES),
        .VEC_RS_DEPTH(VEC_RS_DEPTH), .L1_LINES(L1_LINES), .L1_LINE_BYTES(L1_LINE_BYTES),
        .SBUF_DEPTH(SBUF_DEPTH), .USE_AXI_MEM(USE_AXI_MEM)
    ) core1 (
        .clk(clk), .reset(reset),
        .pc_out0(c1t0_pc_out), .pc_out1(c1t1_pc_out),
        .ecall_halt0(c1t0_ecall_halt), .ecall_halt1(c1t1_ecall_halt),
        .l2_req_valid(c1_l2_req_valid), .l2_req_type(c1_l2_req_type), .l2_req_addr(c1_l2_req_addr),
        .l2_req_wb_data(c1_l2_req_wb_data),
        .l2_resp_valid(c1_l2_resp_valid), .l2_resp_data(c1_l2_resp_data), .l2_resp_exclusive(c1_l2_resp_exclusive),
        .snoop_req_valid(c1_snoop_req_valid), .snoop_req_type(c1_snoop_req_type), .snoop_req_addr(c1_snoop_req_addr),
        .snoop_resp_hit(c1_snoop_resp_hit), .snoop_resp_dirty(c1_snoop_resp_dirty), .snoop_resp_data(c1_snoop_resp_data),
        .ecc_rf_sbe_fault(), .ecc_rf_dbe_fault(), .ecc_l1_sbe_fault(), .ecc_l1_dbe_fault(),
        .ecc_rob_sbe_fault(), .ecc_rob_dbe_fault(),
        .t0_imem_axi_wr_en(c1t0_imem_axi_wr_en), .t0_imem_axi_wr_addr(c1t0_imem_axi_wr_addr), .t0_imem_axi_wr_data(c1t0_imem_axi_wr_data),
        .t1_imem_axi_wr_en(c1t1_imem_axi_wr_en), .t1_imem_axi_wr_addr(c1t1_imem_axi_wr_addr), .t1_imem_axi_wr_data(c1t1_imem_axi_wr_data)
    );

    l2_cache #(.L2_LINES(L2_LINES), .LINE_BYTES(L1_LINE_BYTES), .ADDR_BITS(64),
               .DMEM_FILE(DMEM_FILE), .DMEM_WORDS(DMEM_WORDS), .USE_AXI_MEM(USE_AXI_MEM)) l2 (
        .clk(clk), .reset(reset),
        .c0_req_valid(c0_l2_req_valid), .c0_req_type(c0_l2_req_type), .c0_req_addr(c0_l2_req_addr),
        .c0_req_wb_data(c0_l2_req_wb_data),
        .c0_resp_valid(c0_l2_resp_valid), .c0_resp_data(c0_l2_resp_data), .c0_resp_exclusive(c0_l2_resp_exclusive),
        .c1_req_valid(c1_l2_req_valid), .c1_req_type(c1_l2_req_type), .c1_req_addr(c1_l2_req_addr),
        .c1_req_wb_data(c1_l2_req_wb_data),
        .c1_resp_valid(c1_l2_resp_valid), .c1_resp_data(c1_l2_resp_data), .c1_resp_exclusive(c1_l2_resp_exclusive),
        .snoop0_req_valid(c0_snoop_req_valid), .snoop0_req_type(c0_snoop_req_type), .snoop0_req_addr(c0_snoop_req_addr),
        .snoop0_resp_hit(c0_snoop_resp_hit), .snoop0_resp_dirty(c0_snoop_resp_dirty), .snoop0_resp_data(c0_snoop_resp_data),
        .snoop1_req_valid(c1_snoop_req_valid), .snoop1_req_type(c1_snoop_req_type), .snoop1_req_addr(c1_snoop_req_addr),
        .snoop1_resp_hit(c1_snoop_resp_hit), .snoop1_resp_dirty(c1_snoop_resp_dirty), .snoop1_resp_data(c1_snoop_resp_data),
        .ecc_l2_sbe_fault(), .ecc_l2_dbe_fault(),
        .axi_wr_en(dmem_axi_wr_en), .axi_wr_addr(dmem_axi_wr_addr), .axi_wr_data(dmem_axi_wr_data)
    );
endmodule
