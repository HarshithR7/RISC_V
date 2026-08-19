`timescale 1ns / 1ps
// Single-core wrapper: one riscv64_ooo_proc (which always has its own
// private L1 -- see its own header) paired with one l2_cache.v configured
// for exactly one attached core, giving every single-core testbench the
// same simple, self-contained "just instantiate this one module" surface
// Phase 1-7 had, now that riscv64_ooo_proc.v itself no longer owns a
// backing data_memory.v directly (see its header -- Phase 8 moved that
// behind the shared, external L2).
//
// Core 1's side of l2_cache.v is simply tied off: c1_req_valid held low
// (core 1 never requests, so l2_cache.v's presence1 directory bits never
// get set for anything, and it correctly never even attempts to snoop a
// nonexistent core 1), and snoop1_resp_hit/dirty tied low as
// belt-and-suspenders (not load-bearing, since l2_cache.v structurally
// never asks in the first place -- see the reasoning above).
module riscv64_ooo_proc_solo #(
    parameter IMEM_FILE0 = "instructions0.mem",
    parameter IMEM_FILE1 = "instructions1.mem",
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
    parameter L2_LINES = 64
)(
    input clk,
    input reset,
    output wire [63:0] pc_out0,
    output wire [63:0] pc_out1,
    output wire ecall_halt0,
    output wire ecall_halt1
);
    localparam VBITS = L1_LINE_BYTES * 8;

    wire l2_req_valid; wire [1:0] l2_req_type; wire [63:0] l2_req_addr;
    wire [VBITS-1:0] l2_req_wb_data;
    wire l2_resp_valid; wire [VBITS-1:0] l2_resp_data; wire l2_resp_exclusive;
    wire snoop_req_valid; wire [1:0] snoop_req_type; wire [63:0] snoop_req_addr;
    wire snoop_resp_hit, snoop_resp_dirty; wire [VBITS-1:0] snoop_resp_data;

    riscv64_ooo_proc #(
        .IMEM_FILE0(IMEM_FILE0), .IMEM_FILE1(IMEM_FILE1), .IMEM_WORDS(IMEM_WORDS),
        .ROB_DEPTH(ROB_DEPTH), .ALU_RS_DEPTH(ALU_RS_DEPTH), .MUL_RS_DEPTH(MUL_RS_DEPTH),
        .LSQ_DEPTH(LSQ_DEPTH), .ENABLE_DUAL_ISSUE(ENABLE_DUAL_ISSUE), .LANES(LANES),
        .VEC_RS_DEPTH(VEC_RS_DEPTH), .L1_LINES(L1_LINES), .L1_LINE_BYTES(L1_LINE_BYTES),
        .SBUF_DEPTH(SBUF_DEPTH)
    ) core (
        .clk(clk), .reset(reset),
        .pc_out0(pc_out0), .pc_out1(pc_out1),
        .ecall_halt0(ecall_halt0), .ecall_halt1(ecall_halt1),
        .l2_req_valid(l2_req_valid), .l2_req_type(l2_req_type), .l2_req_addr(l2_req_addr),
        .l2_req_wb_data(l2_req_wb_data),
        .l2_resp_valid(l2_resp_valid), .l2_resp_data(l2_resp_data), .l2_resp_exclusive(l2_resp_exclusive),
        .snoop_req_valid(snoop_req_valid), .snoop_req_type(snoop_req_type), .snoop_req_addr(snoop_req_addr),
        .snoop_resp_hit(snoop_resp_hit), .snoop_resp_dirty(snoop_resp_dirty), .snoop_resp_data(snoop_resp_data)
    );

    l2_cache #(.L2_LINES(L2_LINES), .LINE_BYTES(L1_LINE_BYTES), .ADDR_BITS(64),
               .DMEM_FILE(DMEM_FILE), .DMEM_WORDS(DMEM_WORDS)) l2 (
        .clk(clk), .reset(reset),
        .c0_req_valid(l2_req_valid), .c0_req_type(l2_req_type), .c0_req_addr(l2_req_addr),
        .c0_req_wb_data(l2_req_wb_data),
        .c0_resp_valid(l2_resp_valid), .c0_resp_data(l2_resp_data), .c0_resp_exclusive(l2_resp_exclusive),
        .c1_req_valid(1'b0), .c1_req_type(2'b0), .c1_req_addr(64'b0), .c1_req_wb_data({VBITS{1'b0}}),
        .c1_resp_valid(), .c1_resp_data(), .c1_resp_exclusive(),
        .snoop0_req_valid(snoop_req_valid), .snoop0_req_type(snoop_req_type), .snoop0_req_addr(snoop_req_addr),
        .snoop0_resp_hit(snoop_resp_hit), .snoop0_resp_dirty(snoop_resp_dirty), .snoop0_resp_data(snoop_resp_data),
        .snoop1_req_valid(), .snoop1_req_type(), .snoop1_req_addr(),
        .snoop1_resp_hit(1'b0), .snoop1_resp_dirty(1'b0), .snoop1_resp_data({VBITS{1'b0}})
    );
endmodule
