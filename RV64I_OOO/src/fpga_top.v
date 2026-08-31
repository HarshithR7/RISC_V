`timescale 1ns / 1ps
// FPGA bring-up (PYNQ-Z1): the actual top-level IP block for the Vivado
// design -- everything from here down is this project's own RTL
// (`dual_core_riscv64_ooo.v` with USE_AXI_MEM=1, `axi_ooo_ctrl.v`); only
// this module's own S_AXI_* port list and clk/resetn are meant to be
// wired into a Vivado Block Design, to the Zynq7 Processing System's
// M_AXI_GP0 port via an AXI interconnect, at whatever base address the
// address editor assigns this peripheral.
//
// Reset polarity note: axi_ooo_ctrl.v's own AXI-side reset (S_AXI_ARESETN,
// active-low, the AXI4 convention) is kept separate from
// dual_core_riscv64_ooo.v's core_reset (active-high, this project's own
// convention throughout) -- they are NOT the same signal. S_AXI_ARESETN
// resets the AXI-lite peripheral's own registers (so a bus reset doesn't
// silently lose the loaded program by also resetting the cores); the
// cores' own reset is entirely software-controlled through the CONTROL
// register, defaulting to held-in-reset at S_AXI_ARESETN release (see
// axi_ooo_ctrl.v's own header) until the PS explicitly starts them once
// loading is complete.
module fpga_top #(
    parameter C_S_AXI_ADDR_WIDTH = 20,
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter IMEM_WORDS = 8192,
    parameter DMEM_WORDS = 4096,
    // Phase 14: see riscv64_ooo_proc.v's own header for why these doubled.
    parameter ROB_DEPTH = 16,
    parameter ALU_RS_DEPTH = 8,
    parameter MUL_RS_DEPTH = 4,
    parameter LSQ_DEPTH = 8,
    parameter ENABLE_DUAL_ISSUE = 1,
    parameter LANES = 4,
    parameter VEC_RS_DEPTH = 2,
    parameter L1_LINES = 16,
    parameter L1_LINE_BYTES = 32,
    parameter SBUF_DEPTH = 4,
    parameter L2_LINES = 64
)(
    input wire S_AXI_ACLK,
    input wire S_AXI_ARESETN,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_AWADDR,
    input  wire S_AXI_AWVALID,
    output wire S_AXI_AWREADY,

    input  wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  wire S_AXI_WVALID,
    output wire S_AXI_WREADY,

    output wire [1:0] S_AXI_BRESP,
    output wire S_AXI_BVALID,
    input  wire S_AXI_BREADY,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_ARADDR,
    input  wire S_AXI_ARVALID,
    output wire S_AXI_ARREADY,

    output wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_RDATA,
    output wire [1:0] S_AXI_RRESP,
    output wire S_AXI_RVALID,
    input  wire S_AXI_RREADY
);
    wire core_reset;

    wire [63:0] c0t0_pc, c0t1_pc, c1t0_pc, c1t1_pc;
    wire c0t0_halt, c0t1_halt, c1t0_halt, c1t1_halt;

    wire c0t0_imem_wr_en; wire [$clog2(IMEM_WORDS)-1:0] c0t0_imem_wr_addr; wire [15:0] c0t0_imem_wr_data;
    wire c0t1_imem_wr_en; wire [$clog2(IMEM_WORDS)-1:0] c0t1_imem_wr_addr; wire [15:0] c0t1_imem_wr_data;
    wire c1t0_imem_wr_en; wire [$clog2(IMEM_WORDS)-1:0] c1t0_imem_wr_addr; wire [15:0] c1t0_imem_wr_data;
    wire c1t1_imem_wr_en; wire [$clog2(IMEM_WORDS)-1:0] c1t1_imem_wr_addr; wire [15:0] c1t1_imem_wr_data;
    wire dmem_wr_en; wire [$clog2(DMEM_WORDS)-1:0] dmem_wr_addr; wire [63:0] dmem_wr_data;

    axi_ooo_ctrl #(
        .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH), .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
        .IMEM_WORDS(IMEM_WORDS), .DMEM_WORDS(DMEM_WORDS)
    ) ctrl (
        .S_AXI_ACLK(S_AXI_ACLK), .S_AXI_ARESETN(S_AXI_ARESETN),
        .S_AXI_AWADDR(S_AXI_AWADDR), .S_AXI_AWVALID(S_AXI_AWVALID), .S_AXI_AWREADY(S_AXI_AWREADY),
        .S_AXI_WDATA(S_AXI_WDATA), .S_AXI_WSTRB(S_AXI_WSTRB), .S_AXI_WVALID(S_AXI_WVALID), .S_AXI_WREADY(S_AXI_WREADY),
        .S_AXI_BRESP(S_AXI_BRESP), .S_AXI_BVALID(S_AXI_BVALID), .S_AXI_BREADY(S_AXI_BREADY),
        .S_AXI_ARADDR(S_AXI_ARADDR), .S_AXI_ARVALID(S_AXI_ARVALID), .S_AXI_ARREADY(S_AXI_ARREADY),
        .S_AXI_RDATA(S_AXI_RDATA), .S_AXI_RRESP(S_AXI_RRESP), .S_AXI_RVALID(S_AXI_RVALID), .S_AXI_RREADY(S_AXI_RREADY),
        .core_reset(core_reset),
        .c0t0_halt(c0t0_halt), .c0t1_halt(c0t1_halt), .c1t0_halt(c1t0_halt), .c1t1_halt(c1t1_halt),
        .c0t0_pc(c0t0_pc), .c0t1_pc(c0t1_pc), .c1t0_pc(c1t0_pc), .c1t1_pc(c1t1_pc),
        .c0t0_imem_wr_en(c0t0_imem_wr_en), .c0t0_imem_wr_addr(c0t0_imem_wr_addr), .c0t0_imem_wr_data(c0t0_imem_wr_data),
        .c0t1_imem_wr_en(c0t1_imem_wr_en), .c0t1_imem_wr_addr(c0t1_imem_wr_addr), .c0t1_imem_wr_data(c0t1_imem_wr_data),
        .c1t0_imem_wr_en(c1t0_imem_wr_en), .c1t0_imem_wr_addr(c1t0_imem_wr_addr), .c1t0_imem_wr_data(c1t0_imem_wr_data),
        .c1t1_imem_wr_en(c1t1_imem_wr_en), .c1t1_imem_wr_addr(c1t1_imem_wr_addr), .c1t1_imem_wr_data(c1t1_imem_wr_data),
        .dmem_wr_en(dmem_wr_en), .dmem_wr_addr(dmem_wr_addr), .dmem_wr_data(dmem_wr_data)
    );

    // core_reset is active-high (this project's convention); S_AXI_ACLK
    // is this peripheral's own clock, reused directly as the cores'
    // clock too -- a single-clock-domain design, no CDC needed anywhere
    // in this system.
    dual_core_riscv64_ooo #(
        .IMEM_WORDS(IMEM_WORDS), .DMEM_WORDS(DMEM_WORDS),
        .ROB_DEPTH(ROB_DEPTH), .ALU_RS_DEPTH(ALU_RS_DEPTH), .MUL_RS_DEPTH(MUL_RS_DEPTH),
        .LSQ_DEPTH(LSQ_DEPTH), .ENABLE_DUAL_ISSUE(ENABLE_DUAL_ISSUE), .LANES(LANES),
        .VEC_RS_DEPTH(VEC_RS_DEPTH), .L1_LINES(L1_LINES), .L1_LINE_BYTES(L1_LINE_BYTES),
        .SBUF_DEPTH(SBUF_DEPTH), .L2_LINES(L2_LINES), .USE_AXI_MEM(1)
    ) cores (
        .clk(S_AXI_ACLK), .reset(core_reset),
        .c0t0_pc_out(c0t0_pc), .c0t1_pc_out(c0t1_pc),
        .c0t0_ecall_halt(c0t0_halt), .c0t1_ecall_halt(c0t1_halt),
        .c1t0_pc_out(c1t0_pc), .c1t1_pc_out(c1t1_pc),
        .c1t0_ecall_halt(c1t0_halt), .c1t1_ecall_halt(c1t1_halt),
        .c0t0_imem_axi_wr_en(c0t0_imem_wr_en), .c0t0_imem_axi_wr_addr(c0t0_imem_wr_addr), .c0t0_imem_axi_wr_data(c0t0_imem_wr_data),
        .c0t1_imem_axi_wr_en(c0t1_imem_wr_en), .c0t1_imem_axi_wr_addr(c0t1_imem_wr_addr), .c0t1_imem_axi_wr_data(c0t1_imem_wr_data),
        .c1t0_imem_axi_wr_en(c1t0_imem_wr_en), .c1t0_imem_axi_wr_addr(c1t0_imem_wr_addr), .c1t0_imem_axi_wr_data(c1t0_imem_wr_data),
        .c1t1_imem_axi_wr_en(c1t1_imem_wr_en), .c1t1_imem_axi_wr_addr(c1t1_imem_wr_addr), .c1t1_imem_axi_wr_data(c1t1_imem_wr_data),
        .dmem_axi_wr_en(dmem_wr_en), .dmem_axi_wr_addr(dmem_wr_addr), .dmem_axi_wr_data(dmem_wr_data)
    );
endmodule
