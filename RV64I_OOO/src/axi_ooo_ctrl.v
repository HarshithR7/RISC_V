`timescale 1ns / 1ps
// FPGA bring-up (PYNQ-Z1): a complete, hand-written AXI4-Lite slave --
// not a Vivado IP-catalog peripheral, so (like every other module in
// this project) it can be fully verified in simulation with a real
// bus-functional-model testbench before ever touching Vivado. Owns:
//   - core_reset: the PS holds the dual-core system in reset while
//     loading programs through the memory windows below, then clears it
//     to start execution.
//   - Status readback: each thread's ecall_halt (LATCHED -- see its own
//     comment below for why: the core's own ecall_halt output is a bare
//     one-cycle commit pulse, not sticky, and a software poll loop over
//     AXI reads could otherwise miss it entirely) and 64-bit PC (split
//     across two 32-bit registers -- AXI4-Lite's data width here).
//   - Five write-only memory windows, one per Phase 12 AXI-loadable
//     memory (dual_core_riscv64_ooo.v's 4 imem write ports + 1 shared
//     dmem write port): each 32-bit AXI-Lite write maps to exactly one
//     write pulse on the target memory's own native port, with the
//     16-bit imem case needing no staging (a full halfword fits in one
//     32-bit transfer) and the 64-bit dmem case needing a standard
//     write-low-then-write-high-commits protocol (the low half must be
//     written first for a given word index -- the same convention any
//     32-bit-bus access to a wider register uses).
//
// Write-channel handshake follows the standard, widely-used AXI4-Lite
// slave template (require AWVALID and WVALID together before asserting
// either READY) -- a fully spec-compliant simplification, not a
// shortcut: a slave is always allowed to defer accepting either channel
// until it's ready to accept both.
//
// ---- Address map (byte offsets from this peripheral's own base) ------
//   0x00000  CONTROL      (RW) bit0 = core_reset
//   0x00004  STATUS       (RO) bit0..3 = c0t0/c0t1/c1t0/c1t1 ecall_halt
//   0x00008  C0T0_PC_LO   (RO)
//   0x0000C  C0T0_PC_HI   (RO)
//   0x00010  C0T1_PC_LO   (RO)
//   0x00014  C0T1_PC_HI   (RO)
//   0x00018  C1T0_PC_LO   (RO)
//   0x0001C  C1T0_PC_HI   (RO)
//   0x00020  C1T1_PC_LO   (RO)
//   0x00024  C1T1_PC_HI   (RO)
//   0x10000..0x1FFFF  C0T0_IMEM (WO) -- byte offset>>2 = halfword index
//   0x20000..0x2FFFF  C0T1_IMEM (WO)
//   0x30000..0x3FFFF  C1T0_IMEM (WO)
//   0x40000..0x4FFFF  C1T1_IMEM (WO)
//   0x50000..0x5FFFF  DMEM      (WO) -- word_index = (offset>>3); a
//     write to word_index*8+0 latches the low 32 bits; a write to
//     word_index*8+4 supplies the high 32 bits AND commits the 64-bit
//     write using the latched low value -- write low, then high, per
//     word, in that order.
module axi_ooo_ctrl #(
    parameter C_S_AXI_ADDR_WIDTH = 20,
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter IMEM_WORDS = 8192,
    parameter DMEM_WORDS = 4096
)(
    input wire S_AXI_ACLK,
    input wire S_AXI_ARESETN,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_AWADDR,
    input  wire S_AXI_AWVALID,
    output reg  S_AXI_AWREADY,

    input  wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  wire S_AXI_WVALID,
    output reg  S_AXI_WREADY,

    output reg  [1:0] S_AXI_BRESP,
    output reg  S_AXI_BVALID,
    input  wire S_AXI_BREADY,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_ARADDR,
    input  wire S_AXI_ARVALID,
    output reg  S_AXI_ARREADY,

    output reg  [C_S_AXI_DATA_WIDTH-1:0] S_AXI_RDATA,
    output reg  [1:0] S_AXI_RRESP,
    output reg  S_AXI_RVALID,
    input  wire S_AXI_RREADY,

    // ---- Core-facing side ----------------------------------------------
    output reg core_reset,

    input wire c0t0_halt, input wire c0t1_halt, input wire c1t0_halt, input wire c1t1_halt,
    input wire [63:0] c0t0_pc, input wire [63:0] c0t1_pc, input wire [63:0] c1t0_pc, input wire [63:0] c1t1_pc,

    output reg c0t0_imem_wr_en, output reg [$clog2(IMEM_WORDS)-1:0] c0t0_imem_wr_addr, output reg [15:0] c0t0_imem_wr_data,
    output reg c0t1_imem_wr_en, output reg [$clog2(IMEM_WORDS)-1:0] c0t1_imem_wr_addr, output reg [15:0] c0t1_imem_wr_data,
    output reg c1t0_imem_wr_en, output reg [$clog2(IMEM_WORDS)-1:0] c1t0_imem_wr_addr, output reg [15:0] c1t0_imem_wr_data,
    output reg c1t1_imem_wr_en, output reg [$clog2(IMEM_WORDS)-1:0] c1t1_imem_wr_addr, output reg [15:0] c1t1_imem_wr_data,
    output reg dmem_wr_en, output reg [$clog2(DMEM_WORDS)-1:0] dmem_wr_addr, output reg [63:0] dmem_wr_data
);
    localparam ADDR_CONTROL    = 20'h00000;
    localparam ADDR_STATUS     = 20'h00004;
    localparam ADDR_C0T0_PC_LO = 20'h00008;
    localparam ADDR_C0T0_PC_HI = 20'h0000C;
    localparam ADDR_C0T1_PC_LO = 20'h00010;
    localparam ADDR_C0T1_PC_HI = 20'h00014;
    localparam ADDR_C1T0_PC_LO = 20'h00018;
    localparam ADDR_C1T0_PC_HI = 20'h0001C;
    localparam ADDR_C1T1_PC_LO = 20'h00020;
    localparam ADDR_C1T1_PC_HI = 20'h00024;

    localparam ADDR_C0T0_IMEM_BASE = 20'h10000;
    localparam ADDR_C0T1_IMEM_BASE = 20'h20000;
    localparam ADDR_C1T0_IMEM_BASE = 20'h30000;
    localparam ADDR_C1T1_IMEM_BASE = 20'h40000;
    localparam ADDR_DMEM_BASE      = 20'h50000;

    // ---- Write address/data handshake (standard AXI4-Lite template) ----
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_awaddr;
    wire slv_reg_wren = S_AXI_AWREADY && S_AXI_AWVALID && S_AXI_WREADY && S_AXI_WVALID;

    always @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_AWREADY <= 1'b0;
            axi_awaddr <= {C_S_AXI_ADDR_WIDTH{1'b0}};
        end else if (!S_AXI_AWREADY && S_AXI_AWVALID && S_AXI_WVALID) begin
            S_AXI_AWREADY <= 1'b1;
            axi_awaddr <= S_AXI_AWADDR;
        end else begin
            S_AXI_AWREADY <= 1'b0;
        end
    end

    always @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN)
            S_AXI_WREADY <= 1'b0;
        else if (!S_AXI_WREADY && S_AXI_WVALID && S_AXI_AWVALID)
            S_AXI_WREADY <= 1'b1;
        else
            S_AXI_WREADY <= 1'b0;
    end

    // ---- dmem write-low staging (per this module's header: write low,
    // then high, per word index -- one staging register is enough since
    // the loader is expected to complete each word before starting the
    // next, the same single-outstanding-transaction assumption AXI4-Lite
    // itself already imposes on its issuer). ----
    reg [31:0] dmem_lo_staging;

    // ---- Sticky halt latches -------------------------------------------
    // c*t*_halt (from riscv64_ooo_proc.v: `commit_req && head_is_ecall`)
    // is a bare combinational pulse, true for exactly the one cycle a
    // thread's ECALL actually commits, then gone -- passing it straight
    // through to STATUS would make "has this thread halted" observable
    // by software only on the exact AXI read that happens to land on
    // that one cycle, which is not a real, reliable polling contract.
    // Latched here instead: set the first time the pulse is seen, held
    // until core_reset is asserted again (a level condition, not an
    // edge -- covers both a fresh S_AXI_ARESETN and a CONTROL write
    // re-arming the cores for a new run), so a poll loop anywhere after
    // the actual halt cycle still observes it.
    reg c0t0_halt_latched, c0t1_halt_latched, c1t0_halt_latched, c1t1_halt_latched;
    always @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN) begin
            c0t0_halt_latched <= 1'b0; c0t1_halt_latched <= 1'b0;
            c1t0_halt_latched <= 1'b0; c1t1_halt_latched <= 1'b0;
        end else if (core_reset) begin
            c0t0_halt_latched <= 1'b0; c0t1_halt_latched <= 1'b0;
            c1t0_halt_latched <= 1'b0; c1t1_halt_latched <= 1'b0;
        end else begin
            if (c0t0_halt) c0t0_halt_latched <= 1'b1;
            if (c0t1_halt) c0t1_halt_latched <= 1'b1;
            if (c1t0_halt) c1t0_halt_latched <= 1'b1;
            if (c1t1_halt) c1t1_halt_latched <= 1'b1;
        end
    end

    always @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN) begin
            core_reset <= 1'b1; // power-on: hold the cores in reset until
                                 // the PS explicitly starts them.
            c0t0_imem_wr_en <= 1'b0; c0t1_imem_wr_en <= 1'b0;
            c1t0_imem_wr_en <= 1'b0; c1t1_imem_wr_en <= 1'b0;
            dmem_wr_en <= 1'b0;
        end else begin
            // Every *_wr_en is a one-shot pulse -- default low each
            // cycle, asserted only on the exact cycle a write commits.
            c0t0_imem_wr_en <= 1'b0; c0t1_imem_wr_en <= 1'b0;
            c1t0_imem_wr_en <= 1'b0; c1t1_imem_wr_en <= 1'b0;
            dmem_wr_en <= 1'b0;

            if (slv_reg_wren) begin
                if (axi_awaddr == ADDR_CONTROL) begin
                    core_reset <= S_AXI_WDATA[0];
                end else if (axi_awaddr[19:16] == ADDR_C0T0_IMEM_BASE[19:16]) begin
                    c0t0_imem_wr_en   <= 1'b1;
                    c0t0_imem_wr_addr <= axi_awaddr[$clog2(IMEM_WORDS)+1:2];
                    c0t0_imem_wr_data <= S_AXI_WDATA[15:0];
                end else if (axi_awaddr[19:16] == ADDR_C0T1_IMEM_BASE[19:16]) begin
                    c0t1_imem_wr_en   <= 1'b1;
                    c0t1_imem_wr_addr <= axi_awaddr[$clog2(IMEM_WORDS)+1:2];
                    c0t1_imem_wr_data <= S_AXI_WDATA[15:0];
                end else if (axi_awaddr[19:16] == ADDR_C1T0_IMEM_BASE[19:16]) begin
                    c1t0_imem_wr_en   <= 1'b1;
                    c1t0_imem_wr_addr <= axi_awaddr[$clog2(IMEM_WORDS)+1:2];
                    c1t0_imem_wr_data <= S_AXI_WDATA[15:0];
                end else if (axi_awaddr[19:16] == ADDR_C1T1_IMEM_BASE[19:16]) begin
                    c1t1_imem_wr_en   <= 1'b1;
                    c1t1_imem_wr_addr <= axi_awaddr[$clog2(IMEM_WORDS)+1:2];
                    c1t1_imem_wr_data <= S_AXI_WDATA[15:0];
                end else if (axi_awaddr[19:16] == ADDR_DMEM_BASE[19:16]) begin
                    if (axi_awaddr[2] == 1'b0) begin
                        dmem_lo_staging <= S_AXI_WDATA;
                    end else begin
                        dmem_wr_en   <= 1'b1;
                        dmem_wr_addr <= axi_awaddr[$clog2(DMEM_WORDS)+2:3];
                        dmem_wr_data <= {S_AXI_WDATA, dmem_lo_staging};
                    end
                end
            end
        end
    end

    // ---- Write response ---------------------------------------------
    always @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_BVALID <= 1'b0;
            S_AXI_BRESP  <= 2'b00;
        end else if (slv_reg_wren && !S_AXI_BVALID) begin
            S_AXI_BVALID <= 1'b1;
            S_AXI_BRESP  <= 2'b00; // OKAY
        end else if (S_AXI_BREADY && S_AXI_BVALID) begin
            S_AXI_BVALID <= 1'b0;
        end
    end

    // ---- Read address handshake ---------------------------------------
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_araddr;

    always @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_ARREADY <= 1'b0;
            axi_araddr <= {C_S_AXI_ADDR_WIDTH{1'b0}};
        end else if (!S_AXI_ARREADY && S_AXI_ARVALID) begin
            S_AXI_ARREADY <= 1'b1;
            axi_araddr <= S_AXI_ARADDR;
        end else begin
            S_AXI_ARREADY <= 1'b0;
        end
    end

    wire slv_reg_rden = S_AXI_ARREADY && S_AXI_ARVALID && !S_AXI_RVALID;

    reg [31:0] status_r;
    always @(*) begin
        status_r = {28'b0, c1t1_halt_latched, c1t0_halt_latched, c0t1_halt_latched, c0t0_halt_latched};
    end

    reg [31:0] read_data_r;
    always @(*) begin
        case (axi_araddr)
            ADDR_CONTROL:    read_data_r = {31'b0, core_reset};
            ADDR_STATUS:     read_data_r = status_r;
            ADDR_C0T0_PC_LO: read_data_r = c0t0_pc[31:0];
            ADDR_C0T0_PC_HI: read_data_r = c0t0_pc[63:32];
            ADDR_C0T1_PC_LO: read_data_r = c0t1_pc[31:0];
            ADDR_C0T1_PC_HI: read_data_r = c0t1_pc[63:32];
            ADDR_C1T0_PC_LO: read_data_r = c1t0_pc[31:0];
            ADDR_C1T0_PC_HI: read_data_r = c1t0_pc[63:32];
            ADDR_C1T1_PC_LO: read_data_r = c1t1_pc[31:0];
            ADDR_C1T1_PC_HI: read_data_r = c1t1_pc[63:32];
            default:         read_data_r = 32'b0; // the write-only memory
                                                    // windows have no real
                                                    // readback -- see
                                                    // module header.
        endcase
    end

    always @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_RVALID <= 1'b0;
            S_AXI_RRESP  <= 2'b00;
            S_AXI_RDATA  <= {C_S_AXI_DATA_WIDTH{1'b0}};
        end else if (slv_reg_rden) begin
            S_AXI_RVALID <= 1'b1;
            S_AXI_RRESP  <= 2'b00; // OKAY
            S_AXI_RDATA  <= read_data_r;
        end else if (S_AXI_RVALID && S_AXI_RREADY) begin
            S_AXI_RVALID <= 1'b0;
        end
    end
endmodule
