`timescale 1ns / 1ps
// End-to-end smoke test for fpga_top.v -- the actual FPGA-target
// integration, exercised entirely through its real AXI4-Lite interface
// (the same bus-functional-model tasks as tb_axi_ooo_ctrl.v): AXI-loads
// a trivial single-instruction program (a bare ECALL, 0x00000073) into
// all 4 threads' instruction memories (all 4, not just one -- Phase 7's
// SMT shares its execution backend across both threads even when one is
// otherwise idle, so leaving 3 threads' memories at simulation-X would
// risk X-propagation into that shared backend, not a real representation
// of how this would actually be brought up on real hardware anyway,
// where every thread needs a real program loaded before starting),
// releases core_reset via the CONTROL register, and polls STATUS until
// all 4 ecall_halt bits are observed -- proving the whole chain (AXI
// control -> reset release -> AXI-loaded fetch -> decode -> dispatch ->
// commit -> halt -> status readback) actually works together, not just
// each piece in isolation.
module tb_fpga_top;
    localparam IMEM_WORDS = 8192;
    localparam DMEM_WORDS = 4096;

    reg clk, aresetn;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg  [19:0] awaddr; reg awvalid; wire awready;
    reg  [31:0] wdata;  reg [3:0] wstrb; reg wvalid; wire wready;
    wire [1:0] bresp;   wire bvalid;    reg bready;
    reg  [19:0] araddr; reg arvalid;    wire arready;
    wire [31:0] rdata;  wire [1:0] rresp; wire rvalid; reg rready;

    fpga_top #(.IMEM_WORDS(IMEM_WORDS), .DMEM_WORDS(DMEM_WORDS)) dut (
        .S_AXI_ACLK(clk), .S_AXI_ARESETN(aresetn),
        .S_AXI_AWADDR(awaddr), .S_AXI_AWVALID(awvalid), .S_AXI_AWREADY(awready),
        .S_AXI_WDATA(wdata), .S_AXI_WSTRB(wstrb), .S_AXI_WVALID(wvalid), .S_AXI_WREADY(wready),
        .S_AXI_BRESP(bresp), .S_AXI_BVALID(bvalid), .S_AXI_BREADY(bready),
        .S_AXI_ARADDR(araddr), .S_AXI_ARVALID(arvalid), .S_AXI_ARREADY(arready),
        .S_AXI_RDATA(rdata), .S_AXI_RRESP(rresp), .S_AXI_RVALID(rvalid), .S_AXI_RREADY(rready)
    );

    integer errors;
    integer i;
    integer cycles;

    task axi_write;
        input [19:0] addr;
        input [31:0] data;
        begin
            @(negedge clk);
            awaddr = addr; awvalid = 1; wdata = data; wstrb = 4'hF; wvalid = 1;
            bready = 1;
            @(posedge clk);
            while (!(awready && wready)) @(posedge clk);
            @(negedge clk);
            awvalid = 0; wvalid = 0;
            while (!bvalid) @(posedge clk);
            @(negedge clk);
            bready = 0;
        end
    endtask

    task axi_read;
        input [19:0] addr;
        output [31:0] data;
        begin
            @(negedge clk);
            araddr = addr; arvalid = 1; rready = 1;
            @(posedge clk);
            while (!arready) @(posedge clk);
            @(negedge clk);
            arvalid = 0;
            while (!rvalid) @(posedge clk);
            data = rdata;
            @(negedge clk);
            rready = 0;
        end
    endtask

    // Loads 4 back-to-back ECALLs (0x00000073), not just one: the 2-wide
    // fetch always reads pc+4 too (for lane 1), even right after the
    // first ECALL dispatches and before it's actually committed/halted
    // -- leaving that slot at simulation-X (uninitialized AXI-loadable
    // memory) lets lane1_fire go X, which corrupts the low bits of the
    // straight-line PC-advance ternary (`lane1_fire ? pc+8 : pc+4`) the
    // moment X propagates through it. Real Block RAM has no such third
    // logic state -- uninitialized content is some definite (if
    // arbitrary) bit pattern on real hardware, never X -- so this is a
    // simulation-only testbench gap, not an RTL bug: fixed by loading
    // enough real, valid content that lane 1 never reads uninitialized
    // memory in the few cycles before the thread halts.
    task load_ecall_program;
        input [19:0] base;
        integer j;
        begin
            for (j = 0; j < 4; j = j + 1) begin
                axi_write(base + (j*8),     32'h00000073); // low halfword
                axi_write(base + (j*8) + 4, 32'h00000000); // high halfword
            end
        end
    endtask

    reg [31:0] rd;

    initial begin
        errors = 0;
        aresetn = 0;
        awaddr = 0; awvalid = 0; wdata = 0; wstrb = 0; wvalid = 0; bready = 0;
        araddr = 0; arvalid = 0; rready = 0;
        @(negedge clk); @(negedge clk);
        aresetn = 1;
        @(negedge clk);

        // Confirm the cores start held in reset (power-on default).
        axi_read(20'h00000, rd);
        if (rd !== 32'h1) begin
            $display("[FAIL] CONTROL not held-in-reset at power-on: %h", rd);
            errors = errors + 1;
        end

        // Load all 4 threads while still in reset (the real intended
        // use: load, then start).
        load_ecall_program(20'h10000); // C0T0
        load_ecall_program(20'h20000); // C0T1
        load_ecall_program(20'h30000); // C1T0
        load_ecall_program(20'h40000); // C1T1

        // Release reset -- the cores start executing.
        axi_write(20'h00000, 32'h0);

        // Poll STATUS until all 4 threads report ecall_halt, bounded so
        // a real bug produces a clean test failure instead of an
        // infinite loop.
        cycles = 0;
        rd = 0;
        while (rd !== 32'hF && cycles < 2000) begin
            axi_read(20'h00004, rd);
            cycles = cycles + 1;
        end

        if (rd !== 32'hF) begin
            $display("[FAIL] not all 4 threads halted within %0d polls: STATUS=%h", cycles, rd);
            errors = errors + 1;
        end else begin
            $display("[INFO] all 4 threads halted after %0d STATUS polls", cycles);
        end

        if (errors == 0)
            $display("[PASS] tb_fpga_top: all checks passed");
        else
            $display("[FAIL] tb_fpga_top: %0d error(s)", errors);
        $finish;
    end
endmodule
