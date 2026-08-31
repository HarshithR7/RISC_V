`timescale 1ns / 1ps
// Isolated unit test for axi_ooo_ctrl.v: a real AXI4-Lite bus-functional
// model (axi_write/axi_read tasks driving the actual AWADDR/AWVALID/
// WDATA/WVALID/BREADY and ARADDR/ARVALID/RREADY handshakes, not a
// shortcut) exercises the address map end to end -- CONTROL, STATUS/PC
// readback, all 4 imem windows, and the 2-step dmem write-low-then-high
// protocol -- confirming both the AXI protocol handshaking itself and
// the address decoding land on the right target with the right value.
module tb_axi_ooo_ctrl;
    localparam IMEM_WORDS = 8192;
    localparam DMEM_WORDS = 4096;

    reg clk, aresetn;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg  [19:0] awaddr; reg awvalid; wire awready;
    reg  [31:0] wdata;  reg [3:0] wstrb; reg wvalid; wire wready;
    wire [1:0] bresp;   wire bvalid;    reg bready;
    reg  [19:0] araddr; reg arvalid;    wire arready;
    wire [31:0] rdata;  wire [1:0] rresp; wire rvalid; reg rready;

    wire core_reset;
    reg c0t0_halt, c0t1_halt, c1t0_halt, c1t1_halt;
    reg [63:0] c0t0_pc, c0t1_pc, c1t0_pc, c1t1_pc;

    wire c0t0_imem_wr_en; wire [12:0] c0t0_imem_wr_addr; wire [15:0] c0t0_imem_wr_data;
    wire c0t1_imem_wr_en; wire [12:0] c0t1_imem_wr_addr; wire [15:0] c0t1_imem_wr_data;
    wire c1t0_imem_wr_en; wire [12:0] c1t0_imem_wr_addr; wire [15:0] c1t0_imem_wr_data;
    wire c1t1_imem_wr_en; wire [12:0] c1t1_imem_wr_addr; wire [15:0] c1t1_imem_wr_data;
    wire dmem_wr_en; wire [11:0] dmem_wr_addr; wire [63:0] dmem_wr_data;

    axi_ooo_ctrl #(.IMEM_WORDS(IMEM_WORDS), .DMEM_WORDS(DMEM_WORDS)) dut (
        .S_AXI_ACLK(clk), .S_AXI_ARESETN(aresetn),
        .S_AXI_AWADDR(awaddr), .S_AXI_AWVALID(awvalid), .S_AXI_AWREADY(awready),
        .S_AXI_WDATA(wdata), .S_AXI_WSTRB(wstrb), .S_AXI_WVALID(wvalid), .S_AXI_WREADY(wready),
        .S_AXI_BRESP(bresp), .S_AXI_BVALID(bvalid), .S_AXI_BREADY(bready),
        .S_AXI_ARADDR(araddr), .S_AXI_ARVALID(arvalid), .S_AXI_ARREADY(arready),
        .S_AXI_RDATA(rdata), .S_AXI_RRESP(rresp), .S_AXI_RVALID(rvalid), .S_AXI_RREADY(rready),
        .core_reset(core_reset),
        .c0t0_halt(c0t0_halt), .c0t1_halt(c0t1_halt), .c1t0_halt(c1t0_halt), .c1t1_halt(c1t1_halt),
        .c0t0_pc(c0t0_pc), .c0t1_pc(c0t1_pc), .c1t0_pc(c1t0_pc), .c1t1_pc(c1t1_pc),
        .c0t0_imem_wr_en(c0t0_imem_wr_en), .c0t0_imem_wr_addr(c0t0_imem_wr_addr), .c0t0_imem_wr_data(c0t0_imem_wr_data),
        .c0t1_imem_wr_en(c0t1_imem_wr_en), .c0t1_imem_wr_addr(c0t1_imem_wr_addr), .c0t1_imem_wr_data(c0t1_imem_wr_data),
        .c1t0_imem_wr_en(c1t0_imem_wr_en), .c1t0_imem_wr_addr(c1t0_imem_wr_addr), .c1t0_imem_wr_data(c1t0_imem_wr_data),
        .c1t1_imem_wr_en(c1t1_imem_wr_en), .c1t1_imem_wr_addr(c1t1_imem_wr_addr), .c1t1_imem_wr_data(c1t1_imem_wr_data),
        .dmem_wr_en(dmem_wr_en), .dmem_wr_addr(dmem_wr_addr), .dmem_wr_data(dmem_wr_data)
    );

    // Capture registers: c*_imem_wr_en/dmem_wr_en are one-shot pulses
    // that could go low again before a polling task (waiting out the
    // BVALID/BREADY handshake, several cycles after the actual register
    // write) gets a chance to sample them synchronously. Latching
    // whenever a pulse is SEEN, on any cycle, decouples the check from
    // exactly which cycle the pulse landed on.
    reg cap_c0t0_en, cap_c0t1_en, cap_c1t0_en, cap_c1t1_en, cap_dmem_en;
    reg [12:0] cap_c0t0_addr, cap_c0t1_addr, cap_c1t0_addr, cap_c1t1_addr;
    reg [11:0] cap_dmem_addr;
    reg [15:0] cap_c0t0_data, cap_c0t1_data, cap_c1t0_data, cap_c1t1_data;
    reg [63:0] cap_dmem_data;
    always @(posedge clk) begin
        if (c0t0_imem_wr_en) begin cap_c0t0_en <= 1; cap_c0t0_addr <= c0t0_imem_wr_addr; cap_c0t0_data <= c0t0_imem_wr_data; end
        if (c0t1_imem_wr_en) begin cap_c0t1_en <= 1; cap_c0t1_addr <= c0t1_imem_wr_addr; cap_c0t1_data <= c0t1_imem_wr_data; end
        if (c1t0_imem_wr_en) begin cap_c1t0_en <= 1; cap_c1t0_addr <= c1t0_imem_wr_addr; cap_c1t0_data <= c1t0_imem_wr_data; end
        if (c1t1_imem_wr_en) begin cap_c1t1_en <= 1; cap_c1t1_addr <= c1t1_imem_wr_addr; cap_c1t1_data <= c1t1_imem_wr_data; end
        if (dmem_wr_en)      begin cap_dmem_en <= 1; cap_dmem_addr <= dmem_wr_addr;      cap_dmem_data <= dmem_wr_data;      end
    end

    integer errors;

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

    reg [31:0] rd;

    initial begin
        errors = 0;
        aresetn = 0;
        awaddr = 0; awvalid = 0; wdata = 0; wstrb = 0; wvalid = 0; bready = 0;
        araddr = 0; arvalid = 0; rready = 0;
        c0t0_halt = 0; c0t1_halt = 0; c1t0_halt = 0; c1t1_halt = 0;
        c0t0_pc = 0; c0t1_pc = 0; c1t0_pc = 0; c1t1_pc = 0;
        cap_c0t0_en = 0; cap_c0t1_en = 0; cap_c1t0_en = 0; cap_c1t1_en = 0; cap_dmem_en = 0;
        @(negedge clk); @(negedge clk);
        aresetn = 1;
        @(negedge clk);

        // Power-on: core_reset must start asserted (hold cores in reset
        // until the PS explicitly starts them).
        if (core_reset !== 1'b1) begin
            $display("[FAIL] core_reset not asserted at power-on: %b", core_reset);
            errors = errors + 1;
        end

        // CONTROL readback.
        axi_read(20'h00000, rd);
        if (rd !== 32'h1) begin
            $display("[FAIL] CONTROL readback at power-on: %h (want 1)", rd);
            errors = errors + 1;
        end

        // Clear core_reset (the "start running" transition).
        axi_write(20'h00000, 32'h0);
        if (core_reset !== 1'b0) begin
            $display("[FAIL] core_reset after CONTROL=0 write: %b", core_reset);
            errors = errors + 1;
        end

        // STATUS readback -- drive a specific halt pattern and confirm
        // the bit assignment matches the documented map.
        c0t0_halt = 1; c0t1_halt = 0; c1t0_halt = 1; c1t1_halt = 0;
        axi_read(20'h00004, rd);
        if (rd !== 32'h5) begin // bit0=1 (c0t0), bit2=1 (c1t0)
            $display("[FAIL] STATUS readback: %h (want 5)", rd);
            errors = errors + 1;
        end

        // PC readback -- confirm the 64-bit value splits correctly
        // across the LO/HI register pair.
        c1t1_pc = 64'hDEADBEEFCAFEF00D;
        axi_read(20'h00020, rd);
        if (rd !== 32'hCAFEF00D) begin
            $display("[FAIL] C1T1_PC_LO readback: %h (want CAFEF00D)", rd);
            errors = errors + 1;
        end
        axi_read(20'h00024, rd);
        if (rd !== 32'hDEADBEEF) begin
            $display("[FAIL] C1T1_PC_HI readback: %h (want DEADBEEF)", rd);
            errors = errors + 1;
        end

        // IMEM windows -- one write per region, confirm it lands on the
        // right target's wr_en/addr/data and nowhere else. Halfword
        // index 5 -> byte offset 5*4 = 0x14 within each window. Checked
        // via the cap_* capture registers (see their own comment above):
        // the wr_en pulse is one cycle wide and may already be gone by
        // the time axi_write() finishes waiting out the BVALID/BREADY
        // handshake, several cycles later.
        axi_write(20'h10014, 32'h0000ABCD); // C0T0
        if (!cap_c0t0_en || cap_c0t0_addr !== 13'd5 || cap_c0t0_data !== 16'hABCD) begin
            $display("[FAIL] C0T0_IMEM write: en=%b addr=%0d data=%h", cap_c0t0_en, cap_c0t0_addr, cap_c0t0_data);
            errors = errors + 1;
        end
        if (cap_c0t1_en || cap_c1t0_en || cap_c1t1_en) begin
            $display("[FAIL] C0T0_IMEM write also pulsed a different thread's imem port");
            errors = errors + 1;
        end

        axi_write(20'h20018, 32'h00001234); // C0T1, halfword index 6
        if (!cap_c0t1_en || cap_c0t1_addr !== 13'd6 || cap_c0t1_data !== 16'h1234) begin
            $display("[FAIL] C0T1_IMEM write: en=%b addr=%0d data=%h", cap_c0t1_en, cap_c0t1_addr, cap_c0t1_data);
            errors = errors + 1;
        end

        axi_write(20'h3000C, 32'h00005678); // C1T0, halfword index 3
        if (!cap_c1t0_en || cap_c1t0_addr !== 12'd3 || cap_c1t0_data !== 16'h5678) begin
            $display("[FAIL] C1T0_IMEM write: en=%b addr=%0d data=%h", cap_c1t0_en, cap_c1t0_addr, cap_c1t0_data);
            errors = errors + 1;
        end

        axi_write(20'h40000, 32'h00009ABC); // C1T1, halfword index 0
        if (!cap_c1t1_en || cap_c1t1_addr !== 12'd0 || cap_c1t1_data !== 16'h9ABC) begin
            $display("[FAIL] C1T1_IMEM write: en=%b addr=%0d data=%h", cap_c1t1_en, cap_c1t1_addr, cap_c1t1_data);
            errors = errors + 1;
        end

        // DMEM: write-low (no commit yet), then write-high (commits the
        // full 64-bit value) at word index 2 (byte offset 2*8=0x10).
        axi_write(20'h50010, 32'hAAAAAAAA); // low half
        if (cap_dmem_en) begin
            $display("[FAIL] DMEM low-half write incorrectly pulsed wr_en (must wait for the high half)");
            errors = errors + 1;
        end
        axi_write(20'h50014, 32'hBBBBBBBB); // high half -> commit
        if (!cap_dmem_en || cap_dmem_addr !== 12'd2 || cap_dmem_data !== 64'hBBBBBBBBAAAAAAAA) begin
            $display("[FAIL] DMEM committed write: en=%b addr=%0d data=%h (want addr=2 data=BBBBBBBBAAAAAAAA)",
                      cap_dmem_en, cap_dmem_addr, cap_dmem_data);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("[PASS] tb_axi_ooo_ctrl: all checks passed");
        else
            $display("[FAIL] tb_axi_ooo_ctrl: %0d error(s)", errors);
        $finish;
    end
endmodule
