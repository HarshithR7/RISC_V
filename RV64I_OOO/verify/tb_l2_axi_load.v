`timescale 1ns / 1ps
// Isolated unit test for l2_cache.v's Phase 12 USE_AXI_MEM=1 path: the
// exact configuration a real FPGA build uses. AXI-preloads one full
// cache line's worth of content directly into the backing
// data_memory_axi.v instance (standing in for the PS side writing a
// program/data image before the core ever runs), then drives a genuine
// L1 read miss through l1_cache.v -> l2_cache.v and confirms the
// AXI-loaded content -- not a $readmemh-loaded value, not a hand-poked
// one -- comes back correctly. Same harness style as tb_ecc_l1.v, with
// l2_cache.v's USE_AXI_MEM parameter flipped and the AXI write port
// driven instead of a DMEM_FILE.
module tb_l2_axi_load;
    localparam LINES = 16;
    localparam LINE_BYTES = 32;
    localparam ADDR_BITS = 64;
    localparam DMEM_WORDS = 64;

    reg clk, reset;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg rd_req;
    reg [ADDR_BITS-1:0] rd_addr;
    reg [2:0] rd_f3;
    wire rd_valid, busy;
    wire [63:0] rd_data;

    wire l2_req_valid; wire [1:0] l2_req_type; wire [ADDR_BITS-1:0] l2_req_addr;
    wire [LINE_BYTES*8-1:0] l2_req_wb;
    wire l2_resp_valid; wire [LINE_BYTES*8-1:0] l2_resp_data; wire l2_resp_excl;
    wire snoop_req_valid; wire [1:0] snoop_req_type; wire [ADDR_BITS-1:0] snoop_req_addr;
    wire snoop_resp_hit, snoop_resp_dirty; wire [LINE_BYTES*8-1:0] snoop_resp_data;

    reg axi_wr_en;
    reg [$clog2(DMEM_WORDS)-1:0] axi_wr_addr;
    reg [63:0] axi_wr_data;

    l1_cache #(.LINES(LINES), .LINE_BYTES(LINE_BYTES), .ADDR_BITS(ADDR_BITS)) l1 (
        .clk(clk), .reset(reset),
        .cpu_read_req(rd_req), .cpu_read_addr(rd_addr), .cpu_read_func3(rd_f3),
        .cpu_read_valid(rd_valid), .cpu_read_data(rd_data),
        .cpu_read2_req(1'b0), .cpu_read2_addr({ADDR_BITS{1'b0}}), .cpu_read2_func3(3'b0),
        .cpu_read2_hit(), .cpu_read2_data(),
        .cpu_write_req(1'b0), .cpu_write_addr({ADDR_BITS{1'b0}}), .cpu_write_data(64'b0),
        .cpu_write_func3(3'b0), .cpu_write_done(),
        .busy(busy),
        .l2_req_valid(l2_req_valid), .l2_req_type(l2_req_type), .l2_req_addr(l2_req_addr),
        .l2_req_wb_data(l2_req_wb),
        .l2_resp_valid(l2_resp_valid), .l2_resp_data(l2_resp_data), .l2_resp_exclusive(l2_resp_excl),
        .snoop_req_valid(snoop_req_valid), .snoop_req_type(snoop_req_type), .snoop_req_addr(snoop_req_addr),
        .snoop_resp_hit(snoop_resp_hit), .snoop_resp_dirty(snoop_resp_dirty), .snoop_resp_data(snoop_resp_data),
        .ecc_l1_sbe_fault(), .ecc_l1_dbe_fault()
    );

    l2_cache #(.L2_LINES(64), .LINE_BYTES(LINE_BYTES), .ADDR_BITS(ADDR_BITS),
               .DMEM_FILE("unused.mem"), .DMEM_WORDS(DMEM_WORDS),
               .USE_AXI_MEM(1)) l2 (
        .clk(clk), .reset(reset),
        .c0_req_valid(l2_req_valid), .c0_req_type(l2_req_type), .c0_req_addr(l2_req_addr),
        .c0_req_wb_data(l2_req_wb),
        .c0_resp_valid(l2_resp_valid), .c0_resp_data(l2_resp_data), .c0_resp_exclusive(l2_resp_excl),
        .c1_req_valid(1'b0), .c1_req_type(2'b0), .c1_req_addr(64'b0), .c1_req_wb_data({LINE_BYTES*8{1'b0}}),
        .c1_resp_valid(), .c1_resp_data(), .c1_resp_exclusive(),
        .snoop0_req_valid(snoop_req_valid), .snoop0_req_type(snoop_req_type), .snoop0_req_addr(snoop_req_addr),
        .snoop0_resp_hit(snoop_resp_hit), .snoop0_resp_dirty(snoop_resp_dirty), .snoop0_resp_data(snoop_resp_data),
        .snoop1_req_valid(), .snoop1_req_type(), .snoop1_req_addr(),
        .snoop1_resp_hit(1'b0), .snoop1_resp_dirty(1'b0), .snoop1_resp_data({LINE_BYTES*8{1'b0}}),
        .ecc_l2_sbe_fault(), .ecc_l2_dbe_fault(),
        .axi_wr_en(axi_wr_en), .axi_wr_addr(axi_wr_addr), .axi_wr_data(axi_wr_data)
    );

    integer errors;
    integer i;

    // Holds cpu_read_req asserted (not a one-cycle pulse) until it's
    // actually accepted -- matches how a real caller (lsq.v) drives
    // l1_cache.v, and is robust against l1_cache.v's own Phase 10
    // autonomous background prefetch grabbing the FSM on an idle cycle
    // right before this task's own request arrives (a one-cycle pulse
    // can be silently missed if the FSM isn't in ST_IDLE that exact
    // cycle -- discovered by this test's own first draft, which trusted
    // "busy went low" instead of waiting for the request's own
    // cpu_read_valid pulse specifically, and got back a prefetch fill's
    // incidental read_data_r content instead of its own request's).
    task do_read;
        input [ADDR_BITS-1:0] addr;
        begin
            rd_addr = addr; rd_f3 = 3'b011;
            rd_req = 1;
            @(posedge clk);
            while (!rd_valid) @(posedge clk);
            rd_req = 0;
            @(negedge clk);
        end
    endtask

    initial begin
        errors = 0;
        reset = 1; rd_req = 0; rd_addr = 0; rd_f3 = 0;
        axi_wr_en = 0; axi_wr_addr = 0; axi_wr_data = 0;
        @(negedge clk); @(negedge clk);
        reset = 0;
        @(negedge clk);

        // Preload one full cache line (4 doublewords, addresses
        // 0x2000-0x2018) directly into the backing AXI memory -- this
        // stands in for the PS side writing a program/data image before
        // the core ever starts, the real intended FPGA use of this path.
        for (i = 0; i < LINE_BYTES/8; i = i + 1) begin
            axi_wr_en = 1;
            axi_wr_addr = (64'h2000 >> 3) + i; // word-addressed
            axi_wr_data = 64'hC0FFEE0000000000 | (64'h2000 + i*8);
            @(negedge clk);
        end
        axi_wr_en = 0;
        @(negedge clk);

        // A genuine L1 read MISS -- must go all the way through
        // l2_cache.v's USE_AXI_MEM=1 backing memory, not a
        // $readmemh-loaded or hand-poked value.
        do_read(64'h2008); // LD, second dword in the line
        if (rd_data !== (64'hC0FFEE0000000000 | 64'h2008)) begin
            $display("[FAIL] AXI-preloaded data via real L1 miss: %h (want %h)",
                      rd_data, 64'hC0FFEE0000000000 | 64'h2008);
            errors = errors + 1;
        end

        // A second word in the same already-installed line, now an L1
        // HIT -- confirms the whole line, not just the one requested
        // word, was correctly pulled in from the AXI-loaded backing
        // memory.
        do_read(64'h2010);
        if (rd_data !== (64'hC0FFEE0000000000 | 64'h2010)) begin
            $display("[FAIL] AXI-preloaded data via L1 hit (same line): %h (want %h)",
                      rd_data, 64'hC0FFEE0000000000 | 64'h2010);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("[PASS] tb_l2_axi_load: all checks passed");
        else
            $display("[FAIL] tb_l2_axi_load: %0d error(s)", errors);
        $finish;
    end
endmodule
