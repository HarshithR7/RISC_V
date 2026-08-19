`timescale 1ns / 1ps
// Isolated unit test for l1_cache.v's Phase 10 second read port
// (hit-under-miss): one l1_cache.v + one l2_cache.v + backing memory,
// same harness as tb_ecc_l1.v/tb_prefetch.v. Primes a line B via a real
// demand miss, then starts a genuine, independent demand miss on a
// DIFFERENT line A (a different index) through the PRIMARY port -- while
// that miss is still outstanding (busy stays high across its whole
// multi-cycle L2 round trip), probes B through the SECOND port and
// confirms it reports a hit with correct data on every single cycle of
// that window, all without perturbing A's own in-flight transaction.
// Also confirms the second port correctly REJECTS a probe of a line
// that genuinely isn't resident (not just "always says yes").
module tb_hit_under_miss;
    localparam LINES = 16;
    localparam LINE_BYTES = 32;
    localparam ADDR_BITS = 64;

    reg clk, reset;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg rd_req, wr_req, rd2_req;
    reg [ADDR_BITS-1:0] rd_addr, wr_addr, rd2_addr;
    reg [2:0] rd_f3, wr_f3, rd2_f3;
    reg [63:0] wr_data;
    wire rd_valid, wr_done, busy, rd2_hit;
    wire [63:0] rd_data, rd2_data;
    wire ecc_sbe, ecc_dbe;

    wire l2_req_valid; wire [1:0] l2_req_type; wire [ADDR_BITS-1:0] l2_req_addr;
    wire [LINE_BYTES*8-1:0] l2_req_wb;
    wire l2_resp_valid; wire [LINE_BYTES*8-1:0] l2_resp_data; wire l2_resp_excl;
    wire snoop_req_valid; wire [1:0] snoop_req_type; wire [ADDR_BITS-1:0] snoop_req_addr;
    wire snoop_resp_hit, snoop_resp_dirty; wire [LINE_BYTES*8-1:0] snoop_resp_data;

    l1_cache #(.LINES(LINES), .LINE_BYTES(LINE_BYTES), .ADDR_BITS(ADDR_BITS)) dut (
        .clk(clk), .reset(reset),
        .cpu_read_req(rd_req), .cpu_read_addr(rd_addr), .cpu_read_func3(rd_f3),
        .cpu_read_valid(rd_valid), .cpu_read_data(rd_data),
        .cpu_read2_req(rd2_req), .cpu_read2_addr(rd2_addr), .cpu_read2_func3(rd2_f3),
        .cpu_read2_hit(rd2_hit), .cpu_read2_data(rd2_data),
        .cpu_write_req(wr_req), .cpu_write_addr(wr_addr), .cpu_write_data(wr_data),
        .cpu_write_func3(wr_f3), .cpu_write_done(wr_done),
        .busy(busy),
        .l2_req_valid(l2_req_valid), .l2_req_type(l2_req_type), .l2_req_addr(l2_req_addr),
        .l2_req_wb_data(l2_req_wb),
        .l2_resp_valid(l2_resp_valid), .l2_resp_data(l2_resp_data), .l2_resp_exclusive(l2_resp_excl),
        .snoop_req_valid(snoop_req_valid), .snoop_req_type(snoop_req_type), .snoop_req_addr(snoop_req_addr),
        .snoop_resp_hit(snoop_resp_hit), .snoop_resp_dirty(snoop_resp_dirty), .snoop_resp_data(snoop_resp_data),
        .ecc_l1_sbe_fault(ecc_sbe), .ecc_l1_dbe_fault(ecc_dbe)
    );

    l2_cache #(.L2_LINES(64), .LINE_BYTES(LINE_BYTES), .ADDR_BITS(ADDR_BITS),
               .DMEM_FILE("hum_data.mem"), .DMEM_WORDS(4096)) l2 (
        .clk(clk), .reset(reset),
        .c0_req_valid(l2_req_valid), .c0_req_type(l2_req_type), .c0_req_addr(l2_req_addr),
        .c0_req_wb_data(l2_req_wb),
        .c0_resp_valid(l2_resp_valid), .c0_resp_data(l2_resp_data), .c0_resp_exclusive(l2_resp_excl),
        .c1_req_valid(1'b0), .c1_req_type(2'b0), .c1_req_addr(64'b0), .c1_req_wb_data({LINE_BYTES*8{1'b0}}),
        .c1_resp_valid(), .c1_resp_data(), .c1_resp_exclusive(),
        .snoop0_req_valid(snoop_req_valid), .snoop0_req_type(snoop_req_type), .snoop0_req_addr(snoop_req_addr),
        .snoop0_resp_hit(snoop_resp_hit), .snoop0_resp_dirty(snoop_resp_dirty), .snoop0_resp_data(snoop_resp_data),
        .snoop1_req_valid(), .snoop1_req_type(), .snoop1_req_addr(),
        .snoop1_resp_hit(1'b0), .snoop1_resp_dirty(1'b0), .snoop1_resp_data({LINE_BYTES*8{1'b0}})
    );

    integer errors;
    integer probed_while_busy;

    task wait_not_busy;
        begin
            while (busy) @(negedge clk);
        end
    endtask

    task do_write;
        input [ADDR_BITS-1:0] addr; input [63:0] data;
        begin
            @(negedge clk);
            wr_req = 1; wr_addr = addr; wr_data = data; wr_f3 = 3'b011;
            @(negedge clk);
            wr_req = 0;
            wait_not_busy;
            @(negedge clk);
        end
    endtask

    initial begin
        errors = 0;
        probed_while_busy = 0;
        reset = 1; rd_req = 0; wr_req = 0; rd2_req = 0;
        rd_addr = 0; wr_addr = 0; wr_data = 0; rd_f3 = 0; wr_f3 = 0;
        rd2_addr = 0; rd2_f3 = 0;
        @(negedge clk); @(negedge clk);
        reset = 0;
        @(negedge clk);

        // Prime line B (index 2, address 0x1040) with a known value via
        // a real demand write miss -- installs it in M so there's no
        // ambiguity about what the second port should read back.
        do_write(64'h1040, 64'hCAFEF00DCAFEF00D);

        // Also prove the second port correctly REJECTS a line that truly
        // isn't resident, while everything is still idle (no primary
        // transaction in flight at all) -- a baseline negative check
        // before trusting any "hit" result during the busy window below.
        @(negedge clk);
        rd2_req = 1; rd2_addr = 64'h9000; rd2_f3 = 3'b011; // never touched
        #1;
        if (rd2_hit !== 1'b0) begin
            $display("[FAIL] second port reported a hit for a genuinely non-resident line");
            errors = errors + 1;
        end
        rd2_req = 0;

        // Start a genuine, independent demand MISS on line A (index 0,
        // address 0x1000) through the PRIMARY port.
        @(negedge clk);
        rd_req = 1; rd_addr = 64'h1000; rd_f3 = 3'b011;
        @(negedge clk);
        rd_req = 0;

        // While A's miss is still outstanding (busy stays high across
        // its whole multi-cycle L2 round trip), probe line B through the
        // second port EVERY cycle and confirm it hits with the right
        // data every single time -- proof independent hits keep working
        // throughout, not just opportunistically on one lucky cycle.
        while (busy) begin
            rd2_req = 1; rd2_addr = 64'h1040; rd2_f3 = 3'b011;
            #1; // let the combinational port settle before checking
            if (!rd2_hit || rd2_data !== 64'hCAFEF00DCAFEF00D) begin
                $display("[FAIL] second port missed/wrong-data while primary miss outstanding: hit=%b data=%h (busy=%b, cycle count=%0d)",
                          rd2_hit, rd2_data, busy, probed_while_busy);
                errors = errors + 1;
            end
            probed_while_busy = probed_while_busy + 1;
            @(negedge clk);
        end
        rd2_req = 0;

        if (probed_while_busy < 2) begin
            $display("[FAIL] A's miss completed too fast to meaningfully test hit-under-miss (only %0d busy cycles)", probed_while_busy);
            errors = errors + 1;
        end

        // A's own miss must have completed correctly, undisturbed by all
        // that second-port traffic.
        if (rd_data !== 64'h0) begin
            $display("[FAIL] line A's data wrong after its miss completed: %h (want 0, memory never written there)", rd_data);
            errors = errors + 1;
        end
        if (dut.state[0] == 2'b00) begin
            $display("[FAIL] line A not actually installed after its own miss completed");
            errors = errors + 1;
        end
        if (dut.line[2][63:0] !== 64'hCAFEF00DCAFEF00D) begin
            $display("[FAIL] line B's own stored data was disturbed by second-port traffic: %h", dut.line[2][63:0]);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("[PASS] tb_hit_under_miss: all checks passed (%0d cycles probed while primary miss outstanding)", probed_while_busy);
        else
            $display("[FAIL] tb_hit_under_miss: %0d error(s)", errors);
        $finish;
    end
endmodule
