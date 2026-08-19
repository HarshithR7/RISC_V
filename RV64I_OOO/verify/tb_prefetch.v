`timescale 1ns / 1ps
// Isolated unit test for l1_cache.v's Phase 10 next-line prefetcher: one
// l1_cache.v + one l2_cache.v + backing memory, same harness as
// tb_ecc_l1.v. A real demand read miss installs line 0 (address 0x1000)
// through the genuine MESI fill path; with nothing else requested, the
// prefetcher must then autonomously fetch line 1 (address 0x1020, one
// LINE_BYTES ahead) into the cache WITHOUT the testbench ever asserting
// cpu_read_req/cpu_write_req for that address. A subsequent real demand
// read to 0x1020 must then complete as an immediate hit (busy high for
// exactly one cycle, no l2_req_valid pulse) -- the actual, measurable
// proof that the prefetch did its job, not just that the line's
// state/tag ended up looking resident by some other means.
module tb_prefetch;
    localparam LINES = 16;
    localparam LINE_BYTES = 32;
    localparam ADDR_BITS = 64;

    reg clk, reset;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg rd_req, wr_req;
    reg [ADDR_BITS-1:0] rd_addr, wr_addr;
    reg [2:0] rd_f3, wr_f3;
    reg [63:0] wr_data;
    wire rd_valid, wr_done, busy;
    wire [63:0] rd_data;
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
        .cpu_read2_req(1'b0), .cpu_read2_addr({ADDR_BITS{1'b0}}), .cpu_read2_func3(3'b0),
        .cpu_read2_hit(), .cpu_read2_data(),
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
               .DMEM_FILE("prefetch_data.mem"), .DMEM_WORDS(4096)) l2 (
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
    integer busy_cycles;
    integer l2_req_pulses;

    task wait_not_busy;
        begin
            while (busy) @(posedge clk);
        end
    endtask

    task do_read;
        input [ADDR_BITS-1:0] addr;
        begin
            @(negedge clk);
            rd_req = 1; rd_addr = addr; rd_f3 = 3'b011; // LD
            @(negedge clk);
            rd_req = 0;
            wait_not_busy;
            @(negedge clk);
        end
    endtask

    initial begin
        errors = 0;
        reset = 1; rd_req = 0; wr_req = 0;
        rd_addr = 0; wr_addr = 0; wr_data = 0; rd_f3 = 0; wr_f3 = 0;
        @(negedge clk); @(negedge clk);
        reset = 0;
        @(negedge clk);

        // Real demand read miss: installs line 0 (0x1000-0x101F).
        do_read(64'h1000);
        if (rd_valid !== 1'b0) begin // rd_valid is a pulse, already gone by now
        end

        // Nothing else requested -- give the autonomous prefetcher time
        // to fire (it must trigger on the very next otherwise-idle
        // cycle) and complete its own L2 round trip.
        @(posedge clk);
        if (!dut.is_prefetch) begin
            $display("[FAIL] prefetch never triggered on the next idle cycle (is_prefetch=%b, prefetch_pending was=%b)",
                      dut.is_prefetch, dut.prefetch_pending);
            errors = errors + 1;
        end
        wait_not_busy;
        @(negedge clk);

        // Line 1 (0x1020-0x103F) must now be resident -- installed purely
        // by the prefetcher, never requested by this testbench.
        if (dut.state[1] == 2'b00) begin
            $display("[FAIL] line 1 not resident after prefetch window (state=%b)", dut.state[1]);
            errors = errors + 1;
        end else if (dut.tag[1] != 64'h1020 >> 9) begin
            // TAG_BITS = 64 - IDX_BITS(4) - OFF_BITS(5) = 55; tag = addr[63:9]
            $display("[FAIL] line 1 resident but wrong tag: %h (want %h)", dut.tag[1], 64'h1020 >> 9);
            errors = errors + 1;
        end

        // The real proof: a genuine demand read to 0x1020 must now be an
        // immediate hit -- busy for exactly one cycle, no L2 traffic at
        // all -- not another multi-cycle miss.
        // Sampled at negedges throughout (settled well after each
        // posedge's NBA updates resolve) -- reading busy/l2_req_valid
        // right after @(posedge clk) in the same timestep would race
        // the DUT's own NBA updates from that same edge and overcount
        // (the same category of race tb_ecc_rob.v's own header warns
        // about for @(posedge clk)-adjacent sampling).
        busy_cycles = 0;
        l2_req_pulses = 0;
        @(negedge clk);
        rd_req = 1; rd_addr = 64'h1020; rd_f3 = 3'b011;
        @(negedge clk);
        rd_req = 0;
        while (busy) begin
            if (l2_req_valid) l2_req_pulses = l2_req_pulses + 1;
            busy_cycles = busy_cycles + 1;
            @(negedge clk);
        end

        if (busy_cycles !== 1) begin
            $display("[FAIL] demand read to prefetched line took %0d busy cycles (want 1 -- an immediate hit)", busy_cycles);
            errors = errors + 1;
        end
        if (l2_req_pulses !== 0) begin
            $display("[FAIL] demand read to prefetched line issued %0d L2 request(s) (want 0)", l2_req_pulses);
            errors = errors + 1;
        end
        if (rd_data !== 64'h0) begin
            $display("[FAIL] prefetched line's data wrong: %h (want 0, memory never written there)", rd_data);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("[PASS] tb_prefetch: all checks passed");
        else
            $display("[FAIL] tb_prefetch: %0d error(s)", errors);
        $finish;
    end
endmodule
