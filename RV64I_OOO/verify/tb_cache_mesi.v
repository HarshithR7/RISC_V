`timescale 1ns / 1ps
// Isolated coherency testbench: two l1_cache.v instances + one shared
// l2_cache.v, driven directly (no OoO core involved) -- same "verify the
// new, bug-prone logic in isolation before wiring it into the pipeline"
// discipline this project used for tb_rob_rat.v/tb_div_fu.v. Proves real
// MESI state transitions (write-miss -> M, read-miss with a dirty snoop
// forward -> S/S, S-write upgrade invalidating the other core -> M,
// clean and dirty eviction) rather than just "the two cores don't crash."
module tb_cache_mesi;
    localparam LINES = 16;
    localparam LINE_BYTES = 32;
    localparam ADDR_BITS = 64;
    localparam VLEN = LINE_BYTES * 8;

    reg clk, reset;
    initial begin clk = 0; forever #5 clk = ~clk; end

    // ---- core0's L1 -----------------------------------------------------
    reg c0_rd_req, c0_wr_req;
    reg [ADDR_BITS-1:0] c0_rd_addr, c0_wr_addr;
    reg [2:0] c0_rd_f3, c0_wr_f3;
    reg [63:0] c0_wr_data;
    wire c0_rd_valid, c0_wr_done, c0_busy;
    wire [63:0] c0_rd_data;

    wire c0_l2_req_valid; wire [1:0] c0_l2_req_type; wire [ADDR_BITS-1:0] c0_l2_req_addr;
    wire [LINE_BYTES*8-1:0] c0_l2_req_wb;
    wire c0_l2_resp_valid; wire [LINE_BYTES*8-1:0] c0_l2_resp_data; wire c0_l2_resp_excl;
    wire snoop0_req_valid; wire [1:0] snoop0_req_type; wire [ADDR_BITS-1:0] snoop0_req_addr;
    wire snoop0_resp_hit, snoop0_resp_dirty; wire [LINE_BYTES*8-1:0] snoop0_resp_data;

    l1_cache #(.LINES(LINES), .LINE_BYTES(LINE_BYTES), .ADDR_BITS(ADDR_BITS)) l1_core0 (
        .clk(clk), .reset(reset),
        .cpu_read_req(c0_rd_req), .cpu_read_addr(c0_rd_addr), .cpu_read_func3(c0_rd_f3),
        .cpu_read_valid(c0_rd_valid), .cpu_read_data(c0_rd_data),
        .cpu_write_req(c0_wr_req), .cpu_write_addr(c0_wr_addr), .cpu_write_data(c0_wr_data),
        .cpu_write_func3(c0_wr_f3), .cpu_write_done(c0_wr_done),
        .busy(c0_busy),
        .l2_req_valid(c0_l2_req_valid), .l2_req_type(c0_l2_req_type), .l2_req_addr(c0_l2_req_addr),
        .l2_req_wb_data(c0_l2_req_wb),
        .l2_resp_valid(c0_l2_resp_valid), .l2_resp_data(c0_l2_resp_data), .l2_resp_exclusive(c0_l2_resp_excl),
        .snoop_req_valid(snoop0_req_valid), .snoop_req_type(snoop0_req_type), .snoop_req_addr(snoop0_req_addr),
        .snoop_resp_hit(snoop0_resp_hit), .snoop_resp_dirty(snoop0_resp_dirty), .snoop_resp_data(snoop0_resp_data)
    );

    // ---- core1's L1 -----------------------------------------------------
    reg c1_rd_req, c1_wr_req;
    reg [ADDR_BITS-1:0] c1_rd_addr, c1_wr_addr;
    reg [2:0] c1_rd_f3, c1_wr_f3;
    reg [63:0] c1_wr_data;
    wire c1_rd_valid, c1_wr_done, c1_busy;
    wire [63:0] c1_rd_data;

    wire c1_l2_req_valid; wire [1:0] c1_l2_req_type; wire [ADDR_BITS-1:0] c1_l2_req_addr;
    wire [LINE_BYTES*8-1:0] c1_l2_req_wb;
    wire c1_l2_resp_valid; wire [LINE_BYTES*8-1:0] c1_l2_resp_data; wire c1_l2_resp_excl;
    wire snoop1_req_valid; wire [1:0] snoop1_req_type; wire [ADDR_BITS-1:0] snoop1_req_addr;
    wire snoop1_resp_hit, snoop1_resp_dirty; wire [LINE_BYTES*8-1:0] snoop1_resp_data;

    l1_cache #(.LINES(LINES), .LINE_BYTES(LINE_BYTES), .ADDR_BITS(ADDR_BITS)) l1_core1 (
        .clk(clk), .reset(reset),
        .cpu_read_req(c1_rd_req), .cpu_read_addr(c1_rd_addr), .cpu_read_func3(c1_rd_f3),
        .cpu_read_valid(c1_rd_valid), .cpu_read_data(c1_rd_data),
        .cpu_write_req(c1_wr_req), .cpu_write_addr(c1_wr_addr), .cpu_write_data(c1_wr_data),
        .cpu_write_func3(c1_wr_f3), .cpu_write_done(c1_wr_done),
        .busy(c1_busy),
        .l2_req_valid(c1_l2_req_valid), .l2_req_type(c1_l2_req_type), .l2_req_addr(c1_l2_req_addr),
        .l2_req_wb_data(c1_l2_req_wb),
        .l2_resp_valid(c1_l2_resp_valid), .l2_resp_data(c1_l2_resp_data), .l2_resp_exclusive(c1_l2_resp_excl),
        .snoop_req_valid(snoop1_req_valid), .snoop_req_type(snoop1_req_type), .snoop_req_addr(snoop1_req_addr),
        .snoop_resp_hit(snoop1_resp_hit), .snoop_resp_dirty(snoop1_resp_dirty), .snoop_resp_data(snoop1_resp_data)
    );

    l2_cache #(.L2_LINES(64), .LINE_BYTES(LINE_BYTES), .ADDR_BITS(ADDR_BITS),
               .DMEM_FILE("cache_mesi_data.mem"), .DMEM_WORDS(4096)) l2 (
        .clk(clk), .reset(reset),
        .c0_req_valid(c0_l2_req_valid), .c0_req_type(c0_l2_req_type), .c0_req_addr(c0_l2_req_addr),
        .c0_req_wb_data(c0_l2_req_wb),
        .c0_resp_valid(c0_l2_resp_valid), .c0_resp_data(c0_l2_resp_data), .c0_resp_exclusive(c0_l2_resp_excl),
        .c1_req_valid(c1_l2_req_valid), .c1_req_type(c1_l2_req_type), .c1_req_addr(c1_l2_req_addr),
        .c1_req_wb_data(c1_l2_req_wb),
        .c1_resp_valid(c1_l2_resp_valid), .c1_resp_data(c1_l2_resp_data), .c1_resp_exclusive(c1_l2_resp_excl),
        .snoop0_req_valid(snoop0_req_valid), .snoop0_req_type(snoop0_req_type), .snoop0_req_addr(snoop0_req_addr),
        .snoop0_resp_hit(snoop0_resp_hit), .snoop0_resp_dirty(snoop0_resp_dirty), .snoop0_resp_data(snoop0_resp_data),
        .snoop1_req_valid(snoop1_req_valid), .snoop1_req_type(snoop1_req_type), .snoop1_req_addr(snoop1_req_addr),
        .snoop1_resp_hit(snoop1_resp_hit), .snoop1_resp_dirty(snoop1_resp_dirty), .snoop1_resp_data(snoop1_resp_data)
    );

    integer checks = 0, failures = 0;
    task check_eq64;
        input [255:0] label;
        input [63:0] got, expected;
        begin
            checks = checks + 1;
            if (got !== expected) begin
                failures = failures + 1;
                $display("FAIL %0s: got=%h expected=%h", label, got, expected);
            end else begin
                $display("PASS %0s: %h", label, got);
            end
        end
    endtask

    task check_eq2;
        input [255:0] label;
        input [1:0] got, expected;
        begin
            checks = checks + 1;
            if (got !== expected) begin
                failures = failures + 1;
                $display("FAIL %0s: got=%b expected=%b", label, got, expected);
            end else begin
                $display("PASS %0s: %b", label, got);
            end
        end
    endtask

    // Inputs are driven on negedge, not posedge: changing a DUT input with
    // a blocking assignment on the *same* posedge the DUT's own always
    // block samples it is a genuine same-edge race with no guaranteed
    // ordering between the two processes (found the hard way -- this
    // testbench hung indefinitely until fixed). Driving on the opposite
    // edge guarantees the new value is stable for a full half-cycle
    // before the DUT ever samples it.
    task do_read0; input [63:0] addr; begin
        @(negedge clk); c0_rd_req = 1; c0_rd_addr = addr; c0_rd_f3 = 3'b011;
        @(negedge clk); c0_rd_req = 0;
        while (!c0_rd_valid) @(negedge clk);
    end endtask

    task do_write0; input [63:0] addr; input [63:0] data; begin
        @(negedge clk); c0_wr_req = 1; c0_wr_addr = addr; c0_wr_data = data; c0_wr_f3 = 3'b011;
        @(negedge clk); c0_wr_req = 0;
        while (!c0_wr_done) @(negedge clk);
    end endtask

    task do_read1; input [63:0] addr; begin
        @(negedge clk); c1_rd_req = 1; c1_rd_addr = addr; c1_rd_f3 = 3'b011;
        @(negedge clk); c1_rd_req = 0;
        while (!c1_rd_valid) @(negedge clk);
    end endtask

    task do_write1; input [63:0] addr; input [63:0] data; begin
        @(negedge clk); c1_wr_req = 1; c1_wr_addr = addr; c1_wr_data = data; c1_wr_f3 = 3'b011;
        @(negedge clk); c1_wr_req = 0;
        while (!c1_wr_done) @(negedge clk);
    end endtask

    localparam [63:0] ADDR_A = 64'h1000;
    localparam [63:0] ADDR_A_ALIAS = 64'h1200; // same L1 index as ADDR_A (16 lines * 32B = 0x200 apart)
    localparam [3:0] IDX_A = (ADDR_A >> 5) & (LINES - 1);

    integer wd;
    initial begin
        for (wd = 0; wd < 2000; wd = wd + 1) @(posedge clk);
        $display("WATCHDOG TIMEOUT: l1_core0.fsm=%0d l1_core1.fsm=%0d l2.fsm=%0d", l1_core0.fsm, l1_core1.fsm, l2.fsm);
        $finish;
    end

    initial begin
        reset = 1; c0_rd_req = 0; c0_wr_req = 0; c1_rd_req = 0; c1_wr_req = 0;
        #20; reset = 0;
        @(negedge clk);

        // T1: core0 write-misses ADDR_A -> installs M.
        do_write0(ADDR_A, 64'hAAAA_AAAA_AAAA_AAAA);
        check_eq2("T1 core0 state[idx(A)] == M", l1_core0.state[IDX_A], 2'b11);
        do_read0(ADDR_A);
        check_eq64("T1 core0 reads back own write", c0_rd_data, 64'hAAAA_AAAA_AAAA_AAAA);

        // T2: core1 read-misses ADDR_A -> must see core0's dirty (M) data
        // via L2's snoop-forward path, not stale/zero memory. Both end up S.
        do_read1(ADDR_A);
        check_eq64("T2 core1 sees core0's dirty data via snoop-forward", c1_rd_data, 64'hAAAA_AAAA_AAAA_AAAA);
        check_eq2("T2 core0 downgraded M -> S", l1_core0.state[IDX_A], 2'b01);
        check_eq2("T2 core1 installed S", l1_core1.state[IDX_A], 2'b01);

        // T3: core1 writes the now-Shared line -> BusUpgr, invalidates
        // core0 (no data transfer needed, core1's own S copy was current),
        // core1 -> M.
        do_write1(ADDR_A, 64'hBEEF_BEEF_BEEF_BEEF);
        check_eq2("T3 core0 invalidated by upgrade", l1_core0.state[IDX_A], 2'b00);
        check_eq2("T3 core1 -> M", l1_core1.state[IDX_A], 2'b11);

        // T4: core0 re-reads (now Invalid) -> must see core1's *new* dirty
        // value via another snoop-forward, not the stale AAAA...A value.
        do_read0(ADDR_A);
        check_eq64("T4 core0 sees core1's newer dirty data", c0_rd_data, 64'hBEEF_BEEF_BEEF_BEEF);
        check_eq2("T4 core1 downgraded M -> S", l1_core1.state[IDX_A], 2'b01);
        check_eq2("T4 core0 installed S", l1_core0.state[IDX_A], 2'b01);

        // T5: clean eviction -- core0 accesses an address that maps to the
        // same L1 index as ADDR_A while ADDR_A is only clean (S) there;
        // no writeback needed, just silently replaced.
        do_write0(ADDR_A_ALIAS, 64'h1111_1111_1111_1111);
        check_eq64("T5 core0's tag now the alias address's",
                   {l1_core0.tag[IDX_A], IDX_A, 5'b0},
                   {ADDR_A_ALIAS[ADDR_BITS-1:5], 5'b0});

        // T6: dirty eviction -- core0 makes ADDR_A dirty (M) again, then
        // forces it out via the same aliasing address *without* an
        // intervening read, so the only way the M data can be recovered
        // later is if the eviction's writeback path actually flushed it
        // through L2 into memory.
        do_write0(ADDR_A, 64'h2222_2222_2222_2222);
        check_eq2("T6 core0 ADDR_A -> M again", l1_core0.state[IDX_A], 2'b11);
        do_write0(ADDR_A_ALIAS, 64'h3333_3333_3333_3333); // forces the dirty eviction
        do_read1(ADDR_A);
        check_eq64("T6 dirty-eviction writeback recovered by core1", c1_rd_data, 64'h2222_2222_2222_2222);

        $display("");
        if (failures == 0)
            $display("tb_cache_mesi: ALL PASS (%0d checks)", checks);
        else
            $display("tb_cache_mesi: %0d/%0d FAILED", failures, checks);
        $finish;
    end
endmodule
