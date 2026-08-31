`timescale 1ns / 1ps
// Phase 15 (GPU-class coherent interconnect): isolated 3-way coherency
// testbench -- l2_cache.v's new gpu_req_*/gpu_resp_*/snoop_gpu_* port
// driven by a genuine third l1_cache.v instance, alongside the same two
// "core" L1s tb_cache_mesi.v already exercises. Reusing l1_cache.v
// unmodified as the stand-in "GPU's own coherent cache" is deliberate --
// see l2_cache.v's own header: the interconnect protocol is the real
// deliverable here, not a GPU compute core, and any future agent
// (a real GPU's cache hierarchy included) that speaks this same
// req/resp + snoop shape plugs into the identical port.
//
// tb_cache_mesi.v already proves the 2-agent transitions (write-miss->M,
// read-miss-with-dirty-snoop-forward, S-write upgrade, clean/dirty
// eviction) are correct and unaffected by this phase -- this test's job
// is the genuinely NEW 3-agent behavior: a line shared by all 3 at once,
// and a BusUpgr that must invalidate TWO other agents in the same cycle
// (the old 2-agent code only ever had one "other" to invalidate).
module tb_l2_three_way;
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
        .cpu_read2_req(1'b0), .cpu_read2_addr({ADDR_BITS{1'b0}}), .cpu_read2_func3(3'b0),
        .cpu_read2_hit(), .cpu_read2_data(),
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
        .cpu_read2_req(1'b0), .cpu_read2_addr({ADDR_BITS{1'b0}}), .cpu_read2_func3(3'b0),
        .cpu_read2_hit(), .cpu_read2_data(),
        .cpu_write_req(c1_wr_req), .cpu_write_addr(c1_wr_addr), .cpu_write_data(c1_wr_data),
        .cpu_write_func3(c1_wr_f3), .cpu_write_done(c1_wr_done),
        .busy(c1_busy),
        .l2_req_valid(c1_l2_req_valid), .l2_req_type(c1_l2_req_type), .l2_req_addr(c1_l2_req_addr),
        .l2_req_wb_data(c1_l2_req_wb),
        .l2_resp_valid(c1_l2_resp_valid), .l2_resp_data(c1_l2_resp_data), .l2_resp_exclusive(c1_l2_resp_excl),
        .snoop_req_valid(snoop1_req_valid), .snoop_req_type(snoop1_req_type), .snoop_req_addr(snoop1_req_addr),
        .snoop_resp_hit(snoop1_resp_hit), .snoop_resp_dirty(snoop1_resp_dirty), .snoop_resp_data(snoop1_resp_data)
    );

    // ---- "GPU"'s L1 -- a plain l1_cache.v instance, exactly like core0's/
    // core1's above, attached to l2_cache.v's new 3rd port. Standing in
    // for whatever a real coherent accelerator's own private cache would
    // be -- l2_cache.v's interconnect port doesn't know or care that this
    // is "just" another l1_cache.v instance rather than a GPU's actual
    // cache hierarchy.
    reg gp_rd_req, gp_wr_req;
    reg [ADDR_BITS-1:0] gp_rd_addr, gp_wr_addr;
    reg [2:0] gp_rd_f3, gp_wr_f3;
    reg [63:0] gp_wr_data;
    wire gp_rd_valid, gp_wr_done, gp_busy;
    wire [63:0] gp_rd_data;

    wire gp_l2_req_valid; wire [1:0] gp_l2_req_type; wire [ADDR_BITS-1:0] gp_l2_req_addr;
    wire [LINE_BYTES*8-1:0] gp_l2_req_wb;
    wire gp_l2_resp_valid; wire [LINE_BYTES*8-1:0] gp_l2_resp_data; wire gp_l2_resp_excl;
    wire snoop_gp_req_valid; wire [1:0] snoop_gp_req_type; wire [ADDR_BITS-1:0] snoop_gp_req_addr;
    wire snoop_gp_resp_hit, snoop_gp_resp_dirty; wire [LINE_BYTES*8-1:0] snoop_gp_resp_data;

    l1_cache #(.LINES(LINES), .LINE_BYTES(LINE_BYTES), .ADDR_BITS(ADDR_BITS)) l1_gpu (
        .clk(clk), .reset(reset),
        .cpu_read_req(gp_rd_req), .cpu_read_addr(gp_rd_addr), .cpu_read_func3(gp_rd_f3),
        .cpu_read_valid(gp_rd_valid), .cpu_read_data(gp_rd_data),
        .cpu_read2_req(1'b0), .cpu_read2_addr({ADDR_BITS{1'b0}}), .cpu_read2_func3(3'b0),
        .cpu_read2_hit(), .cpu_read2_data(),
        .cpu_write_req(gp_wr_req), .cpu_write_addr(gp_wr_addr), .cpu_write_data(gp_wr_data),
        .cpu_write_func3(gp_wr_f3), .cpu_write_done(gp_wr_done),
        .busy(gp_busy),
        .l2_req_valid(gp_l2_req_valid), .l2_req_type(gp_l2_req_type), .l2_req_addr(gp_l2_req_addr),
        .l2_req_wb_data(gp_l2_req_wb),
        .l2_resp_valid(gp_l2_resp_valid), .l2_resp_data(gp_l2_resp_data), .l2_resp_exclusive(gp_l2_resp_excl),
        .snoop_req_valid(snoop_gp_req_valid), .snoop_req_type(snoop_gp_req_type), .snoop_req_addr(snoop_gp_req_addr),
        .snoop_resp_hit(snoop_gp_resp_hit), .snoop_resp_dirty(snoop_gp_resp_dirty), .snoop_resp_data(snoop_gp_resp_data)
    );

    l2_cache #(.L2_LINES(64), .LINE_BYTES(LINE_BYTES), .ADDR_BITS(ADDR_BITS),
               .DMEM_FILE("l2_three_way_data.mem"), .DMEM_WORDS(4096)) l2 (
        .clk(clk), .reset(reset),
        .c0_req_valid(c0_l2_req_valid), .c0_req_type(c0_l2_req_type), .c0_req_addr(c0_l2_req_addr),
        .c0_req_wb_data(c0_l2_req_wb),
        .c0_resp_valid(c0_l2_resp_valid), .c0_resp_data(c0_l2_resp_data), .c0_resp_exclusive(c0_l2_resp_excl),
        .c1_req_valid(c1_l2_req_valid), .c1_req_type(c1_l2_req_type), .c1_req_addr(c1_l2_req_addr),
        .c1_req_wb_data(c1_l2_req_wb),
        .c1_resp_valid(c1_l2_resp_valid), .c1_resp_data(c1_l2_resp_data), .c1_resp_exclusive(c1_l2_resp_excl),
        .gpu_req_valid(gp_l2_req_valid), .gpu_req_type(gp_l2_req_type), .gpu_req_addr(gp_l2_req_addr),
        .gpu_req_wb_data(gp_l2_req_wb),
        .gpu_resp_valid(gp_l2_resp_valid), .gpu_resp_data(gp_l2_resp_data), .gpu_resp_exclusive(gp_l2_resp_excl),
        .snoop0_req_valid(snoop0_req_valid), .snoop0_req_type(snoop0_req_type), .snoop0_req_addr(snoop0_req_addr),
        .snoop0_resp_hit(snoop0_resp_hit), .snoop0_resp_dirty(snoop0_resp_dirty), .snoop0_resp_data(snoop0_resp_data),
        .snoop1_req_valid(snoop1_req_valid), .snoop1_req_type(snoop1_req_type), .snoop1_req_addr(snoop1_req_addr),
        .snoop1_resp_hit(snoop1_resp_hit), .snoop1_resp_dirty(snoop1_resp_dirty), .snoop1_resp_data(snoop1_resp_data),
        .snoop_gpu_req_valid(snoop_gp_req_valid), .snoop_gpu_req_type(snoop_gp_req_type), .snoop_gpu_req_addr(snoop_gp_req_addr),
        .snoop_gpu_resp_hit(snoop_gp_resp_hit), .snoop_gpu_resp_dirty(snoop_gp_resp_dirty), .snoop_gpu_resp_data(snoop_gp_resp_data)
    );

    integer checks = 0, failures = 0;
    task check_eq64;
        input [511:0] label;
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
        input [511:0] label;
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

    // Inputs driven on negedge, not posedge -- see tb_cache_mesi.v's own
    // comment on this same pattern for why (a same-edge race this
    // project's development actually hit).
    task do_read0; input [63:0] addr; begin
        @(negedge clk); c0_rd_req = 1; c0_rd_addr = addr; c0_rd_f3 = 3'b011;
        @(negedge clk); c0_rd_req = 0;
        while (!c0_rd_valid) @(negedge clk);
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

    task do_readgp; input [63:0] addr; begin
        @(negedge clk); gp_rd_req = 1; gp_rd_addr = addr; gp_rd_f3 = 3'b011;
        @(negedge clk); gp_rd_req = 0;
        while (!gp_rd_valid) @(negedge clk);
    end endtask

    task do_writegp; input [63:0] addr; input [63:0] data; begin
        @(negedge clk); gp_wr_req = 1; gp_wr_addr = addr; gp_wr_data = data; gp_wr_f3 = 3'b011;
        @(negedge clk); gp_wr_req = 0;
        while (!gp_wr_done) @(negedge clk);
    end endtask

    localparam [63:0] ADDR_A = 64'h2000;
    localparam [3:0] IDX_A = (ADDR_A >> 5) & (LINES - 1);

    integer wd;
    initial begin
        for (wd = 0; wd < 2000; wd = wd + 1) @(posedge clk);
        $display("WATCHDOG TIMEOUT: l1_core0.fsm=%0d l1_core1.fsm=%0d l1_gpu.fsm=%0d l2.fsm=%0d",
                  l1_core0.fsm, l1_core1.fsm, l1_gpu.fsm, l2.fsm);
        $finish;
    end

    initial begin
        reset = 1;
        c0_rd_req = 0; c0_wr_req = 0; c1_rd_req = 0; c1_wr_req = 0; gp_rd_req = 0; gp_wr_req = 0;
        #20; reset = 0;
        @(negedge clk);

        // T1: the GPU write-misses ADDR_A -> installs M in its own L1.
        do_writegp(ADDR_A, 64'hFACE_1111_2222_3333);
        check_eq2("T1 gpu state[idx(A)] == M", l1_gpu.state[IDX_A], 2'b11);

        // T2: core0 read-misses ADDR_A -> must see the GPU's dirty data
        // via L2's snoop-forward path (proving a CPU core sees data the
        // GPU produced, through the interconnect), not stale/zero memory.
        // Both end up S.
        do_read0(ADDR_A);
        check_eq64("T2 core0 sees GPU's dirty data via snoop-forward", c0_rd_data, 64'hFACE_1111_2222_3333);
        check_eq2("T2 gpu downgraded M -> S", l1_gpu.state[IDX_A], 2'b01);
        check_eq2("T2 core0 installed S", l1_core0.state[IDX_A], 2'b01);

        // T3: core1 read-misses the now-clean, already-shared line -- a
        // genuine 3-way share (gpu=S, core0=S, core1=S all at once), the
        // state this design could never reach with only 2 agents.
        do_read1(ADDR_A);
        check_eq64("T3 core1 sees the shared line", c1_rd_data, 64'hFACE_1111_2222_3333);
        check_eq2("T3 gpu stays S", l1_gpu.state[IDX_A], 2'b01);
        check_eq2("T3 core0 stays S", l1_core0.state[IDX_A], 2'b01);
        check_eq2("T3 core1 installed S", l1_core1.state[IDX_A], 2'b01);

        // T4: core1 writes the 3-way-shared line -> BusUpgr must invalidate
        // BOTH other agents (gpu AND core0) in the same transaction -- the
        // old 2-agent code only ever had one "other" to invalidate; this
        // is the actual new logic Phase 15 added (evict_want0/1/gpu +
        // normal_want0/1/gpu all independently gating per agent).
        do_write1(ADDR_A, 64'hC1C1_C1C1_C1C1_C1C1);
        check_eq2("T4 gpu invalidated by upgrade", l1_gpu.state[IDX_A], 2'b00);
        check_eq2("T4 core0 invalidated by upgrade", l1_core0.state[IDX_A], 2'b00);
        check_eq2("T4 core1 -> M", l1_core1.state[IDX_A], 2'b11);

        // T5: the GPU re-reads (now Invalid) -> must see core1's *newer*
        // dirty data via another snoop-forward -- proving the reverse
        // direction too (a CPU core's write becomes visible to the GPU
        // through the same coherent fabric).
        do_readgp(ADDR_A);
        check_eq64("T5 gpu sees core1's newer dirty data", gp_rd_data, 64'hC1C1_C1C1_C1C1_C1C1);
        check_eq2("T5 core1 downgraded M -> S", l1_core1.state[IDX_A], 2'b01);
        check_eq2("T5 gpu installed S", l1_gpu.state[IDX_A], 2'b01);

        $display("");
        if (failures == 0)
            $display("tb_l2_three_way: ALL PASS (%0d checks)", checks);
        else
            $display("tb_l2_three_way: %0d/%0d FAILED", failures, checks);
        $finish;
    end
endmodule
