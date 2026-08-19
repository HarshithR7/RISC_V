`timescale 1ns / 1ps
// Isolated unit test for l2_cache.v's Phase 9 ECC integration: one
// l2_cache.v driven directly (core 0's request/response ports, core 1's
// side tied off, same convention as riscv64_ooo_proc_solo.v), no
// l1_cache.v involved -- exercising L2's own request/response protocol
// straight, the same "test the new logic in isolation first" discipline
// as tb_ecc_l1.v.
//
// A real BusRd miss pulls a line in from the backing memory (a genuine
// fill through the real FSM, not a hand-poked initial value); a second
// BusRd for the same address then hits directly in l2_data[] (skipping
// ST_MEM_FETCH/ST_MEM_WAIT entirely), which is the read path this test
// actually cares about. Corruption is injected with the same one-time
// hierarchical procedural write used throughout this project's Phase 9
// work (see tb_lockstep.v's header for why not force/release).
module tb_ecc_l2;
    localparam LINE_BYTES = 32;
    localparam ADDR_BITS = 64;
    localparam VLEN = LINE_BYTES * 8;
    localparam REQ_BUSRD = 2'b00;

    reg clk, reset;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg c0_req_valid; reg [1:0] c0_req_type; reg [ADDR_BITS-1:0] c0_req_addr;
    reg [VLEN-1:0] c0_req_wb_data;
    wire c0_resp_valid; wire [VLEN-1:0] c0_resp_data; wire c0_resp_exclusive;
    wire ecc_sbe, ecc_dbe;

    l2_cache #(.L2_LINES(64), .LINE_BYTES(LINE_BYTES), .ADDR_BITS(ADDR_BITS),
               .DMEM_FILE("ecc_l2_data.mem"), .DMEM_WORDS(4096)) dut (
        .clk(clk), .reset(reset),
        .c0_req_valid(c0_req_valid), .c0_req_type(c0_req_type), .c0_req_addr(c0_req_addr),
        .c0_req_wb_data(c0_req_wb_data),
        .c0_resp_valid(c0_resp_valid), .c0_resp_data(c0_resp_data), .c0_resp_exclusive(c0_resp_exclusive),
        .c1_req_valid(1'b0), .c1_req_type(2'b0), .c1_req_addr(64'b0), .c1_req_wb_data({VLEN{1'b0}}),
        .c1_resp_valid(), .c1_resp_data(), .c1_resp_exclusive(),
        .snoop0_req_valid(), .snoop0_req_type(), .snoop0_req_addr(),
        .snoop0_resp_hit(1'b0), .snoop0_resp_dirty(1'b0), .snoop0_resp_data({VLEN{1'b0}}),
        .snoop1_req_valid(), .snoop1_req_type(), .snoop1_req_addr(),
        .snoop1_resp_hit(1'b0), .snoop1_resp_dirty(1'b0), .snoop1_resp_data({VLEN{1'b0}}),
        .ecc_l2_sbe_fault(ecc_sbe), .ecc_l2_dbe_fault(ecc_dbe)
    );

    integer errors;

    task do_busrd;
        input [ADDR_BITS-1:0] addr;
        begin
            @(negedge clk);
            c0_req_valid = 1; c0_req_type = REQ_BUSRD; c0_req_addr = addr;
            @(posedge clk);
            while (!c0_resp_valid) @(posedge clk);
            @(negedge clk);
            c0_req_valid = 0;
            @(negedge clk); @(negedge clk); // clear the ST_COOLDOWN gap
        end
    endtask

    initial begin
        errors = 0;
        reset = 1; c0_req_valid = 0; c0_req_type = 0; c0_req_addr = 0; c0_req_wb_data = 0;
        @(negedge clk); @(negedge clk);
        reset = 0;
        @(negedge clk);

        // Real fill through the genuine memory-fetch FSM path.
        do_busrd(64'h2000);
        if (ecc_sbe !== 1'b0 || ecc_dbe !== 1'b0) begin
            $display("[FAIL] fill fault flags: sbe=%b dbe=%b (want 0,0)", ecc_sbe, ecc_dbe);
            errors = errors + 1;
        end

        // Second BusRd for the same line: hits directly in l2_data[],
        // clean, no corruption yet.
        do_busrd(64'h2000);
        if (ecc_sbe !== 1'b0 || ecc_dbe !== 1'b0) begin
            $display("[FAIL] clean hit: sbe=%b dbe=%b (want 0,0)", ecc_sbe, ecc_dbe);
            errors = errors + 1;
        end

        // Single-bit corruption directly in L2's own storage for that
        // line's index (0x2000 / 32 bytes-per-line = index 256 mod 64
        // L2_LINES = index 0).
        dut.l2_data[0][100] = ~dut.l2_data[0][100];
        do_busrd(64'h2000);
        if (ecc_sbe !== 1'b1 || ecc_dbe !== 1'b0) begin
            $display("[FAIL] single-bit L2 corruption: sbe=%b (want 1) dbe=%b (want 0)", ecc_sbe, ecc_dbe);
            errors = errors + 1;
        end

        // Double-bit corruption: must report dbe instead.
        dut.l2_data[0][100] = ~dut.l2_data[0][100]; // repair
        dut.l2_data[0][10] = ~dut.l2_data[0][10];
        dut.l2_data[0][60] = ~dut.l2_data[0][60];
        do_busrd(64'h2000);
        if (ecc_dbe !== 1'b1) begin
            $display("[FAIL] double-bit L2 corruption: dbe=%b (want 1)", ecc_dbe);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("[PASS] tb_ecc_l2: all checks passed");
        else
            $display("[FAIL] tb_ecc_l2: %0d error(s)", errors);
        $finish;
    end
endmodule
