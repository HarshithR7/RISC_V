`timescale 1ns / 1ps
// Isolated unit test for lsq.v's Phase 10 merging write buffer: direct-
// drive style, same alloc/commit idiom as tb_ecc_rob.v (no fetch/
// decode/RS/ROB involved -- commit_lookup_tid/tag and commit_fire are
// driven directly, exactly like a real ROB retirement would). l1_busy
// is tied permanently high so the buffer never actually drains during
// the test, letting the test inspect sbuf_valid[]/sbuf_data[] directly.
//
// Two stores are allocated to the exact same address and width (same
// base+imm, same func3) but with different data, then committed one
// after another. The second commit must collapse into the FIRST one's
// buffer slot (no second slot consumed) with the second store's -- the
// newer, correct -- value, not silently duplicate the entry or lose the
// update.
module tb_merge_wbuf;
    localparam DEPTH = 4;
    localparam TB = 3;
    localparam SBUF_DEPTH = 4;

    reg clk, reset;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg alloc_req;
    reg alloc_tid, alloc_is_store;
    reg [2:0] alloc_func3;
    reg [63:0] alloc_imm;
    reg alloc_base_ready; reg [63:0] alloc_base_val; reg [TB-1:0] alloc_base_tag;
    reg alloc_data_ready; reg [63:0] alloc_data_val; reg [TB-1:0] alloc_data_tag;
    reg [TB-1:0] alloc_dest_tag;
    wire full;

    reg commit_lookup_tid;
    reg [TB-1:0] commit_lookup_tag;
    reg commit_fire;
    wire commit_match, store_buffer_full;

    wire l1_write_req; wire [63:0] l1_write_addr, l1_write_data; wire [2:0] l1_write_func3;

    lsq #(.DEPTH(DEPTH), .TAG_BITS(TB), .SBUF_DEPTH(SBUF_DEPTH)) dut (
        .clk(clk), .reset(reset),
        .alloc_req(alloc_req), .alloc_tid(alloc_tid), .alloc_is_store(alloc_is_store),
        .alloc_func3(alloc_func3), .alloc_imm(alloc_imm),
        .alloc_base_ready(alloc_base_ready), .alloc_base_val(alloc_base_val), .alloc_base_tag(alloc_base_tag),
        .alloc_data_ready(alloc_data_ready), .alloc_data_val(alloc_data_val), .alloc_data_tag(alloc_data_tag),
        .alloc_dest_tag(alloc_dest_tag),
        .full(full),
        .alloc2_req(1'b0), .alloc2_tid(1'b0), .alloc2_is_store(1'b0),
        .alloc2_func3(3'b0), .alloc2_imm(64'b0),
        .alloc2_base_ready(1'b0), .alloc2_base_val(64'b0), .alloc2_base_tag({TB{1'b0}}),
        .alloc2_data_ready(1'b0), .alloc2_data_val(64'b0), .alloc2_data_tag({TB{1'b0}}),
        .alloc2_dest_tag({TB{1'b0}}),
        .has_2_free(),
        .cdbA_valid(1'b0), .cdbA_tid(1'b0), .cdbA_tag({TB{1'b0}}), .cdbA_value(64'b0),
        .cdbB_valid(1'b0), .cdbB_tid(1'b0), .cdbB_tag({TB{1'b0}}), .cdbB_value(64'b0),
        .rob_head_tag0({TB{1'b0}}), .rob_head_tag1({TB{1'b0}}),
        .commit_lookup_tid(commit_lookup_tid), .commit_lookup_tag(commit_lookup_tag),
        .req_valid(), .req_tid(), .req_tag(), .req_value(), .req_grant(1'b0),
        .l1_read_req(), .l1_read_addr(), .l1_read_func3(),
        .l1_read_valid(1'b0), .l1_read_data(64'b0),
        .l1_write_req(l1_write_req), .l1_write_addr(l1_write_addr), .l1_write_data(l1_write_data),
        .l1_write_func3(l1_write_func3), .l1_write_done(1'b0), .l1_busy(1'b1),
        .l1_read2_req(), .l1_read2_addr(), .l1_read2_func3(),
        .l1_read2_hit(1'b0), .l1_read2_data(64'b0),
        .store_ready(), .store_ready_tag_flat(), .store_ready_tid_flat(),
        .commit_match(commit_match), .commit_fire(commit_fire), .store_buffer_full(store_buffer_full),
        .squash0_valid(1'b0), .squash0_tag({TB{1'b0}}),
        .squash1_valid(1'b0), .squash1_tag({TB{1'b0}})
    );

    integer errors;

    task do_alloc_store;
        input [TB-1:0] tag; input [63:0] data;
        begin
            alloc_req = 1; alloc_tid = 1'b0; alloc_is_store = 1'b1;
            alloc_func3 = 3'b011; // SD
            alloc_imm = 64'h0;
            alloc_base_ready = 1'b1; alloc_base_val = 64'h2000; alloc_base_tag = {TB{1'b0}};
            alloc_data_ready = 1'b1; alloc_data_val = data; alloc_data_tag = {TB{1'b0}};
            alloc_dest_tag = tag;
            #1;
            @(posedge clk); #1;
            alloc_req = 0;
        end
    endtask

    task do_commit;
        input [TB-1:0] tag;
        begin
            commit_lookup_tid = 1'b0; commit_lookup_tag = tag;
            #1;
            commit_fire = 1'b1;
            @(posedge clk); #1;
            commit_fire = 1'b0;
        end
    endtask

    initial begin
        errors = 0;
        alloc_req = 0; alloc_tid = 0; alloc_is_store = 0; alloc_func3 = 0; alloc_imm = 0;
        alloc_base_ready = 0; alloc_base_val = 0; alloc_base_tag = 0;
        alloc_data_ready = 0; alloc_data_val = 0; alloc_data_tag = 0; alloc_dest_tag = 0;
        commit_lookup_tid = 0; commit_lookup_tag = 0; commit_fire = 0;
        reset = 1;
        @(posedge clk); #1; @(posedge clk); #1;
        reset = 0;

        // Two stores to the exact same address+width, different data,
        // different dest tags (as two real, distinct instructions would
        // have).
        do_alloc_store(3'd1, 64'hAAAAAAAAAAAAAAAA);
        do_alloc_store(3'd2, 64'hBBBBBBBBBBBBBBBB);

        // Commit the first: buffer starts empty, so this must allocate a
        // fresh slot (slot 0, the lowest free index).
        do_commit(3'd1);
        if (!dut.sbuf_valid[0] || dut.sbuf_data[0] !== 64'hAAAAAAAAAAAAAAAA) begin
            $display("[FAIL] first store didn't land in sbuf[0]: valid=%b data=%h",
                      dut.sbuf_valid[0], dut.sbuf_data[0]);
            errors = errors + 1;
        end
        if (dut.sbuf_valid[1]) begin
            $display("[FAIL] first store unexpectedly also touched sbuf[1]");
            errors = errors + 1;
        end

        // Commit the second: same address+width, l1_busy is permanently
        // 1 so the first entry can never have started draining -- this
        // MUST merge into sbuf[0] rather than consume a second slot.
        do_commit(3'd2);
        if (!dut.sbuf_valid[0] || dut.sbuf_data[0] !== 64'hBBBBBBBBBBBBBBBB) begin
            $display("[FAIL] second (newer) store's value didn't land in sbuf[0]: valid=%b data=%h (want BBBB...)",
                      dut.sbuf_valid[0], dut.sbuf_data[0]);
            errors = errors + 1;
        end
        if (dut.sbuf_valid[1]) begin
            $display("[FAIL] second store consumed a NEW slot (sbuf[1]) instead of merging -- not a merging write buffer");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("[PASS] tb_merge_wbuf: all checks passed");
        else
            $display("[FAIL] tb_merge_wbuf: %0d error(s)", errors);
        $finish;
    end
endmodule
