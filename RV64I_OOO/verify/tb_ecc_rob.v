`timescale 1ns / 1ps
// Isolated unit test for rob.v's Phase 9 ECC integration: same
// direct-drive style as tb_rob_rat.v (no fetch/decode/RS involved).
// Allocates one entry, marks it done through the real mark_valid port
// (so value_check[] gets real, correctly-computed check bits, not a
// hand-poked value), then hierarchically corrupts value_arr[]/
// value_check[] directly and confirms head_value is transparently
// corrected (with ecc_rob_sbe_fault) or, for a second corruption,
// ecc_rob_dbe_fault fires instead.
module tb_ecc_rob;
    localparam DEPTH = 8;
    localparam TB = 3;

    reg clk, reset;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg alloc_req;
    wire [TB-1:0] alloc_tag;
    reg mark_valid;
    reg [TB-1:0] mark_tag;
    reg [63:0] mark_value;
    wire head_ready;
    wire [63:0] head_value;
    reg commit_req;
    wire ecc_sbe, ecc_dbe;

    rob #(.DEPTH(DEPTH), .EXTRA_MARK_N(4)) dut (
        .clk(clk), .reset(reset),
        .alloc_req(alloc_req), .alloc_has_dest(1'b1), .alloc_rd(5'd5),
        .alloc_is_store(1'b0), .alloc_is_ecall(1'b0), .alloc_is_vec_dest(1'b0),
        .alloc_tag(alloc_tag), .full(), .empty(),
        .alloc2_req(1'b0), .alloc2_has_dest(1'b0), .alloc2_rd(5'b0),
        .alloc2_is_store(1'b0), .alloc2_is_ecall(1'b0), .alloc2_is_vec_dest(1'b0),
        .alloc2_tag(), .free_count(),
        .mark_valid(mark_valid), .mark_tag(mark_tag), .mark_value(mark_value),
        .mark_b_valid(1'b0), .mark_b_tag({TB{1'b0}}), .mark_b_value(64'b0),
        .vec_mark_valid(1'b0), .vec_mark_tag({TB{1'b0}}), .vec_mark_value(128'b0),
        .mark2_valid(1'b0), .mark2_tag({TB{1'b0}}),
        .extra_mark_valid(4'b0), .extra_mark_tag_flat(12'b0),
        .squash_valid(1'b0), .squash_tag({TB{1'b0}}),
        .lookup1_tag({TB{1'b0}}), .lookup1_done(), .lookup1_value(),
        .lookup2_tag({TB{1'b0}}), .lookup2_done(), .lookup2_value(),
        .lookup3_tag({TB{1'b0}}), .lookup3_done(), .lookup3_value(),
        .lookup4_tag({TB{1'b0}}), .lookup4_done(), .lookup4_value(),
        .vec_lookup1_tag({TB{1'b0}}), .vec_lookup1_done(), .vec_lookup1_value(),
        .vec_lookup2_tag({TB{1'b0}}), .vec_lookup2_done(), .vec_lookup2_value(),
        .head_ready(head_ready), .head_tag(), .head_has_dest(),
        .head_rd(), .head_value(head_value),
        .head_vec_value(), .head_is_vec_dest(),
        .head_is_store(), .head_is_ecall(),
        .commit_req(commit_req),
        .head2_ready(), .head2_tag(), .head2_has_dest(), .head2_rd(), .head2_value(),
        .head2_vec_value(), .head2_is_vec_dest(),
        .head2_is_store(), .head2_is_ecall(), .commit_req2(1'b0),
        .ecc_rob_sbe_fault(ecc_sbe), .ecc_rob_dbe_fault(ecc_dbe)
    );

    integer errors;
    reg [TB-1:0] my_tag;
    initial begin
        errors = 0;
        reset = 1; alloc_req = 0; mark_valid = 0; mark_tag = 0; mark_value = 0; commit_req = 0;
        @(posedge clk); #1; @(posedge clk); #1;
        reset = 0;

        // Allocate one entry, then mark it done through the real
        // mark_valid port -- real, correctly-computed check bits land in
        // value_check[0], not a hand-poked value. Same
        // sample-alloc_tag-*before*-the-clock-edge-that-advances-tail_ptr
        // convention as tb_rob_rat.v's own do_alloc task (see its header
        // comment for why sampling after the edge reads the wrong tag).
        alloc_req = 1;
        #1;
        my_tag = alloc_tag;
        @(posedge clk); #1;
        alloc_req = 0;

        mark_valid = 1; mark_tag = my_tag; mark_value = 64'h1122334455667788;
        @(posedge clk); #1;
        mark_valid = 0;

        if (head_ready !== 1'b1 || head_value !== 64'h1122334455667788 || ecc_sbe !== 1'b0 || ecc_dbe !== 1'b0) begin
            $display("[FAIL] clean mark/head: ready=%b value=%h sbe=%b dbe=%b", head_ready, head_value, ecc_sbe, ecc_dbe);
            errors = errors + 1;
        end

        // Single-bit corruption directly in the DUT's own value_arr[0]
        // (the head entry's slot).
        dut.value_arr[0][30] = ~dut.value_arr[0][30];
        #1;
        if (head_value !== 64'h1122334455667788 || ecc_sbe !== 1'b1 || ecc_dbe !== 1'b0) begin
            $display("[FAIL] single-bit corruption: value=%h (want 1122334455667788) sbe=%b (want 1) dbe=%b (want 0)",
                      head_value, ecc_sbe, ecc_dbe);
            errors = errors + 1;
        end

        // Double-bit corruption: dbe instead, no silent miscorrection.
        dut.value_arr[0][30] = ~dut.value_arr[0][30]; // repair
        dut.value_arr[0][3]  = ~dut.value_arr[0][3];
        dut.value_arr[0][45] = ~dut.value_arr[0][45];
        #1;
        if (ecc_dbe !== 1'b1) begin
            $display("[FAIL] double-bit corruption: dbe=%b (want 1)", ecc_dbe);
            errors = errors + 1;
        end

        // head_ready must go low (and stop consuming these values, so no
        // more fault flags) once the entry actually commits.
        dut.value_arr[0][3] = ~dut.value_arr[0][3];
        dut.value_arr[0][45] = ~dut.value_arr[0][45]; // fully repaired
        commit_req = 1;
        @(posedge clk); #1;
        commit_req = 0;
        if (head_ready !== 1'b0) begin
            $display("[FAIL] head_ready after commit: %b (want 0)", head_ready);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("[PASS] tb_ecc_rob: all checks passed");
        else
            $display("[FAIL] tb_ecc_rob: %0d error(s)", errors);
        $finish;
    end
endmodule
