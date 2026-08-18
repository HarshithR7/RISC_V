`timescale 1ns/1ps
// Unit-level testbench for rob.v + rat.v in isolation -- no fetch/decode/
// RS/FU involved, manually driving dispatch/mark/commit exactly like a
// real dispatch_ooo.v/commit.v would, to prove the commit-order renaming
// discipline directly before it's buried inside a full pipeline. Same
// "isolate the riskiest new logic first" spirit as tb_div_fu.v and
// RV64I/verify/tb_fpu_unit.v.
module tb_rob_rat;
    localparam DEPTH = 8;
    localparam TB = 3; // $clog2(DEPTH)

    reg clk, reset;
    integer errors = 0;

    // ---- ROB ----
    reg alloc_req, alloc_has_dest, alloc_is_store, alloc_is_ecall;
    reg [4:0] alloc_rd;
    wire [TB-1:0] alloc_tag;
    wire full, empty;
    reg mark_valid;
    reg [TB-1:0] mark_tag;
    reg [63:0] mark_value;
    wire head_ready;
    wire [TB-1:0] head_tag;
    wire head_has_dest;
    wire [4:0] head_rd;
    wire [63:0] head_value;
    wire head_is_store, head_is_ecall;
    reg commit_req;

    rob #(.DEPTH(DEPTH), .EXTRA_MARK_N(4)) rob_dut (
        .clk(clk), .reset(reset),
        .alloc_req(alloc_req), .alloc_has_dest(alloc_has_dest), .alloc_rd(alloc_rd),
        .alloc_is_store(alloc_is_store), .alloc_is_ecall(alloc_is_ecall),
        .alloc_is_vec_dest(1'b0),
        .alloc_tag(alloc_tag), .full(full), .empty(empty),
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
        .head_ready(head_ready), .head_tag(head_tag), .head_has_dest(head_has_dest),
        .head_rd(head_rd), .head_value(head_value),
        .head_vec_value(), .head_is_vec_dest(),
        .head_is_store(head_is_store), .head_is_ecall(head_is_ecall),
        .commit_req(commit_req),
        .head2_ready(), .head2_tag(), .head2_has_dest(), .head2_rd(), .head2_value(),
        .head2_vec_value(), .head2_is_vec_dest(),
        .head2_is_store(), .head2_is_ecall(), .commit_req2(1'b0)
    );

    // ---- RAT ----
    reg [4:0] rs1, rs2;
    wire rs1_busy, rs2_busy;
    wire [TB-1:0] rs1_tag, rs2_tag;
    reg write_en;
    reg [4:0] wr_rd;
    reg [TB-1:0] wr_tag;
    reg commit_clear_en;
    reg [4:0] commit_rd;
    reg [TB-1:0] commit_tag;

    rat #(.TAG_BITS(TB)) rat_dut (
        .clk(clk), .reset(reset),
        .rs1(rs1), .rs2(rs2),
        .rs1_busy(rs1_busy), .rs1_tag(rs1_tag),
        .rs2_busy(rs2_busy), .rs2_tag(rs2_tag),
        .rs1b(5'b0), .rs2b(5'b0),
        .rs1b_busy(), .rs1b_tag(), .rs2b_busy(), .rs2b_tag(),
        .write_en(write_en), .rd(wr_rd), .new_tag(wr_tag),
        .write2_en(1'b0), .rd2(5'b0), .new_tag2({TB{1'b0}}),
        .commit_clear_en(commit_clear_en), .commit_rd(commit_rd), .commit_tag(commit_tag),
        .commit_clear_en2(1'b0), .commit_rd2(5'b0), .commit_tag2({TB{1'b0}}),
        .checkpoint_save(1'b0), .checkpoint_restore(1'b0)
    );

    initial begin clk = 0; forever #5 clk = ~clk; end

    task check(input [1023:0] name, input got, input exp);
        begin
            if (got !== exp) begin
                $display("FAIL %0s: got=%0d exp=%0d", name, got, exp);
                errors = errors + 1;
            end else $display("PASS %0s", name);
        end
    endtask

    // Correctly-timed dispatch helper: alloc_tag is combinational
    // (=tail_ptr), so it must be sampled *before* the clock edge that
    // actually advances tail_ptr, not after -- sampling after the edge
    // reads the *next* instruction's tag instead of this one's (a real
    // bug this testbench's first draft made, caught by every single
    // mark/commit check downstream failing with head_ready staying 0).
    task do_alloc(input has_dest, input [4:0] rd, input is_store, input is_ecall,
                  output [TB-1:0] tag);
        begin
            alloc_req = 1; alloc_has_dest = has_dest; alloc_rd = rd;
            alloc_is_store = is_store; alloc_is_ecall = is_ecall;
            #1;
            tag = alloc_tag;
            @(posedge clk); #1;
            alloc_req = 0;
        end
    endtask

    task do_mark(input [TB-1:0] tag, input [63:0] value);
        begin
            mark_valid = 1; mark_tag = tag; mark_value = value;
            @(posedge clk); #1;
            mark_valid = 0;
        end
    endtask

    task do_rename(input [4:0] rd, input [TB-1:0] tag);
        begin
            write_en = 1; wr_rd = rd; wr_tag = tag;
            @(posedge clk); #1;
            write_en = 0;
        end
    endtask

    task do_commit(input [4:0] rd, input [TB-1:0] tag);
        begin
            commit_req = 1; commit_clear_en = 1; commit_rd = rd; commit_tag = tag;
            @(posedge clk); #1;
            commit_req = 0; commit_clear_en = 0;
        end
    endtask

    task get_reg_busy(input [4:0] r, output result);
        begin
            rs2 = r; #1; result = rs2_busy; rs2 = 0;
        end
    endtask
    task get_reg_tag(input [4:0] r, output [TB-1:0] result);
        begin
            rs2 = r; #1; result = rs2_tag; rs2 = 0;
        end
    endtask

    // One clean idle cycle with every control input deasserted, so tests
    // never leak an asserted signal into the next one.
    task idle_cycle;
        begin
            alloc_req = 0; mark_valid = 0; commit_req = 0;
            write_en = 0; commit_clear_en = 0;
            @(posedge clk); #1;
        end
    endtask

    reg [TB-1:0] t1, t2;
    reg probe_busy;
    reg [TB-1:0] probe_tag;

    initial begin
        reset = 1;
        alloc_req=0; alloc_has_dest=0; alloc_rd=0; alloc_is_store=0; alloc_is_ecall=0;
        mark_valid=0; mark_tag=0; mark_value=0; commit_req=0;
        rs1=0; rs2=0; write_en=0; wr_rd=0; wr_tag=0;
        commit_clear_en=0; commit_rd=0; commit_tag=0;
        @(posedge clk); @(posedge clk); #1;
        reset = 0;
        idle_cycle;

        // ==== Test 1: RAW -- I1 (rd=x5) then I2 (rd=x6, rs1=x5) ====
        do_alloc(1, 5'd5, 0, 0, t1);         // I1 dispatches, gets tag t1
        do_rename(5'd5, t1);                  // I1's rename lands
        rs1 = 5'd5;                           // I2's rs1 read, combinational
        #1;
        check("T1: x5 busy when I2 reads rs1", rs1_busy, 1'b1);
        check("T1: x5 tag == I1's tag when I2 reads rs1", (rs1_tag == t1), 1'b1);
        rs1 = 0;
        do_alloc(1, 5'd6, 0, 0, t2);           // I2 dispatches, gets tag t2
        do_rename(5'd6, t2);                   // I2's rename lands
        do_mark(t1, 64'd100);                   // I1's result broadcasts
        check("T1: ROB head ready after mark", head_ready, 1'b1);
        check("T1: ROB head value == 100", (head_value == 64'd100), 1'b1);
        check("T1: ROB head rd == x5", (head_rd == 5'd5), 1'b1);
        do_commit(5'd5, t1);
        get_reg_busy(5'd5, probe_busy);
        check("T1: x5 no longer busy after commit", probe_busy, 1'b0);
        // Drain I2 too -- every test must fully drain what it allocates,
        // or a stuck entry silently corrupts the next test's assumption
        // that the ROB starts empty (exactly what happened here on the
        // first draft: I2 was left marked-but-uncommitted, and Test 2's
        // checks read stale/undefined head data as a result).
        do_mark(t2, 64'd200);
        do_commit(5'd6, t2);
        check("T1: ROB empty after draining both I1 and I2", empty, 1'b1);
        idle_cycle;

        // ==== Test 2: WAW pair I1(rd=x3) then I2(rd=x3), different
        // cycles, verifying I1's commit does NOT clear a RAT entry I2
        // has since remapped ====
        do_alloc(1, 5'd3, 0, 0, t1);
        do_rename(5'd3, t1);
        do_alloc(1, 5'd3, 0, 0, t2);           // I2, a later cycle, same rd
        do_rename(5'd3, t2);
        get_reg_tag(5'd3, probe_tag);
        check("T2: x3 tag is I2's tag after both dispatch", (probe_tag == t2), 1'b1);
        do_mark(t1, 64'd11);
        do_mark(t2, 64'd22);
        check("T2: ROB head is I1 (rd=x3, val=11)", (head_value == 64'd11), 1'b1);
        do_commit(5'd3, t1);                    // I1 commits first (in-order)
        get_reg_busy(5'd3, probe_busy);
        check("T2: x3 STILL busy after I1 commits (I2 not clobbered)", probe_busy, 1'b1);
        get_reg_tag(5'd3, probe_tag);
        check("T2: x3 tag still I2's after I1 commits", (probe_tag == t2), 1'b1);
        check("T2: ROB head now I2 (rd=x3, val=22)", (head_value == 64'd22), 1'b1);
        do_commit(5'd3, t2);                    // I2 commits second
        get_reg_busy(5'd3, probe_busy);
        check("T2: x3 no longer busy after I2 commits", probe_busy, 1'b0);
        idle_cycle;

        // ==== Test 3: x0 exclusion -- rd=0 must never be renamed, and
        // must never crash indexing the RAT's [1:31] arrays ====
        do_alloc(1, 5'd0, 0, 0, t1);            // e.g. addi x0, x1, 5
        do_rename(5'd0, t1);
        do_mark(t1, 64'd999);
        do_commit(5'd0, t1);
        get_reg_busy(5'd1, probe_busy);
        check("T3: x0 path completes without corrupting x1's RAT state", probe_busy, 1'b0);
        idle_cycle;

        // ==== Test 4: ROB-full stall -- fill all DEPTH entries, verify a
        // further alloc_req is ignored (doesn't corrupt entry 0 via
        // tail_ptr wraparound) ====
        begin : fill_rob
            integer k;
            reg [TB-1:0] discard_tag;
            for (k = 0; k < DEPTH; k = k + 1)
                do_alloc(1, k[4:0] + 5'd1, 0, 0, discard_tag);
        end
        check("T4: ROB full after DEPTH allocations", full, 1'b1);
        // Attempt one more allocation while full -- must be a no-op.
        alloc_req = 1; alloc_has_dest = 1; alloc_rd = 5'd31;
        @(posedge clk); #1;
        alloc_req = 0;
        check("T4: ROB still full, extra alloc had no effect", full, 1'b1);
        // Drain and verify entry 0's data wasn't corrupted by the
        // rejected 9th allocation (would show up as head_rd != 1, the
        // rd this test originally allocated into entry 0). Must mark the
        // *actual* head_tag, not a hardcoded 0 -- ROB tags keep advancing
        // across the whole testbench, they don't reset to 0 per test (a
        // real bug the first draft had: marking tag 0 literally, when
        // head_tag was actually 5 by this point, left head_ready false
        // and made the following commit_req a silent no-op).
        do_mark(head_tag, 64'd0);
        check("T4: head_rd after drain-mark is the original entry 0's rd (x1)", (head_rd == 5'd1), 1'b1);
        commit_req = 1;
        @(posedge clk); #1;
        commit_req = 0;
        check("T4: ROB no longer full after one commit", full, 1'b0);
        // Drain the rest without checking individually -- just to leave
        // the ROB empty for the next test.
        begin : drain_rest
            integer k;
            reg [TB-1:0] ht;
            for (k = 0; k < DEPTH - 1; k = k + 1) begin
                ht = head_tag;
                do_mark(ht, 64'd0);
                commit_req = 1;
                @(posedge clk); #1;
                commit_req = 0;
            end
        end
        check("T4: ROB empty after full drain", empty, 1'b1);
        idle_cycle;

        // ==== Test 5: same-cycle race -- I1 (rd=x7) commits the exact
        // same cycle a brand-new, younger instruction J (rd=x7) dispatches
        // and re-renames x7. J's rename must win (it's younger). ====
        do_alloc(1, 5'd7, 0, 0, t1);
        do_rename(5'd7, t1);
        do_mark(t1, 64'd77);
        check("T5: I1 (x7) at ROB head, ready", head_ready, 1'b1);
        // Same cycle: commit I1 (RAT commit_clear_en x7,t1) AND dispatch J
        // (rd=x7, new tag) with its RAT write_en asserted together.
        commit_req = 1; commit_clear_en = 1; commit_rd = 5'd7; commit_tag = t1;
        alloc_req = 1; alloc_has_dest = 1; alloc_rd = 5'd7; // J
        #1;
        t2 = alloc_tag; // J's tag, sampled combinationally before the edge
        write_en = 1; wr_rd = 5'd7; wr_tag = t2;
        @(posedge clk); #1;
        commit_req = 0; commit_clear_en = 0; alloc_req = 0; write_en = 0;
        get_reg_busy(5'd7, probe_busy);
        check("T5: x7 busy after the race cycle (J's rename won, not cleared)", probe_busy, 1'b1);
        get_reg_tag(5'd7, probe_tag);
        check("T5: x7 tag == J's tag after the race cycle", (probe_tag == t2), 1'b1);
        idle_cycle;

        if (errors == 0) $display("tb_rob_rat: ALL PASS");
        else $display("tb_rob_rat: %0d FAILURES", errors);
        $finish;
    end
endmodule
