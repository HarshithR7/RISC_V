`timescale 1ns/1ps
// Unit-level testbench for icache.v in isolation -- direct-drive style
// matching tb_rob_rat.v. Backing content (icache_test.mem) is a small,
// self-describing fixture: the 32-bit word at byte address A holds the
// value A itself, so a correct read at any address is trivially checked
// against the address it was fetched from, with no real instruction
// decoding needed.
module tb_icache;
    localparam LINES = 4;          // small on purpose: index bits=2, so
                                    // byte addresses 256 apart alias the
                                    // same set with different tags --
                                    // lets the eviction test collide two
                                    // real, distinct lines deliberately.
    localparam LINE_BYTES = 32;
    localparam MISS_LATENCY = 2;
    localparam IMEM_WORDS = 128;   // halfwords -- see instruction_fetch.v

    reg clk, reset;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg [63:0] pc, pc1, pc2;
    wire [31:0] instruction0, instruction1, instruction2;
    wire stall;

    icache #(
        .IMEM_FILE("icache_test.mem"), .IMEM_WORDS(IMEM_WORDS),
        .LINES(LINES), .LINE_BYTES(LINE_BYTES), .MISS_LATENCY(MISS_LATENCY)
    ) dut (
        .clk(clk), .reset(reset),
        .pc(pc), .pc1(pc1), .pc2(pc2),
        .instruction0(instruction0), .instruction1(instruction1), .instruction2(instruction2),
        .stall(stall),
        .axi_wr_en(1'b0), .axi_wr_addr({$clog2(IMEM_WORDS){1'b0}}), .axi_wr_data(16'b0)
    );

    integer errors = 0;
    task check(input [255:0] name, input [63:0] got, input [63:0] exp);
        begin
            if (got !== exp) begin
                $display("[FAIL] %0s: got=%0d exp=%0d", name, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    // Drives pc/pc1/pc2 for a group starting at `base`, waits until
    // `stall` deasserts (bounded), then checks all 3 lanes' data against
    // the fixture's self-describing content.
    integer wait_cycles;
    task fetch_group(input [63:0] base, input integer max_wait);
        begin
            pc = base; pc1 = base + 4; pc2 = base + 8;
            @(posedge clk); #1;
            wait_cycles = 0;
            while (stall && wait_cycles < max_wait) begin
                @(posedge clk); #1;
                wait_cycles = wait_cycles + 1;
            end
        end
    endtask

    initial begin
        reset = 1; pc = 0; pc1 = 4; pc2 = 8;
        @(posedge clk); #1; @(posedge clk); #1;
        reset = 0;

        // ---- T1: cold miss + fill, group entirely within line 0 --------
        fetch_group(64'd0, 40);
        check("T1 stall cleared", stall, 1'b0);
        check("T1 instr0", instruction0, 64'd0);
        check("T1 instr1", instruction1, 64'd4);
        check("T1 instr2", instruction2, 64'd8);

        // ---- T2: warm repeated hit, exactly 1-cycle latency -------------
        // (already resident from T1 -- fetch_group should see stall=0
        // immediately, i.e. wait_cycles stays 0.)
        fetch_group(64'd0, 40);
        check("T2 stall cleared immediately", (wait_cycles == 0), 1'b1);
        check("T2 instr0", instruction0, 64'd0);

        // ---- T3: still-within-line-0 different offset, warm hit --------
        fetch_group(64'd12, 40);
        check("T3 stall cleared immediately", (wait_cycles == 0), 1'b1);
        check("T3 instr0", instruction0, 64'd12);
        check("T3 instr1", instruction1, 64'd16);
        check("T3 instr2", instruction2, 64'd20);

        // ---- T4: line-boundary crossing, line(pc) already resident -----
        // (line 0, from T1/T2/T3) but line(pc2) is NOT yet -- pc's in-line
        // offset is 28 (the last word of a 32-byte line), so pc1=32 and
        // pc2=36 both land in line 1, still unfilled. Exercises the
        // "hit_pc true but group_hit false -> line(pc2) is the missing
        // one" branch.
        fetch_group(64'd28, 40);
        check("T4 stall cleared", stall, 1'b0);
        check("T4 instr0", instruction0, 64'd28);
        check("T4 instr1", instruction1, 64'd32);
        check("T4 instr2", instruction2, 64'd36);

        // ---- T5: line-boundary crossing, NEITHER line resident yet -----
        // pc=92 (line 2 offset 28), pc1/pc2 spill into line 3. Both lines
        // are cold -- exercises the two-line-fill (pending_b) path from
        // scratch.
        fetch_group(64'd92, 60);
        check("T5 stall cleared", stall, 1'b0);
        check("T5 instr0", instruction0, 64'd92);
        check("T5 instr1", instruction1, 64'd96);
        check("T5 instr2", instruction2, 64'd100);

        // Re-fetch T5's group: should now be fully warm (both lines
        // resident), no wait.
        fetch_group(64'd92, 40);
        check("T5b stall cleared immediately", (wait_cycles == 0), 1'b1);

        // ---- T6: direct-mapped eviction/overwrite -----------------------
        // Byte 128 aliases the same set (index 0) as byte 0 (LINES=4,
        // LINE_BYTES=32 -> index = addr[6:5], so a 128-byte stride keeps
        // the same index bits while incrementing the tag) -- fetching it
        // must evict line 0's entry.
        fetch_group(64'd128, 40);
        check("T6 stall cleared", stall, 1'b0);
        check("T6 instr0", instruction0, 64'd128);

        // Re-fetching byte 0 must now MISS again (its set was evicted by
        // T6) -- confirmed by requiring a real wait, not an instant hit.
        fetch_group(64'd0, 40);
        check("T6b evicted line re-misses (real wait observed)", (wait_cycles > 0), 1'b1);
        check("T6b instr0 still correct after refill", instruction0, 64'd0);

        if (errors == 0)
            $display("[PASS] tb_icache: all checks passed");
        else
            $display("[FAIL] tb_icache: %0d error(s)", errors);
        $finish;
    end
endmodule
