`timescale 1ns/1ps
// Unit-level testbench for btb.v in isolation -- direct-drive style
// matching tb_rob_rat.v/tb_icache.v.
module tb_btb;
    localparam INDEX_BITS = 6; // 64 entries, matches bht.v/the default

    reg clk, reset;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg [63:0] predict_pc;
    wire predict_valid;
    wire [63:0] predict_target;
    reg update_valid;
    reg [63:0] update_pc, update_target;

    btb #(.INDEX_BITS(INDEX_BITS)) dut (
        .clk(clk), .reset(reset),
        .predict_pc(predict_pc), .predict_valid(predict_valid), .predict_target(predict_target),
        .update_valid(update_valid), .update_pc(update_pc), .update_target(update_target)
    );

    integer errors = 0;
    task check(input [255:0] name, input got, input exp);
        begin
            if (got !== exp) begin
                $display("[FAIL] %0s: got=%b exp=%b", name, got, exp);
                errors = errors + 1;
            end
        end
    endtask
    task check64(input [255:0] name, input [63:0] got, input [63:0] exp);
        begin
            if (got !== exp) begin
                $display("[FAIL] %0s: got=%h exp=%h", name, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        reset = 1; predict_pc = 0; update_valid = 0; update_pc = 0; update_target = 0;
        @(posedge clk); #1; @(posedge clk); #1;
        reset = 0;

        // ---- T1: cold miss on an unseen PC ------------------------------
        predict_pc = 64'h1000;
        #1;
        check("T1 cold miss", predict_valid, 1'b0);

        // ---- T2: train, then hit for the same PC ------------------------
        update_valid = 1; update_pc = 64'h1000; update_target = 64'h2000;
        @(posedge clk); #1;
        update_valid = 0;
        predict_pc = 64'h1000;
        #1;
        check("T2 predict_valid after train", predict_valid, 1'b1);
        check64("T2 predict_target after train", predict_target, 64'h2000);

        // ---- T3: retrain the same PC with a different target ------------
        // (a real call site whose target genuinely changed -- confirms no
        // stale-target leakage, matches an indirect-call BTB-misprediction
        // scenario's training step.)
        update_valid = 1; update_pc = 64'h1000; update_target = 64'h3000;
        @(posedge clk); #1;
        update_valid = 0;
        predict_pc = 64'h1000;
        #1;
        check("T3 predict_target after retrain", predict_target, 64'h3000);

        // ---- T4: a different, still-unseen PC misses independently ------
        // (a different index, not just a different address -- 0x1000 and
        // 0x2010 differ in predict_pc[7:2], unlike the T5 pair below which
        // deliberately shares an index to test aliasing.)
        predict_pc = 64'h2010;
        #1;
        check("T4 unrelated PC still misses", predict_valid, 1'b0);

        // ---- T5: aliasing -- two PCs sharing the same index; training ---
        // the 2nd overwrites the 1st's slot (documented, expected, not a
        // bug -- see btb.v's header).
        predict_pc = 64'h1000;
        #1;
        check("T5 pre: original PC still valid", predict_valid, 1'b1);
        update_valid = 1;
        update_pc = 64'h1000 + (64'h1 << (INDEX_BITS + 2)); // same index bits, different tag
        update_target = 64'h5000;
        @(posedge clk); #1;
        update_valid = 0;
        predict_pc = 64'h1000;
        #1;
        check("T5 aliasing PC now reads the aliased entry's target", predict_target, 64'h5000);

        // ---- T6: reset clears every entry --------------------------------
        reset = 1;
        @(posedge clk); #1; @(posedge clk); #1;
        reset = 0;
        predict_pc = 64'h1000;
        #1;
        check("T6 reset clears trained entries", predict_valid, 1'b0);

        if (errors == 0)
            $display("[PASS] tb_btb: all checks passed");
        else
            $display("[FAIL] tb_btb: %0d error(s)", errors);
        $finish;
    end
endmodule
