`timescale 1ns/1ps
// Unit-level testbench for div_fu.v in isolation -- no ROB/RS/decode
// involved, same "fast to iterate on the riskiest new logic first" spirit
// as RV64I/verify/tb_fpu_unit.v. Expected quotient/remainder values are
// computed independently by hand (truncating-toward-zero signed division,
// matching RV64I's existing scalar M-extension and RVV vdiv/vrem
// convention -- not floor division), not copied from the RTL.
module tb_div_fu;
    reg clk, reset;
    reg start, is_signed, is_word;
    reg [63:0] dividend, divisor;
    wire busy, done;
    wire [63:0] quotient, remainder;
    integer errors = 0;
    integer cyc_count;

    div_fu dut (
        .clk(clk), .reset(reset), .start(start),
        .is_signed(is_signed), .is_word(is_word),
        .dividend(dividend), .divisor(divisor),
        .busy(busy), .done(done),
        .quotient(quotient), .remainder(remainder)
    );

    initial begin clk = 0; forever #5 clk = ~clk; end

    task run_div(input [255:0] name, input sgn, input wd, input [63:0] a, input [63:0] b,
                 input [63:0] exp_q, input [63:0] exp_r);
        begin
            @(negedge clk);
            is_signed = sgn; is_word = wd; dividend = a; divisor = b; start = 1;
            @(negedge clk);
            start = 0;
            cyc_count = 0;
            while (!done) begin
                @(negedge clk);
                cyc_count = cyc_count + 1;
                if (cyc_count > 200) begin
                    $display("FAIL %0s: timed out waiting for done", name);
                    errors = errors + 1;
                    disable run_div;
                end
            end
            if (quotient !== exp_q || remainder !== exp_r) begin
                $display("FAIL %0s: q=%h(exp %h) r=%h(exp %h) [%0d cyc]",
                          name, quotient, exp_q, remainder, exp_r, cyc_count);
                errors = errors + 1;
            end else begin
                $display("PASS %0s: q=%h r=%h [%0d cyc]", name, quotient, remainder, cyc_count);
            end
        end
    endtask

    initial begin
        reset = 1; start = 0; is_signed = 0; is_word = 0; dividend = 0; divisor = 0;
        @(negedge clk); @(negedge clk);
        reset = 0;

        // ---- 64-bit, unsigned ----
        run_div("DIVU 20/4",  0, 0, 64'd20, 64'd4, 64'd5, 64'd0);
        run_div("DIVU 7/2",   0, 0, 64'd7,  64'd2, 64'd3, 64'd1);
        run_div("DIVU 0/5",   0, 0, 64'd0,  64'd5, 64'd0, 64'd0);

        // ---- 64-bit, signed -- all four sign combinations, truncating
        // toward zero (not floor): 7/-2 = -3 r 1, -7/2 = -3 r -1, etc.
        run_div("DIV -20/4",  1, 0, 64'hFFFFFFFFFFFFFFEC /*-20*/, 64'd4,
                               64'hFFFFFFFFFFFFFFFB /*-5*/, 64'd0);
        run_div("DIV 7/-2",   1, 0, 64'd7, 64'hFFFFFFFFFFFFFFFE /*-2*/,
                               64'hFFFFFFFFFFFFFFFD /*-3*/, 64'd1);
        run_div("DIV -7/2",   1, 0, 64'hFFFFFFFFFFFFFFF9 /*-7*/, 64'd2,
                               64'hFFFFFFFFFFFFFFFD /*-3*/, 64'hFFFFFFFFFFFFFFFF /*-1*/);
        run_div("DIV -7/-2",  1, 0, 64'hFFFFFFFFFFFFFFF9 /*-7*/, 64'hFFFFFFFFFFFFFFFE /*-2*/,
                               64'd3, 64'hFFFFFFFFFFFFFFFF /*-1*/);

        // ---- 64-bit, divide-by-zero (defined result, not a trap) ----
        run_div("DIVU 42/0",  0, 0, 64'd42, 64'd0,
                               64'hFFFFFFFFFFFFFFFF, 64'd42);
        run_div("DIV -42/0",  1, 0, 64'hFFFFFFFFFFFFFFD6 /*-42*/, 64'd0,
                               64'hFFFFFFFFFFFFFFFF, 64'hFFFFFFFFFFFFFFD6 /*-42*/);

        // ---- 64-bit, signed overflow: MIN_INT64 / -1 ----
        run_div("DIV MIN/-1", 1, 0, 64'h8000000000000000, 64'hFFFFFFFFFFFFFFFF,
                               64'h8000000000000000, 64'd0);

        // ---- 32-bit (W) forms -- garbage upper 32 bits on every operand,
        // to prove word ops genuinely ignore them, not just happen to work
        // when the upper bits are zero ----
        run_div("DIVUW 20/4", 0, 1, 64'hDEADBEEF00000014 /*20*/, 64'hCAFEBABE00000004 /*4*/,
                               64'd5, 64'd0);
        run_div("DIVW -20/4", 1, 1, 64'h12345678FFFFFFEC /*-20*/, 64'h0000000000000004,
                               64'hFFFFFFFFFFFFFFFB /*-5*/, 64'd0);
        run_div("DIVW MIN32/-1", 1, 1, 64'hAAAAAAAA80000000 /*MIN_INT32*/, 64'hBBBBBBBBFFFFFFFF /*-1*/,
                               64'hFFFFFFFF80000000 /*MIN_INT32 sign-ext*/, 64'd0);
        run_div("DIVUW 42/0", 0, 1, 64'h111111110000002A /*42*/, 64'h2222222200000000 /*0*/,
                               64'hFFFFFFFFFFFFFFFF, 64'h000000000000002A /*42*/);
        run_div("REMW 7/-2",  1, 1, 64'h0000000000000007 /*7*/, 64'hFFFFFFFFFFFFFFFE /*-2*/,
                               64'hFFFFFFFFFFFFFFFD /*quotient -3, checked here too*/, 64'd1);

        if (errors == 0) $display("tb_div_fu: ALL PASS");
        else $display("tb_div_fu: %0d FAILURES", errors);
        $finish;
    end
endmodule
