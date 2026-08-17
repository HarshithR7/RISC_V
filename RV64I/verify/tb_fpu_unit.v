`timescale 1ns/1ps
// Unit-level testbench for fpu.v in isolation (no fetch/decode/register
// file involved) -- fast to iterate on, and exactly what caught two real
// bugs during F development: an Icarus width-propagation quirk in a
// negation expression, and $itor/$rtoi silently truncating to 32 bits.
// Expected values are float32/float64 bit patterns computed independently
// in Python (see RV64I/README.md).
module tb_fpu_unit;
    reg [63:0] a, b, c;
    reg [63:0] int_a;
    reg [4:0] fp_op;
    reg is_double, word_int_src;
    wire [63:0] fresult;
    wire [63:0] iresult;
    integer errors = 0;

    fpu dut (.a(a), .b(b), .c(c), .int_a(int_a), .fp_op(fp_op), .is_double(is_double),
              .word_int_src(word_int_src), .fresult(fresult), .iresult(iresult));

    task check_f(input [255:0] name, input [63:0] got, input [63:0] exp);
        begin
            if (got !== exp) begin
                $display("FAIL %0s: got=%h exp=%h", name, got, exp);
                errors = errors + 1;
            end else $display("PASS %0s: %h", name, got);
        end
    endtask
    task check_i(input [255:0] name, input [63:0] got, input [63:0] exp);
        begin
            if (got !== exp) begin
                $display("FAIL %0s: got=%h exp=%h", name, got, exp);
                errors = errors + 1;
            end else $display("PASS %0s: %h", name, got);
        end
    endtask

    initial begin
        is_double = 0;
        #1;
        // ---- Single precision (F) ----
        a=64'hFFFFFFFF40200000; b=64'hFFFFFFFF40600000; fp_op=0; #1 check_f("FADD 2.5+3.5", fresult, 64'hFFFFFFFF40c00000);
        a=64'hFFFFFFFF40c00000; b=64'hFFFFFFFF40600000; fp_op=1; #1 check_f("FSUB 6-3.5", fresult, 64'hFFFFFFFF40200000);
        a=64'hFFFFFFFF40200000; b=64'hFFFFFFFF40000000; fp_op=2; #1 check_f("FMUL 2.5*2", fresult, 64'hFFFFFFFF40a00000);
        a=64'hFFFFFFFF40e00000; b=64'hFFFFFFFF40000000; fp_op=3; #1 check_f("FDIV 7/2", fresult, 64'hFFFFFFFF40600000);
        a=64'hFFFFFFFF40000000; fp_op=4; #1 check_f("FSQRT sqrt(2)", fresult, 64'hFFFFFFFF3fb504f3);
        a=64'hFFFFFFFF40200000; b=64'hFFFFFFFFbf800000; fp_op=5; #1 check_f("FSGNJ", fresult, 64'hFFFFFFFFc0200000);
        a=64'hFFFFFFFF40200000; b=64'hFFFFFFFFbf800000; fp_op=8; #1 check_f("FMIN", fresult, 64'hFFFFFFFFbf800000);
        a=64'hFFFFFFFF40200000; b=64'hFFFFFFFF40200000; fp_op=10; #1 check_i("FEQ equal", iresult, 64'd1);
        a=64'hFFFFFFFF42c80000; fp_op=14; #1 check_i("FCVT.W.S 100.0", iresult, 64'd100);
        a=64'hFFFFFFFFbf800000; fp_op=14; #1 check_i("FCVT.W.S -1.0", iresult, 64'hFFFFFFFFFFFFFFFF);
        int_a=64'd42; word_int_src=1; fp_op=18; #1 check_f("FCVT.S.W 42", fresult, 64'hFFFFFFFF42280000);
        a = 64'hFFFFFFFFDEADBEEF; fp_op=22; #1 check_i("FMV.X.W", iresult, {{32{1'b1}},32'hDEADBEEF});
        int_a = 64'h1234567800ABCDEF; fp_op=23; #1 check_f("FMV.W.X", fresult, 64'hFFFFFFFF00ABCDEF);
        int_a = 64'd10000000000; fp_op=20; #1 check_f("FCVT.S.L 10e9", fresult, 64'hFFFFFFFF501502f9);
        a = 64'hFFFFFFFF501502f9; fp_op=16; #1 check_i("FCVT.L.S roundtrip", iresult, 64'd10000000000);
        a = 64'hFFFFFFFF7fc00000; b = 64'hFFFFFFFF40200000; fp_op=0; #1 check_f("FADD NaN+x -> NaN", fresult, 64'hFFFFFFFF7fc00000);
        a=64'hFFFFFFFF40000000; b=64'hFFFFFFFF40400000; c=64'hFFFFFFFF3f800000; fp_op=24; #1 check_f("FMADD 2*3+1", fresult, 64'hFFFFFFFF40e00000);

        // ---- Double precision (D) ----
        is_double = 1;
        a=64'h4004000000000000; b=64'h400c000000000000; fp_op=0; #1 check_f("FADD.D 2.5+3.5", fresult, 64'h4018000000000000); // 6.0
        a=64'h4018000000000000; b=64'h400c000000000000; fp_op=1; #1 check_f("FSUB.D 6-3.5", fresult, 64'h4004000000000000); // 2.5
        a=64'h4010000000000000; fp_op=4; #1 check_f("FSQRT.D sqrt(4)", fresult, 64'h4000000000000000); // 2.0
        a=64'h4059000000000000; fp_op=14; #1 check_i("FCVT.W.D 100.0", iresult, 64'd100);
        a=64'hbff0000000000000; fp_op=14; #1 check_i("FCVT.W.D -1.0", iresult, 64'hFFFFFFFFFFFFFFFF);
        int_a=64'd42; word_int_src=1; fp_op=18; #1 check_f("FCVT.D.W 42", fresult, 64'h4045000000000000);
        a=64'h4004000000000000; b=64'h4004000000000000; fp_op=10; #1 check_i("FEQ.D equal", iresult, 64'd1);
        a=64'hDEADBEEF12345678; fp_op=22; #1 check_i("FMV.X.D", iresult, 64'hDEADBEEF12345678);
        int_a=64'h1122334455667788; fp_op=23; #1 check_f("FMV.D.X", fresult, 64'h1122334455667788);
        a=64'h4059000000000000; fp_op=13; #1 check_i("FCLASS.D +normal", iresult, 64'h40); // 100.0
        // FCVT.D.S: 2.5 (float32) -> 2.5 (double), exact widening
        a=64'hFFFFFFFF40200000; fp_op=29; #1 check_f("FCVT.D.S 2.5", fresult, 64'h4004000000000000);
        // FCVT.S.D: 2.5 (double) -> 2.5 (float32), exact narrowing
        a=64'h4004000000000000; fp_op=28; #1 check_f("FCVT.S.D 2.5", fresult, 64'hFFFFFFFF40200000);

        if (errors == 0) $display("tb_fpu_unit: ALL PASS");
        else $display("tb_fpu_unit: %0d FAILURES", errors);
        $finish;
    end
endmodule
