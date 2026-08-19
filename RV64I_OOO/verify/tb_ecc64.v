`timescale 1ns / 1ps
// Isolated unit test for ecc64.v, the SECDED(72,64) primitive every
// ECC-protected structure in this phase builds on -- same convention as
// tb_rob_rat.v: a pure-logic testbench with no assembly/program involved,
// run directly via iverilog+vvp, PASS/FAIL counted in Verilog itself.
//
// Exhaustively flips every one of the 72 stored bits (64 data + 8 check)
// one at a time across a handful of representative data patterns, and
// separately checks a sample of two-bit-flip combinations, rather than
// trusting the Hamming construction by inspection -- correction position
// math is exactly the kind of off-by-one-prone logic where "looks right"
// and "is right" diverge.
module tb_ecc64;
    reg [63:0] data;
    wire [7:0] check;
    reg [63:0] rd_data;
    reg [7:0] rd_check;
    wire [63:0] corrected;
    wire sbe, dbe;

    ecc64 dut (
        .wr_data(data), .wr_check(check),
        .rd_data(rd_data), .rd_check(rd_check),
        .rd_data_corrected(corrected), .rd_sbe(sbe), .rd_dbe(dbe)
    );

    integer errors;
    integer bit_pos;
    integer pattern_idx;
    reg [63:0] patterns [0:4];
    reg [71:0] codeword;

    task check_no_error;
        begin
            #1;
            if (corrected !== data || sbe !== 1'b0 || dbe !== 1'b0) begin
                $display("[FAIL] no-error case: data=%h corrected=%h sbe=%b dbe=%b",
                          data, corrected, sbe, dbe);
                errors = errors + 1;
            end
        end
    endtask

    integer i;
    initial begin
        errors = 0;
        patterns[0] = 64'h0000000000000000;
        patterns[1] = 64'hFFFFFFFFFFFFFFFF;
        patterns[2] = 64'hA5A5A5A5A5A5A5A5;
        patterns[3] = 64'h123456789ABCDEF0;
        patterns[4] = 64'h8000000000000001;

        for (pattern_idx = 0; pattern_idx < 5; pattern_idx = pattern_idx + 1) begin
            data = patterns[pattern_idx];
            #1;
            // Zero-error case
            rd_data = data;
            rd_check = check;
            check_no_error;

            // Single-bit flips across all 72 stored bit positions
            for (bit_pos = 0; bit_pos < 72; bit_pos = bit_pos + 1) begin
                codeword = {check, data}; // bit 0..63 = data, 64..71 = check
                codeword[bit_pos] = ~codeword[bit_pos];
                rd_data = codeword[63:0];
                rd_check = codeword[71:64];
                #1;
                if (corrected !== data || sbe !== 1'b1 || dbe !== 1'b0) begin
                    $display("[FAIL] pattern=%h bit_pos=%0d: corrected=%h (want %h) sbe=%b (want 1) dbe=%b (want 0)",
                              data, bit_pos, corrected, data, sbe, dbe);
                    errors = errors + 1;
                end
            end

            // A sample of double-bit flips: must be flagged dbe, must
            // NOT be silently miscorrected to a wrong-but-confident value.
            for (i = 0; i < 20; i = i + 1) begin
                codeword = {check, data};
                codeword[i] = ~codeword[i];
                codeword[(i + 37) % 72] = ~codeword[(i + 37) % 72];
                rd_data = codeword[63:0];
                rd_check = codeword[71:64];
                #1;
                if (dbe !== 1'b1) begin
                    $display("[FAIL] pattern=%h double-flip bits %0d,%0d: dbe=%b (want 1)",
                              data, i, (i + 37) % 72, dbe);
                    errors = errors + 1;
                end
            end
        end

        if (errors == 0)
            $display("[PASS] tb_ecc64: all checks passed (5 patterns x (1 clean + 72 single-flip + 20 double-flip))");
        else
            $display("[FAIL] tb_ecc64: %0d error(s)", errors);
        $finish;
    end
endmodule
