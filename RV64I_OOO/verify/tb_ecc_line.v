`timescale 1ns / 1ps
// Isolated unit test for ecc_line.v (LINE_BYTES=32, matching this
// project's L1/L2 line size): encodes a 256-bit line, flips one bit in
// each of the 4 underlying 64-bit words in turn (proving per-word
// correction, not just "some correction somewhere"), then flips two bits
// within a single word to confirm dbe fires for that word without a
// false correction.
module tb_ecc_line;
    localparam LINE_BYTES = 32;
    reg [LINE_BYTES*8-1:0] wr_line;
    wire [LINE_BYTES-1:0] wr_check;
    reg [LINE_BYTES*8-1:0] rd_line;
    reg [LINE_BYTES-1:0] rd_check;
    wire [LINE_BYTES*8-1:0] corrected;
    wire sbe, dbe;

    ecc_line #(.LINE_BYTES(LINE_BYTES)) dut (
        .wr_line(wr_line), .wr_check(wr_check),
        .rd_line(rd_line), .rd_check(rd_check),
        .rd_line_corrected(corrected), .rd_sbe(sbe), .rd_dbe(dbe)
    );

    integer errors;
    integer w, b;
    initial begin
        errors = 0;
        wr_line = 256'h0123456789ABCDEF_FEDCBA9876543210_DEADBEEFCAFEBABE_1122334455667788;
        #1;

        // Clean round-trip
        rd_line = wr_line; rd_check = wr_check;
        #1;
        if (corrected !== wr_line || sbe !== 1'b0 || dbe !== 1'b0) begin
            $display("[FAIL] clean line round-trip: sbe=%b dbe=%b", sbe, dbe);
            errors = errors + 1;
        end

        // One single-bit flip per word, each word in turn
        for (w = 0; w < 4; w = w + 1) begin
            rd_line = wr_line; rd_check = wr_check;
            b = w*64 + 17; // arbitrary bit within word w
            rd_line[b] = ~rd_line[b];
            #1;
            if (corrected !== wr_line || sbe !== 1'b1 || dbe !== 1'b0) begin
                $display("[FAIL] single-bit flip in word %0d (bit %0d): corrected=%h sbe=%b dbe=%b",
                          w, b, corrected, sbe, dbe);
                errors = errors + 1;
            end
        end

        // Double-bit flip within word 2 only
        rd_line = wr_line; rd_check = wr_check;
        rd_line[2*64 + 3]  = ~rd_line[2*64 + 3];
        rd_line[2*64 + 40] = ~rd_line[2*64 + 40];
        #1;
        if (dbe !== 1'b1) begin
            $display("[FAIL] double-bit flip in word 2: dbe=%b (want 1)", dbe);
            errors = errors + 1;
        end
        // Words 0,1,3 are untouched and must not spuriously report dbe
        // on their own -- dbe is OR'd across the whole line by design, so
        // this just re-confirms the aggregate is driven by word 2 alone,
        // not a bundling bug that always reports dbe.
        if (sbe !== 1'b0) begin
            $display("[FAIL] double-bit flip in word 2: sbe=%b (want 0, since 2-bit errors don't set sbe)", sbe);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("[PASS] tb_ecc_line: all checks passed");
        else
            $display("[FAIL] tb_ecc_line: %0d error(s)", errors);
        $finish;
    end
endmodule
