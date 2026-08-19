`timescale 1ns / 1ps
// ECC wrapper for a whole cache line, built on top of ecc64.v: a line is
// just LINE_BYTES/8 independent 64-bit words, each with its own SECDED(72,64)
// code -- no benefit to a single wider code across the whole line (it
// would only ever correct one bit line-wide instead of one bit *per
// word*, strictly worse), so this is a thin generate-loop bundle of
// per-word ecc64 instances, not a new code. Check-bit storage is exactly
// LINE_BYTES bits wide (8 check bits per 64-bit word, and there are
// LINE_BYTES/8 words -- (LINE_BYTES/8)*8 == LINE_BYTES whenever LINE_BYTES
// is, as it always is here, a multiple of 8).
//
// Same wr_*/rd_* bundling convention as ecc64.v: both directions are
// always computed combinationally; a caller wires up whichever half (or
// both -- see l1_cache.v, which feeds a single instance's wr_line from a
// value derived from that same instance's own rd_line_corrected, e.g. a
// read-modify-write merge) it needs for a given instantiation.
//
// rd_sbe/rd_dbe are OR'd across all words in the line: a caller that
// needs per-word detail can read the underlying per-word ecc64 instances
// directly (WORDS[w].u.rd_sbe etc.) the same way this project's
// testbenches already reach into other modules' internals, but no
// current caller needs that granularity.
module ecc_line #(
    parameter LINE_BYTES = 32
)(
    input  [LINE_BYTES*8-1:0] wr_line,
    output [LINE_BYTES-1:0]   wr_check,

    input  [LINE_BYTES*8-1:0] rd_line,
    input  [LINE_BYTES-1:0]   rd_check,
    output [LINE_BYTES*8-1:0] rd_line_corrected,
    output rd_sbe,
    output rd_dbe
);
    localparam DWORDS = LINE_BYTES / 8;

    wire [DWORDS-1:0] sbe_bits, dbe_bits;

    genvar w;
    generate
        for (w = 0; w < DWORDS; w = w + 1) begin : WORDS
            ecc64 u (
                .wr_data(wr_line[w*64 +: 64]), .wr_check(wr_check[w*8 +: 8]),
                .rd_data(rd_line[w*64 +: 64]), .rd_check(rd_check[w*8 +: 8]),
                .rd_data_corrected(rd_line_corrected[w*64 +: 64]),
                .rd_sbe(sbe_bits[w]), .rd_dbe(dbe_bits[w])
            );
        end
    endgenerate

    assign rd_sbe = |sbe_bits;
    assign rd_dbe = |dbe_bits;
endmodule
