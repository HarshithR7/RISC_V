`timescale 1ns / 1ps
// SECDED(72,64) Hamming ECC: single-error-correct, double-error-detect
// over a 64-bit word, using 8 check bits (7 real Hamming parity bits
// covering a 71-bit virtual codeword, plus 1 overall parity bit across
// that whole 71-bit codeword) -- the same construction real ECC DRAM/ECC
// caches use. This is the one shared primitive every ECC-protected
// storage structure in Phase 9's "ECC on memory structures" work
// (register file, L1, L2, ROB) instantiates, so the actual Hamming
// bit-position math only has to be gotten right once.
//
// Bit-position convention (1-indexed, standard Hamming layout): the 71
// positions 1..71 are logically laid out with parity bits at every
// power-of-two position (1,2,4,8,16,32,64 -- 7 of them) and the 64 data
// bits filling every other position, in order. Position p's binary
// representation directly says which parity groups (bits of p that are
// set) cover it -- that's the entire mechanism: parity bit k covers
// exactly the positions whose binary representation has bit k set,
// which is also why a parity bit's own position (a lone power of two)
// only ever appears in its own group, never any other's. Data bit d's
// mapping to codeword position is computed once by `ecc_mask`, which
// walks positions 1..71 skipping every power of two and records, for
// each data bit, whether its position has bit k set -- i.e. `ecc_mask(k)`
// is exactly the 64-bit "which data bits does parity group k cover" mask.
//
// This module always computes both directions combinationally: the
// wr_* side (encode) for whoever is about to store a word, and the rd_*
// side (decode+correct) for whoever just read one back. A caller only
// wires up the half it needs for a given instantiation; the other half
// is simply unused logic, not a correctness concern.
//
// Decode result classification (the standard SECDED disambiguation):
//   syndrome == 0, overall parity OK       -> no error
//   syndrome == 0, overall parity mismatch -> the overall-parity check
//                                              bit itself was the one
//                                              flipped bit (sbe, data
//                                              untouched)
//   syndrome != 0, overall parity mismatch -> single-bit error at the
//                                              Hamming position named by
//                                              the syndrome (sbe,
//                                              corrected -- in data if
//                                              that position is a data
//                                              bit, in a Hamming parity
//                                              bit otherwise)
//   syndrome != 0, overall parity OK       -> two bits wrong (any single
//                                              real bit-flip always also
//                                              flips overall parity; only
//                                              an even number of flips
//                                              can leave it matching
//                                              while still disturbing the
//                                              Hamming syndrome) -- dbe,
//                                              uncorrectable, data is
//                                              *not* modified since which
//                                              two bits is undeterminable
//                                              from this code.
module ecc64 (
    input  [63:0] wr_data,
    output [7:0]  wr_check,

    input  [63:0] rd_data,
    input  [7:0]  rd_check,
    output reg [63:0] rd_data_corrected,
    output reg        rd_sbe,
    output reg        rd_dbe
);
    // Which data bits (0..63) does Hamming parity group k (k=0..6, i.e.
    // parity bit at codeword position 2^k) cover? Walks the 71-position
    // virtual codeword in order, skipping power-of-two (parity)
    // positions, assigning the rest to data bits 0..63 in sequence.
    function [63:0] ecc_mask;
        input integer k;
        integer d;
        integer pos;
        reg [63:0] m;
    begin
        m = 64'b0;
        pos = 0;
        for (d = 0; d < 64; d = d + 1) begin
            pos = pos + 1;
            while ((pos & (pos - 1)) == 0)
                pos = pos + 1;
            m[d] = (((pos >> k) & 1) == 1);
        end
        ecc_mask = m;
    end
    endfunction

    // Does data bit d's Hamming codeword position have bit k set? Same
    // walk as ecc_mask, but returns one bit instead of the whole mask --
    // used only to classify a syndrome hit as "this is data bit d" while
    // decoding.
    function is_data_bit_at_position;
        input integer target_pos;
        input integer d;
        integer dd;
        integer pos;
    begin
        pos = 0;
        is_data_bit_at_position = 1'b0;
        for (dd = 0; dd <= d; dd = dd + 1) begin
            pos = pos + 1;
            while ((pos & (pos - 1)) == 0)
                pos = pos + 1;
        end
        is_data_bit_at_position = (pos == target_pos);
    end
    endfunction

    genvar gk;
    generate
        for (gk = 0; gk < 7; gk = gk + 1) begin : ENC_PARITY
            assign wr_check[gk] = ^(wr_data & ecc_mask(gk));
        end
    endgenerate
    assign wr_check[7] = ^{wr_data, wr_check[6:0]};

    integer sk;
    reg [6:0] syndrome;
    reg overall_err;
    integer di;
    always @(*) begin
        for (sk = 0; sk < 7; sk = sk + 1)
            syndrome[sk] = rd_check[sk] ^ (^(rd_data & ecc_mask(sk)));
        overall_err = rd_check[7] ^ (^{rd_data, rd_check[6:0]});

        rd_data_corrected = rd_data;
        rd_sbe = 1'b0;
        rd_dbe = 1'b0;

        if (syndrome == 7'b0 && overall_err == 1'b0) begin
            // no error
        end else if (syndrome == 7'b0 && overall_err == 1'b1) begin
            // the overall parity bit itself flipped; data untouched
            rd_sbe = 1'b1;
        end else if (syndrome != 7'b0 && overall_err == 1'b1) begin
            rd_sbe = 1'b1;
            for (di = 0; di < 64; di = di + 1)
                if (is_data_bit_at_position(syndrome, di))
                    rd_data_corrected[di] = ~rd_data[di];
            // if the syndrome instead names one of the 7 Hamming parity
            // bit positions (a power of two), no data bit matches
            // is_data_bit_at_position for any di, so data is correctly
            // left untouched -- the corrected bit was a check bit.
        end else begin
            // syndrome != 0 && overall_err == 0: double-bit error
            rd_dbe = 1'b1;
        end
    end
endmodule
