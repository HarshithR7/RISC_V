`timescale 1ns / 1ps
// RV64FD floating-point unit (single AND double precision). Arithmetic is
// computed via Verilog's native double-precision `real`
// ($bitstoreal/$realtobits): for single-precision ops, a float32 bit
// pattern is exactly widened to float64 (f32_to_f64, lossless), the
// operation runs in double precision, and the double result is rounded
// back down to float32 (f64_to_f32 -- the only place single-precision
// rounding happens). For double-precision ops, `a`/`b`/`c` already *are*
// double bit patterns, so there is no widen-then-round step at all: the
// `real` result is used directly. Only RNE (round-to-nearest-even) is
// implemented -- `rm` is decoded but not otherwise used; see
// RV64I/README.md for why. Comparison/classify/sign-injection/move
// operations are exact bit manipulation with no rounding involved.
module fpu (
    input  [63:0] a, b, c,      // rs1, rs2, rs3 -- float32 (NaN-boxed, low
                                 // 32 bits significant) or float64, per is_double
    input  [63:0] int_a,        // integer rs1, for FCVT.{S,D}.*/FMV.{W,D}.X
    input  [4:0]  fp_op,
    input  is_double,           // 0 = single precision (F), 1 = double (D)
    input  word_int_src,        // 1 = int_a is a 32-bit (sign-extended) source, 0 = 64-bit
    output reg [63:0] fresult,       // float result: NaN-boxed float32, or a full float64
    output reg [63:0] iresult        // integer result (compares/classify/convert-to-int/FMV.X.*)
);
    localparam FADD=5'd0, FSUB=5'd1, FMUL=5'd2, FDIV=5'd3, FSQRT=5'd4,
               FSGNJ=5'd5, FSGNJN=5'd6, FSGNJX=5'd7, FMIN=5'd8, FMAX=5'd9,
               FEQ=5'd10, FLT=5'd11, FLE=5'd12, FCLASS=5'd13,
               FCVT_W_S=5'd14, FCVT_WU_S=5'd15, FCVT_L_S=5'd16, FCVT_LU_S=5'd17,
               FCVT_S_W=5'd18, FCVT_S_WU=5'd19, FCVT_S_L=5'd20, FCVT_S_LU=5'd21,
               FMV_X_W=5'd22, FMV_W_X=5'd23,
               FMADD=5'd24, FMSUB=5'd25, FNMSUB=5'd26, FNMADD=5'd27,
               FCVT_S_D=5'd28, FCVT_D_S=5'd29;

    function is_nan32;
        input [31:0] v;
        is_nan32 = (v[30:23] == 8'hFF) && (v[22:0] != 0);
    endfunction
    function is_nan64;
        input [63:0] v;
        is_nan64 = (v[62:52] == 11'h7FF) && (v[51:0] != 0);
    endfunction
    function is_nan_op;
        input [63:0] v;
        input dbl;
        is_nan_op = dbl ? is_nan64(v) : is_nan32(v[31:0]);
    endfunction
    localparam [31:0] QNAN32 = 32'h7FC00000;
    localparam [63:0] QNAN64 = 64'h7FF8000000000000;

    // ---- float32 <-> float64 (exact widening; no rounding possible) --
    function [63:0] f32_to_f64;
        input [31:0] a32;
        reg s; reg [7:0] e; reg [22:0] m;
        reg [10:0] e64; reg [51:0] m64;
        integer lz;
        reg [22:0] ms;
        begin
            s = a32[31]; e = a32[30:23]; m = a32[22:0];
            if (e == 8'hFF)
                f32_to_f64 = {s, 11'h7FF, m, 29'b0};
            else if (e == 8'h00 && m == 23'b0)
                f32_to_f64 = {s, 11'd0, 52'd0};
            else if (e == 8'h00) begin
                lz = 0; ms = m;
                while (ms[22] == 1'b0) begin ms = ms << 1; lz = lz + 1; end
                ms = ms << 1; // drop the now-implicit leading 1
                e64 = 11'd896 - lz[10:0];
                f32_to_f64 = {s, e64, ms[21:0], 30'b0};
            end else begin
                e64 = {3'b0, e} + 11'd896; // rebias: 1023-127
                f32_to_f64 = {s, e64, m, 29'b0};
            end
        end
    endfunction

    // Round a double bit pattern down to float32 (round-to-nearest-even).
    function [31:0] f64_to_f32;
        input [63:0] a64;
        reg s; reg [10:0] e; reg [51:0] m;
        reg signed [12:0] e32;
        reg guard, sticky, roundup;
        reg [23:0] mrounded;
        integer shift_amt;
        reg [52:0] sig53;
        reg [76:0] wide;
        begin
            s = a64[63]; e = a64[62:52]; m = a64[51:0];
            if (e == 11'h7FF) begin
                if (m == 52'b0) f64_to_f32 = {s, 8'hFF, 23'b0};
                else f64_to_f32 = {s, 8'hFF, (m[51:29] == 0) ? 23'h400000 : m[51:29]};
            end else if (e == 11'd0 && m == 52'd0) begin
                f64_to_f32 = {s, 31'b0};
            end else if (e == 11'd0) begin
                f64_to_f32 = {s, 31'b0}; // double subnormal: far too small to matter for float32
            end else begin
                e32 = $signed({2'b0, e}) - 13'd896;
                if (e32 >= 255) begin
                    f64_to_f32 = {s, 8'hFF, 23'b0}; // overflow -> inf
                end else if (e32 <= 0) begin
                    shift_amt = 1 - e32;
                    if (shift_amt > 25) begin
                        f64_to_f32 = {s, 31'b0};
                    end else begin
                        sig53 = {1'b1, m};
                        wide = {sig53, 24'b0} >> shift_amt;
                        guard = wide[53];
                        sticky = |wide[52:0];
                        roundup = guard && (sticky || wide[54]);
                        mrounded = wide[76:54] + roundup;
                        if (mrounded[23]) f64_to_f32 = {s, 8'd1, 23'b0}; // rounded up to smallest normal
                        else f64_to_f32 = {s, 8'd0, mrounded[22:0]};
                    end
                end else begin
                    guard = m[28];
                    sticky = |m[27:0];
                    roundup = guard && (sticky || m[29]);
                    mrounded = m[51:29] + roundup;
                    if (mrounded[23]) begin
                        if (e32 + 1 >= 255) f64_to_f32 = {s, 8'hFF, 23'b0};
                        else f64_to_f32 = {s, e32[7:0] + 8'd1, 23'b0};
                    end else begin
                        f64_to_f32 = {s, e32[7:0], mrounded[22:0]};
                    end
                end
            end
        end
    endfunction

    real ra, rb, rc, rresult;
    reg [63:0] arith_result;
    reg [31:0] neg32;   // -int_a[31:0], computed at an explicit 32-bit width
    reg [63:0] neg64;   // -int_a, computed at an explicit 64-bit width
    // (Icarus quirk, confirmed empirically: `~int_a[31:0] + 1'b1` used
    // inline inside a 64-bit-context ternary does NOT stay at its
    // self-determined 32-bit width -- it silently widens int_a[31:0] to
    // 64 bits *before* negating, corrupting the upper 32 bits. Computing
    // the negation into its own explicitly-sized reg first sidesteps the
    // ambiguity entirely.)

    wire a_nan = is_nan_op(a, is_double);
    wire b_nan = is_nan_op(b, is_double);
    wire c_nan = is_nan_op(c, is_double);
    wire [63:0] qnan = is_double ? QNAN64 : {32'hFFFFFFFF, QNAN32};

    always @(*) begin
        ra = is_double ? $bitstoreal(a) : $bitstoreal(f32_to_f64(a[31:0]));
        rb = is_double ? $bitstoreal(b) : $bitstoreal(f32_to_f64(b[31:0]));
        rc = is_double ? $bitstoreal(c) : $bitstoreal(f32_to_f64(c[31:0]));
        neg32 = ~int_a[31:0] + 1'b1;
        neg64 = ~int_a + 1'b1;

        case (fp_op)
            FADD: rresult = ra + rb;
            FSUB: rresult = ra - rb;
            FMUL: rresult = ra * rb;
            FDIV: rresult = ra / rb;
            FSQRT: rresult = (ra < 0.0) ? (0.0/0.0) : ra ** 0.5;
            FMADD:  rresult = (ra * rb) + rc;
            FMSUB:  rresult = (ra * rb) - rc;
            FNMSUB: rresult = -(ra * rb) + rc;
            FNMADD: rresult = -(ra * rb) - rc;
            default: rresult = 0.0;
        endcase

        // NaN propagation: RISC-V requires a quiet NaN result if any input
        // is NaN (real-arithmetic NaN propagation is not guaranteed to
        // preserve this cleanly, so it's forced explicitly here).
        case (fp_op)
            FADD, FSUB, FMUL, FDIV:
                arith_result = (a_nan || b_nan) ? qnan :
                                (is_double ? $realtobits(rresult) : {32'hFFFFFFFF, f64_to_f32($realtobits(rresult))});
            FSQRT:
                arith_result = a_nan ? qnan :
                                (is_double ? $realtobits(rresult) : {32'hFFFFFFFF, f64_to_f32($realtobits(rresult))});
            FMADD, FMSUB, FNMSUB, FNMADD:
                arith_result = (a_nan || b_nan || c_nan) ? qnan :
                                (is_double ? $realtobits(rresult) : {32'hFFFFFFFF, f64_to_f32($realtobits(rresult))});
            default: arith_result = 64'b0;
        endcase

        // Sign injection: exact bit manipulation, magnitude from a, sign
        // from (a,b) per the specific op -- never touches rounding.
        fresult = 64'b0;
        case (fp_op)
            FADD, FSUB, FMUL, FDIV, FSQRT, FMADD, FMSUB, FNMSUB, FNMADD:
                fresult = arith_result;
            FSGNJ:  fresult = is_double ? {b[63], a[62:0]} : {32'hFFFFFFFF, b[31], a[30:0]};
            FSGNJN: fresult = is_double ? {~b[63], a[62:0]} : {32'hFFFFFFFF, ~b[31], a[30:0]};
            FSGNJX: fresult = is_double ? {a[63]^b[63], a[62:0]} : {32'hFFFFFFFF, a[31]^b[31], a[30:0]};
            FMIN: begin
                if (a_nan && b_nan) fresult = qnan;
                else if (a_nan) fresult = b;
                else if (b_nan) fresult = a;
                else fresult = (ra <= rb) ? a : b; // -0.0 < +0.0 not distinguished by `real`, acceptable here
            end
            FMAX: begin
                if (a_nan && b_nan) fresult = qnan;
                else if (a_nan) fresult = b;
                else if (b_nan) fresult = a;
                else fresult = (ra >= rb) ? a : b;
            end
            FCVT_S_W:  fresult = is_double ? int_to_f64(int_a[31] ? {32'b0, neg32} : {32'b0, int_a[31:0]}, int_a[31])
                                            : {32'hFFFFFFFF, int_to_f32(int_a[31] ? {32'b0, neg32} : {32'b0, int_a[31:0]}, int_a[31])};
            FCVT_S_WU: fresult = is_double ? int_to_f64({32'b0, int_a[31:0]}, 1'b0)
                                            : {32'hFFFFFFFF, int_to_f32({32'b0, int_a[31:0]}, 1'b0)};
            FCVT_S_L:  fresult = is_double ? int_to_f64(int_a[63] ? neg64 : int_a, int_a[63])
                                            : {32'hFFFFFFFF, int_to_f32(int_a[63] ? neg64 : int_a, int_a[63])};
            FCVT_S_LU: fresult = is_double ? int_to_f64(int_a, 1'b0)
                                            : {32'hFFFFFFFF, int_to_f32(int_a, 1'b0)};
            FMV_W_X:   fresult = is_double ? int_a : {32'hFFFFFFFF, int_a[31:0]}; // FMV.D.X / FMV.W.X
            FCVT_S_D:  fresult = {32'hFFFFFFFF, f64_to_f32(a)};       // double -> float (rounds)
            FCVT_D_S:  fresult = f32_to_f64(a[31:0]);                 // float -> double (exact)
            default: ;
        endcase

        // Integer-result paths: compares, classify, convert-to-int, FMV.X.*
        iresult = 64'b0;
        case (fp_op)
            FEQ: iresult = (a_nan || b_nan) ? 64'd0 : ((ra == rb) ? 64'd1 : 64'd0);
            FLT: iresult = (a_nan || b_nan) ? 64'd0 : ((ra <  rb) ? 64'd1 : 64'd0);
            FLE: iresult = (a_nan || b_nan) ? 64'd0 : ((ra <= rb) ? 64'd1 : 64'd0);
            FCLASS: iresult = is_double ? dclass(a) : fclass(a[31:0]);
            FMV_X_W: iresult = is_double ? a : {{32{a[31]}}, a[31:0]}; // FMV.X.D / FMV.X.W (W sign-extends per spec)
            FCVT_W_S:  iresult = is_double ? d64_to_int(a, 1'b1, 1'b1) : f32_to_int(a[31:0], 1'b1, 1'b1);
            FCVT_WU_S: iresult = is_double ? d64_to_int(a, 1'b0, 1'b1) : f32_to_int(a[31:0], 1'b0, 1'b1);
            FCVT_L_S:  iresult = is_double ? d64_to_int(a, 1'b1, 1'b0) : f32_to_int(a[31:0], 1'b1, 1'b0);
            FCVT_LU_S: iresult = is_double ? d64_to_int(a, 1'b0, 1'b0) : f32_to_int(a[31:0], 1'b0, 1'b0);
            default: ;
        endcase
    end

    // Converts a 64-bit unsigned magnitude + sign to the nearest float32
    // (round-to-nearest-even), entirely via bit manipulation -- $itor is
    // documented-but-actually-32-bit-only in this Icarus build (confirmed
    // empirically: $itor(10_000_000_000) silently truncates to the value
    // mod 2^32), so it's unusable for the L/LU conversions.
    function [31:0] int_to_f32;
        input [63:0] mag;
        input sign;
        integer msb_pos;
        integer i;
        reg [63:0] shifted;
        reg guard, sticky, roundup;
        reg [24:0] mtrunc; // 1 extra bit to catch a rounding carry-out
        reg [7:0] e;
        begin
            if (mag == 64'd0) begin
                int_to_f32 = {sign, 31'b0};
            end else begin
                msb_pos = 0;
                for (i = 0; i < 64; i = i + 1)
                    if (mag[i]) msb_pos = i;
                if (msb_pos >= 23) begin
                    shifted = mag >> (msb_pos - 23);
                    guard  = mag[msb_pos-24];
                    sticky = (msb_pos >= 25) ? |(mag & ((64'b1 << (msb_pos-24)) - 64'b1)) : 1'b0;
                end else begin
                    shifted = mag << (23 - msb_pos);
                    guard = 1'b0; sticky = 1'b0;
                end
                roundup = guard && (sticky || shifted[0]);
                mtrunc = shifted[23:0] + roundup; // bit23 = implicit leading 1
                e = msb_pos[7:0] + 8'd127;
                if (mtrunc[24]) // rounding carried out of the mantissa
                    int_to_f32 = {sign, e + 8'd1, 23'b0};
                else
                    int_to_f32 = {sign, e, mtrunc[22:0]};
            end
        end
    endfunction

    // Same as int_to_f32 but building a full float64 (53-bit significand
    // window instead of 24-bit). A 64-bit magnitude always fits within
    // double's exponent range, so there's no overflow-to-infinity case
    // the way there can be for float32.
    function [63:0] int_to_f64;
        input [63:0] mag;
        input sign;
        integer msb_pos;
        integer i;
        reg [63:0] shifted;
        reg guard, sticky, roundup;
        reg [53:0] mtrunc; // 1 extra bit to catch a rounding carry-out
        reg [10:0] e;
        begin
            if (mag == 64'd0) begin
                int_to_f64 = {sign, 63'b0};
            end else begin
                msb_pos = 0;
                for (i = 0; i < 64; i = i + 1)
                    if (mag[i]) msb_pos = i;
                if (msb_pos >= 52) begin
                    shifted = mag >> (msb_pos - 52);
                    guard  = (msb_pos >= 53) ? mag[msb_pos-53] : 1'b0;
                    sticky = (msb_pos >= 54) ? |(mag & ((64'b1 << (msb_pos-53)) - 64'b1)) : 1'b0;
                end else begin
                    shifted = mag << (52 - msb_pos);
                    guard = 1'b0; sticky = 1'b0;
                end
                roundup = guard && (sticky || shifted[0]);
                mtrunc = shifted[52:0] + roundup; // bit52 = implicit leading 1
                e = msb_pos[10:0] + 11'd1023;
                if (mtrunc[53])
                    int_to_f64 = {sign, e + 11'd1, 52'b0};
                else
                    int_to_f64 = {sign, e, mtrunc[51:0]};
            end
        end
    endfunction

    function [63:0] fclass;
        input [31:0] v;
        reg s; reg [7:0] e; reg [22:0] m;
        begin
            s = v[31]; e = v[30:23]; m = v[22:0];
            if (e == 8'hFF && m != 0)
                fclass = v[22] ? 64'h200 : 64'h100; // quiet / signaling NaN
            else if (e == 8'hFF)
                fclass = s ? 64'h1 : 64'h80; // -inf / +inf
            else if (e == 8'h00 && m == 0)
                fclass = s ? 64'h8 : 64'h10; // -0 / +0
            else if (e == 8'h00)
                fclass = s ? 64'h4 : 64'h20; // -subnormal / +subnormal
            else
                fclass = s ? 64'h2 : 64'h40; // -normal / +normal
        end
    endfunction

    function [63:0] dclass;
        input [63:0] v;
        reg s; reg [10:0] e; reg [51:0] m;
        begin
            s = v[63]; e = v[62:52]; m = v[51:0];
            if (e == 11'h7FF && m != 0)
                dclass = v[51] ? 64'h200 : 64'h100;
            else if (e == 11'h7FF)
                dclass = s ? 64'h1 : 64'h80;
            else if (e == 11'd0 && m == 0)
                dclass = s ? 64'h8 : 64'h10;
            else if (e == 11'd0)
                dclass = s ? 64'h4 : 64'h20;
            else
                dclass = s ? 64'h2 : 64'h40;
        end
    endfunction

    // Float -> integer conversion with RISC-V's defined out-of-range
    // behavior (saturating, not trapping): NaN and too-large-positive ->
    // max value for the target type; too-negative -> min value (or 0 for
    // an unsigned target). The fractional bits being discarded are
    // truncated toward zero rather than round-to-nearest -- a documented
    // simplification (see README): correctly-rounded conversion needs the
    // same guard/sticky-bit machinery as f64_to_f32, and this core's `rm`
    // field is ignored everywhere else too, so truncation here is
    // consistent with "RNE only, and only where it's essentially free."
    function [63:0] f32_to_int;
        input [31:0] v;
        input signed_result;
        input is_word;
        reg s; reg [7:0] e; reg [22:0] m;
        reg [63:0] maxpos, minneg, maxu;
        reg [23:0] sig;
        integer shift;
        reg [63:0] mag;
        begin
            s = v[31]; e = v[30:23]; m = v[22:0];
            maxpos = is_word ? 64'h000000007FFFFFFF : 64'h7FFFFFFFFFFFFFFF;
            minneg = is_word ? 64'hFFFFFFFF80000000 : 64'h8000000000000000;
            maxu   = is_word ? 64'h00000000FFFFFFFF : 64'hFFFFFFFFFFFFFFFF;

            if (is_nan32(v)) begin
                f32_to_int = signed_result ? maxpos : maxu;
            end else if (e == 8'hFF) begin // +-infinity
                if (signed_result) f32_to_int = s ? minneg : maxpos;
                else f32_to_int = s ? 64'd0 : maxu;
            end else if (e == 8'h00 && m == 23'b0) begin
                f32_to_int = 64'd0;
            end else begin
                if (e == 8'h00) begin
                    sig = {1'b0, m};
                    shift = -149; // value = m * 2^(-126-23)
                end else begin
                    sig = {1'b1, m};
                    shift = ({24'b0, e} - 13'd150); // (e-127)-23
                end
                if (shift >= 0)
                    mag = (shift >= 40) ? 64'hFFFFFFFFFFFFFFFF : ({40'b0, sig} << shift);
                else if (-shift >= 24)
                    mag = 64'd0;
                else
                    mag = sig >> (-shift);

                if (signed_result) begin
                    if (s) f32_to_int = (mag > (is_word ? 64'h80000000 : 64'h8000000000000000)) ? minneg : (~mag + 1'b1);
                    else   f32_to_int = (mag > (is_word ? 64'h7FFFFFFF : 64'h7FFFFFFFFFFFFFFF)) ? maxpos : mag;
                end else begin
                    if (s) f32_to_int = 64'd0;
                    else   f32_to_int = (mag > maxu) ? maxu : mag;
                end
            end
        end
    endfunction

    // Same idea as f32_to_int, for a float64 source.
    function [63:0] d64_to_int;
        input [63:0] v;
        input signed_result;
        input is_word;
        reg s; reg [10:0] e; reg [51:0] m;
        reg [63:0] maxpos, minneg, maxu;
        reg [52:0] sig;
        integer shift;
        reg [127:0] wide;
        reg [63:0] mag;
        begin
            s = v[63]; e = v[62:52]; m = v[51:0];
            maxpos = is_word ? 64'h000000007FFFFFFF : 64'h7FFFFFFFFFFFFFFF;
            minneg = is_word ? 64'hFFFFFFFF80000000 : 64'h8000000000000000;
            maxu   = is_word ? 64'h00000000FFFFFFFF : 64'hFFFFFFFFFFFFFFFF;

            if (is_nan64(v)) begin
                d64_to_int = signed_result ? maxpos : maxu;
            end else if (e == 11'h7FF) begin
                if (signed_result) d64_to_int = s ? minneg : maxpos;
                else d64_to_int = s ? 64'd0 : maxu;
            end else if (e == 11'd0 && m == 52'd0) begin
                d64_to_int = 64'd0;
            end else begin
                if (e == 11'd0) begin
                    sig = {1'b0, m};
                    shift = -1074; // value = m * 2^(-1022-52)
                end else begin
                    sig = {1'b1, m};
                    shift = ({22'b0, e} - 13'd1075); // (e-1023)-52
                end
                if (shift >= 0) begin
                    wide = (shift >= 64) ? {128{1'b1}} : ({75'b0, sig} << shift);
                    mag = (|wide[127:64]) ? 64'hFFFFFFFFFFFFFFFF : wide[63:0];
                end else if (-shift >= 53) begin
                    mag = 64'd0;
                end else begin
                    mag = sig >> (-shift);
                end

                if (signed_result) begin
                    if (s) d64_to_int = (mag > (is_word ? 64'h80000000 : 64'h8000000000000000)) ? minneg : (~mag + 1'b1);
                    else   d64_to_int = (mag > (is_word ? 64'h7FFFFFFF : 64'h7FFFFFFFFFFFFFFF)) ? maxpos : mag;
                end else begin
                    if (s) d64_to_int = 64'd0;
                    else   d64_to_int = (mag > maxu) ? maxu : mag;
                end
            end
        end
    endfunction
endmodule
