`timescale 1ns / 1ps
// Elementwise vector ALU: LANES lanes of 32-bit elements (VLEN=LANES*32,
// SEW=32, LMUL=1 -- the only SEW/LMUL configuration this scoped RVV
// implementation supports, though LANES/VLEN itself is a genuine
// synthesis-time parameter -- see riscv64_proc.v's own LANES parameter
// and RV64I/README.md's "configurable vector width" section).
// `operand2` is pre-selected at the top level to be either vs1's vector
// data (.vv), a scalar broadcast into all LANES lanes (.vx), or a
// sign-extended 5-bit immediate broadcast into all LANES lanes (.vi) --
// this module doesn't need to know which form the instruction used.
//
// `vd_result` holds full per-lane 32-bit results (arithmetic ops); for
// compare ops (which produce a 1-bit-per-element mask, not a 32-bit
// value), `cmp_bits` holds the LANES individual comparison booleans --
// riscv64_proc.v packs those into a mask register's low bits instead of
// using vd_result directly. Reductions are a different execution model
// (fold across lanes into one scalar) and are computed separately at the
// top level, reusing this module's op encoding but not its datapath.
module vector_alu #(
    parameter LANES = 4,
    parameter VLEN = LANES * 32
)(
    input [VLEN-1:0] vs2_data,
    input [VLEN-1:0] operand2,
    input [4:0] v_op,
    output reg [VLEN-1:0] vd_result,
    output reg [LANES-1:0] cmp_bits
);
    localparam VADD=5'd0, VSUB=5'd1, VAND=5'd2, VOR=5'd3, VXOR=5'd4, VMUL=5'd5,
               VSEQ=5'd6, VSNE=5'd7, VSLT=5'd8, VSLTU=5'd9, VSLE=5'd10, VSLEU=5'd11,
               VMIN=5'd12, VMINU=5'd13, VMAX=5'd14, VMAXU=5'd15,
               VDIVU=5'd16, VDIV=5'd17, VREMU=5'd18, VREM=5'd19,
               VSLL=5'd20, VSRL=5'd21, VSRA=5'd22;

    genvar i;
    generate
        for (i = 0; i < LANES; i = i + 1) begin : lane
            wire [31:0] a = vs2_data[i*32 +: 32];
            wire [31:0] b = operand2[i*32 +: 32];
            reg  [31:0] r;
            reg  cbit;
            // Signed divide/remainder computed into explicit `signed` regs
            // *before* the divide-by-zero/overflow ternary, not inline
            // inside it: this Icarus build silently drops the $signed()
            // cast's effect on `/`/`%` when they're nested directly inside
            // a ternary expression (confirmed with a 2-line isolated probe
            // -- (-20)/4 came back as 0x3FFFFFFB, i.e. computed as
            // *unsigned* division, instead of the correct 0xFFFFFFFB).
            // Same family of bug as the `~int_a[31:0]+1'b1`-in-a-ternary
            // issue documented in the README for the FPU; the fix is the
            // same pattern: hoist the signed operation out to its own reg.
            reg signed [31:0] sdiv, srem;
            always @(*) begin
                sdiv = $signed(a) / $signed(b);
                srem = $signed(a) % $signed(b);
                case (v_op)
                    VADD:  r = a + b;
                    VSUB:  r = a - b;
                    VAND:  r = a & b;
                    VOR:   r = a | b;
                    VXOR:  r = a ^ b;
                    VMUL:  r = a * b; // low 32 bits of the product
                    VMIN:  r = ($signed(a) < $signed(b)) ? a : b;
                    VMINU: r = (a < b) ? a : b;
                    VMAX:  r = ($signed(a) > $signed(b)) ? a : b;
                    VMAXU: r = (a > b) ? a : b;
                    // Divide-by-zero and signed-overflow results are
                    // defined by the RVV spec (not a trap): dividing by
                    // zero yields all-ones (unsigned) or -1 (signed) for
                    // the quotient, and the dividend itself for the
                    // remainder; the one signed-overflow case (MIN_INT /
                    // -1) yields MIN_INT for the quotient and 0 for the
                    // remainder.
                    VDIVU: r = (b == 32'b0) ? 32'hFFFFFFFF : (a / b);
                    VDIV:  r = (b == 32'b0) ? 32'hFFFFFFFF :
                                (a == 32'h80000000 && b == 32'hFFFFFFFF) ? a :
                                sdiv;
                    VREMU: r = (b == 32'b0) ? a : (a % b);
                    VREM:  r = (b == 32'b0) ? a :
                                (a == 32'h80000000 && b == 32'hFFFFFFFF) ? 32'b0 :
                                srem;
                    // Shift amount is the low 5 bits of the operand
                    // (element-width mod, SEW=32 here) regardless of
                    // whether it arrived via .vv/.vx (a real value) or
                    // .vi (a sign-extended 5-bit immediate -- sign
                    // extension never touches bits[4:0], so this is safe
                    // either way).
                    VSLL:  r = a << b[4:0];
                    VSRL:  r = a >> b[4:0];
                    VSRA:  r = $signed(a) >>> b[4:0];
                    default: r = 32'b0;
                endcase
                case (v_op)
                    VSEQ:  cbit = (a == b);
                    VSNE:  cbit = (a != b);
                    VSLT:  cbit = ($signed(a) < $signed(b));
                    VSLTU: cbit = (a < b);
                    VSLE:  cbit = ($signed(a) <= $signed(b));
                    VSLEU: cbit = (a <= b);
                    default: cbit = 1'b0;
                endcase
            end
            always @(*) begin
                vd_result[i*32 +: 32] = r;
                cmp_bits[i] = cbit;
            end
        end
    endgenerate
endmodule
