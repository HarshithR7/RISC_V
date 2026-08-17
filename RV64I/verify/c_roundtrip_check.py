"""
Python port of compressed_decoder.v (mechanical transcription), used to
round-trip-check every asm64.py compressed encoder before burning RTL
simulation cycles debugging bit-layout mistakes.
"""
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
import asm64 as A  # noqa: E402


def sext(v, bits):
    v &= (1 << bits) - 1
    if v & (1 << (bits - 1)):
        v -= (1 << bits)
    return v


def decode(c16):
    c = c16 & 0xFFFF
    op = c & 0x3
    f3 = (c >> 13) & 0x7

    def bit(n):
        return (c >> n) & 1

    def bits(hi, lo):
        return (c >> lo) & ((1 << (hi - lo + 1)) - 1)

    rd_rs1_5 = bits(11, 7)
    rs2_5 = bits(6, 2)
    rd3 = 8 + bits(4, 2)
    rs1p = 8 + bits(9, 7)
    rs2p = 8 + bits(4, 2)

    if op == 0b00:
        if f3 == 0b000:
            imm = (bits(10, 7) << 6) | (bits(12, 11) << 4) | (bit(5) << 3) | (bit(6) << 2)
            return ("addi4spn", rd3, 2, imm)
        if f3 == 0b010:
            off = (bit(5) << 6) | (bits(12, 10) << 3) | (bit(6) << 2)
            return ("lw", rs2p, rs1p, off)
        if f3 == 0b011:
            off = (bits(6, 5) << 6) | (bits(12, 10) << 3)
            return ("ld", rs2p, rs1p, off)
        if f3 == 0b110:
            off = (bit(5) << 6) | (bits(12, 10) << 3) | (bit(6) << 2)
            return ("sw", rs2p, rs1p, off)
        if f3 == 0b111:
            off = (bits(6, 5) << 6) | (bits(12, 10) << 3)
            return ("sd", rs2p, rs1p, off)
    elif op == 0b01:
        if f3 == 0b000:
            imm = sext((bit(12) << 5) | bits(6, 2), 6)
            return ("addi", rd_rs1_5, imm)
        if f3 == 0b001:
            imm = sext((bit(12) << 5) | bits(6, 2), 6)
            return ("addiw", rd_rs1_5, imm)
        if f3 == 0b010:
            imm = sext((bit(12) << 5) | bits(6, 2), 6)
            return ("li", rd_rs1_5, imm)
        if f3 == 0b011:
            if rd_rs1_5 == 2:
                imm = sext((bit(12) << 9) | (bits(4, 3) << 7) | (bit(5) << 6) |
                           (bit(2) << 5) | (bit(6) << 4), 10)
                return ("addi16sp", imm)
            else:
                # bit 17 is the sign bit of this 18-bit field (imm[17:12]
                # holds the 6 significant bits, imm[19:18] mirror bit 17 in
                # the real 20-bit U-immediate but aren't reproduced here).
                imm = sext((bit(12) << 17) | (bits(6, 2) << 12), 18)
                return ("lui", rd_rs1_5, imm)
        if f3 == 0b100:
            grp = bits(11, 10)
            if grp == 0b00:
                sh = (bit(12) << 5) | bits(6, 2)
                return ("srli", rs1p, sh)
            if grp == 0b01:
                sh = (bit(12) << 5) | bits(6, 2)
                return ("srai", rs1p, sh)
            if grp == 0b10:
                imm = sext((bit(12) << 5) | bits(6, 2), 6)
                return ("andi", rs1p, imm)
            if grp == 0b11:
                sub = bits(6, 5)
                names0 = {0: "sub", 1: "xor", 2: "or", 3: "and"}
                names1 = {0: "subw", 1: "addw"}
                if bit(12) == 0:
                    return (names0[sub], rs1p, rs2p)
                else:
                    return (names1.get(sub, "reserved"), rs1p, rs2p)
        if f3 == 0b101:
            off = sext((bit(12) << 11) | (bit(8) << 10) | (bits(10, 9) << 8) |
                       (bit(6) << 7) | (bit(7) << 6) | (bit(2) << 5) |
                       (bit(11) << 4) | (bits(5, 3) << 1), 12)
            return ("j", off)
        if f3 == 0b110:
            off = sext((bit(12) << 8) | (bits(6, 5) << 6) | (bit(2) << 5) |
                       (bits(11, 10) << 3) | (bits(4, 3) << 1), 9)
            return ("beqz", rs1p, off)
        if f3 == 0b111:
            off = sext((bit(12) << 8) | (bits(6, 5) << 6) | (bit(2) << 5) |
                       (bits(11, 10) << 3) | (bits(4, 3) << 1), 9)
            return ("bnez", rs1p, off)
    elif op == 0b10:
        if f3 == 0b000:
            sh = (bit(12) << 5) | bits(6, 2)
            return ("slli", rd_rs1_5, sh)
        if f3 == 0b010:
            off = (bits(3, 2) << 6) | (bit(12) << 5) | (bits(6, 4) << 2)
            return ("lwsp", rd_rs1_5, off)
        if f3 == 0b011:
            off = (bits(4, 2) << 6) | (bit(12) << 5) | (bits(6, 5) << 3)
            return ("ldsp", rd_rs1_5, off)
        if f3 == 0b100:
            if bit(12) == 0:
                if rs2_5 == 0:
                    return ("jr", rd_rs1_5)
                else:
                    return ("mv", rd_rs1_5, rs2_5)
            else:
                if rd_rs1_5 == 0 and rs2_5 == 0:
                    return ("ebreak",)
                elif rs2_5 == 0:
                    return ("jalr", rd_rs1_5)
                else:
                    return ("add", rd_rs1_5, rs2_5)
        if f3 == 0b110:
            off = (bits(8, 7) << 6) | (bits(12, 9) << 2)
            return ("swsp", rs2_5, off)
        if f3 == 0b111:
            off = (bits(9, 7) << 6) | (bits(12, 10) << 3)
            return ("sdsp", rs2_5, off)
    return ("UNKNOWN",)


def check(desc, encoded, expected):
    got = decode(encoded)
    ok = got == expected
    print(f"{'OK  ' if ok else 'FAIL'} {desc:24s} enc={encoded:04x} got={got} expected={expected}")
    return ok


def main():
    fails = 0
    fails += not check("c.addi4spn x8,16", A.c_addi4spn(8, 16), ("addi4spn", 8, 2, 16))
    fails += not check("c.lw x9,4(x8)", A.c_lw_sw(0b00, 0b010, 9, 8, 4), ("lw", 9, 8, 4))
    fails += not check("c.sw x9,4(x8)", A.c_lw_sw(0b00, 0b110, 9, 8, 4), ("sw", 9, 8, 4))
    fails += not check("c.ld x9,8(x8)", A.c_ld_sd(0b00, 0b011, 9, 8, 8), ("ld", 9, 8, 8))
    fails += not check("c.sd x9,8(x8)", A.c_ld_sd(0b00, 0b111, 9, 8, 8), ("sd", 9, 8, 8))
    fails += not check("c.addi x5,10", A.c_i_imm6(0b000, 5, 10), ("addi", 5, 10))
    fails += not check("c.addi x5,-10", A.c_i_imm6(0b000, 5, -10), ("addi", 5, -10))
    fails += not check("c.addiw x5,10", A.c_i_imm6(0b001, 5, 10), ("addiw", 5, 10))
    fails += not check("c.li x5,10", A.c_i_imm6(0b010, 5, 10), ("li", 5, 10))
    fails += not check("c.li x5,-10", A.c_i_imm6(0b010, 5, -10), ("li", 5, -10))
    fails += not check("c.addi16sp -32", A.c_addi16sp(-32), ("addi16sp", -32))
    fails += not check("c.addi16sp 48", A.c_addi16sp(48), ("addi16sp", 48))
    fails += not check("c.addi16sp -496", A.c_addi16sp(-496), ("addi16sp", -496))
    fails += not check("c.lui x5,5", A.c_lui(5, 5 << 12), ("lui", 5, 5 << 12))
    fails += not check("c.lui x5,-5", A.c_lui(5, (-5 & 0x3F) << 12), ("lui", 5, sext((-5 & 0x3F) << 12, 18)))
    fails += not check("c.srli x8,4", A.c_shift_imm(8, 0b00, 4), ("srli", 8, 4))
    fails += not check("c.srai x8,4", A.c_shift_imm(8, 0b01, 4), ("srai", 8, 4))
    fails += not check("c.srli x8,40", A.c_shift_imm(8, 0b00, 40), ("srli", 8, 40))
    fails += not check("c.andi x8,15", A.c_andi(8, 15), ("andi", 8, 15))
    fails += not check("c.andi x8,-1", A.c_andi(8, -1), ("andi", 8, -1))
    fails += not check("c.sub x8,x9", A.c_ra_ra(8, 9, 0, 0b00), ("sub", 8, 9))
    fails += not check("c.xor x8,x9", A.c_ra_ra(8, 9, 0, 0b01), ("xor", 8, 9))
    fails += not check("c.or x8,x9", A.c_ra_ra(8, 9, 0, 0b10), ("or", 8, 9))
    fails += not check("c.and x8,x9", A.c_ra_ra(8, 9, 0, 0b11), ("and", 8, 9))
    fails += not check("c.subw x8,x9", A.c_ra_ra(8, 9, 1, 0b00), ("subw", 8, 9))
    fails += not check("c.addw x8,x9", A.c_ra_ra(8, 9, 1, 0b01), ("addw", 8, 9))
    fails += not check("c.j +100", A.c_j(100), ("j", 100))
    fails += not check("c.j -100", A.c_j(-100), ("j", -100))
    fails += not check("c.j +2000", A.c_j(2000), ("j", 2000))
    fails += not check("c.j -2000", A.c_j(-2000), ("j", -2000))
    fails += not check("c.beqz x9,+20", A.c_branch(0b110, 9, 20), ("beqz", 9, 20))
    fails += not check("c.beqz x9,-20", A.c_branch(0b110, 9, -20), ("beqz", 9, -20))
    fails += not check("c.beqz x9,+250", A.c_branch(0b110, 9, 250), ("beqz", 9, 250))
    fails += not check("c.bnez x9,-250", A.c_branch(0b111, 9, -250), ("bnez", 9, -250))
    fails += not check("c.lwsp x5,32", A.c_lwsp(5, 32), ("lwsp", 5, 32))
    fails += not check("c.lwsp x5,252", A.c_lwsp(5, 252), ("lwsp", 5, 252))
    fails += not check("c.ldsp x5,40", A.c_ldsp(5, 40), ("ldsp", 5, 40))
    fails += not check("c.ldsp x5,504", A.c_ldsp(5, 504), ("ldsp", 5, 504))
    fails += not check("c.swsp x5,32", A.c_swsp(5, 32), ("swsp", 5, 32))
    fails += not check("c.sdsp x5,40", A.c_sdsp(5, 40), ("sdsp", 5, 40))
    fails += not check("c.slli x5,10", A.c_slli(5, 10), ("slli", 5, 10))
    fails += not check("c.slli x5,40", A.c_slli(5, 40), ("slli", 5, 40))
    fails += not check("c.jr x5", A.c_cr(0b1000, 5, 0), ("jr", 5))
    fails += not check("c.mv x5,x6", A.c_cr(0b1000, 5, 6), ("mv", 5, 6))
    fails += not check("c.jalr x5", A.c_cr(0b1001, 5, 0), ("jalr", 5))
    fails += not check("c.add x5,x6", A.c_cr(0b1001, 5, 6), ("add", 5, 6))
    fails += not check("c.ebreak", A.c_cr(0b1001, 0, 0), ("ebreak",))

    print(f"\n{'ALL PASS' if fails == 0 else f'{fails} FAILURES'}")


if __name__ == "__main__":
    main()
