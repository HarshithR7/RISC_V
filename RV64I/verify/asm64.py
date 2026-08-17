"""
Minimal RV64I assembler used to generate .mem files for testing the RV64I
core in RISC_V/RV64I/src/. Extends the RV32I assembler
(RISC_V/verify/asm.py) with: LD/SD/LWU, the OP-IMM-32 word-ops
(ADDIW/SLLIW/SRLIW/SRAIW), the OP-32 word-ops (ADDW/SUBW/SLLW/SRLW/SRAW),
and 6-bit shift amounts for the 64-bit shift instructions.
"""
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "verify"))
from asm import (   # noqa: E402
    REGS, reg, imm, r_type, i_type, s_type, b_type, u_type, j_type,
    parse_mem_operand,
)

OPCODE_R      = 0b0110011
OPCODE_I      = 0b0010011
OPCODE_LOAD   = 0b0000011
OPCODE_S      = 0b0100011
OPCODE_B      = 0b1100011
OPCODE_JAL    = 0b1101111
OPCODE_JALR   = 0b1100111
OPCODE_LUI    = 0b0110111
OPCODE_AUIPC  = 0b0010111
OPCODE_SYSTEM = 0b1110011
OPCODE_OP_IMM_32 = 0b0011011
OPCODE_OP_32     = 0b0111011

R_OPS = {
    "add":  (0b000, 0b0000000), "sub":  (0b000, 0b0100000),
    "sll":  (0b001, 0b0000000), "slt":  (0b010, 0b0000000),
    "sltu": (0b011, 0b0000000), "xor":  (0b100, 0b0000000),
    "srl":  (0b101, 0b0000000), "sra":  (0b101, 0b0100000),
    "or":   (0b110, 0b0000000), "and":  (0b111, 0b0000000),
}
OP32_OPS = {
    "addw": (0b000, 0b0000000), "subw": (0b000, 0b0100000),
    "sllw": (0b001, 0b0000000), "srlw": (0b101, 0b0000000),
    "sraw": (0b101, 0b0100000),
}
RVM_FUNCT7 = 0b0000001
M_OPS = {  # func3 order matches the RISC-V M-extension encoding directly
    "mul": 0b000, "mulh": 0b001, "mulhsu": 0b010, "mulhu": 0b011,
    "div": 0b100, "divu": 0b101, "rem": 0b110, "remu": 0b111,
}
M_OPS_32 = {  # OP-32 opcode: no MULHW/MULHSUW/MULHUW in the spec
    "mulw": 0b000, "divw": 0b100, "divuw": 0b101, "remw": 0b110, "remuw": 0b111,
}

OPCODE_AMO = 0b0101111
AMO_FUNCT5_LR = 0b00010
AMO_FUNCT5_SC = 0b00011
A_OPS_LR = {"lr.w": 0b010, "lr.d": 0b011}
A_OPS_SC = {"sc.w": 0b010, "sc.d": 0b011}
A_OPS_AMO = {
    "amoswap.w": (0b010, 0b00001), "amoswap.d": (0b011, 0b00001),
    "amoadd.w":  (0b010, 0b00000), "amoadd.d":  (0b011, 0b00000),
    "amoxor.w":  (0b010, 0b00100), "amoxor.d":  (0b011, 0b00100),
    "amoand.w":  (0b010, 0b01100), "amoand.d":  (0b011, 0b01100),
    "amoor.w":   (0b010, 0b01000), "amoor.d":   (0b011, 0b01000),
    "amomin.w":  (0b010, 0b10000), "amomin.d":  (0b011, 0b10000),
    "amomax.w":  (0b010, 0b10100), "amomax.d":  (0b011, 0b10100),
    "amominu.w": (0b010, 0b11000), "amominu.d": (0b011, 0b11000),
    "amomaxu.w": (0b010, 0b11100), "amomaxu.d": (0b011, 0b11100),
}


def a_type(opcode, funct3, funct5, rd, rs1, rs2, aq=0, rl=0):
    return (funct5 << 27) | (aq << 26) | (rl << 25) | (rs2 << 20) | \
           (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode


def parse_paren_reg(s):
    m = re.match(r"\s*\(\s*(\w+)\s*\)\s*", s.strip())
    if not m:
        raise ValueError(f"bad (reg) operand '{s}' (no offset allowed for A-extension ops)")
    return m.group(1)


# ---------------------------------------------------------------------------
# F extension (single-precision floating point)
# ---------------------------------------------------------------------------
FREGS = {f"f{i}": i for i in range(32)}


def freg(x):
    x = x.strip().rstrip(',')
    if x not in FREGS:
        raise ValueError(f"unknown float register '{x}'")
    return FREGS[x]


OPCODE_LOAD_FP  = 0b0000111
OPCODE_STORE_FP = 0b0100111
OPCODE_MADD  = 0b1000011
OPCODE_MSUB  = 0b1000111
OPCODE_NMSUB = 0b1001011
OPCODE_NMADD = 0b1001111
OPCODE_OP_FP = 0b1010011

F_R4_OPS = {"fmadd.s": (OPCODE_MADD, 0b00), "fmsub.s": (OPCODE_MSUB, 0b00),
            "fnmsub.s": (OPCODE_NMSUB, 0b00), "fnmadd.s": (OPCODE_NMADD, 0b00),
            "fmadd.d": (OPCODE_MADD, 0b01), "fmsub.d": (OPCODE_MSUB, 0b01),
            "fnmsub.d": (OPCODE_NMSUB, 0b01), "fnmadd.d": (OPCODE_NMADD, 0b01)}
# name -> (funct7, funct3_or_None, rs2_or_None) -- D's funct7 is always F's
# with the LSB set (e.g. FADD.S=0000000 -> FADD.D=0000001).
F_OP_OPS = {
    "fadd.s": (0b0000000, None, None), "fsub.s": (0b0000100, None, None),
    "fmul.s": (0b0001000, None, None), "fdiv.s": (0b0001100, None, None),
    "fsqrt.s": (0b0101100, None, 0b00000),
    "fsgnj.s": (0b0010000, 0b000, None), "fsgnjn.s": (0b0010000, 0b001, None),
    "fsgnjx.s": (0b0010000, 0b010, None),
    "fmin.s": (0b0010100, 0b000, None), "fmax.s": (0b0010100, 0b001, None),
    "fadd.d": (0b0000001, None, None), "fsub.d": (0b0000101, None, None),
    "fmul.d": (0b0001001, None, None), "fdiv.d": (0b0001101, None, None),
    "fsqrt.d": (0b0101101, None, 0b00000),
    "fsgnj.d": (0b0010001, 0b000, None), "fsgnjn.d": (0b0010001, 0b001, None),
    "fsgnjx.d": (0b0010001, 0b010, None),
    "fmin.d": (0b0010101, 0b000, None), "fmax.d": (0b0010101, 0b001, None),
}
F_CMP_OPS = {"feq.s": (0b010, 0b1010000), "flt.s": (0b001, 0b1010000), "fle.s": (0b000, 0b1010000),
             "feq.d": (0b010, 0b1010001), "flt.d": (0b001, 0b1010001), "fle.d": (0b000, 0b1010001)}
F_TO_INT_OPS = {"fcvt.w.s": (0b00000, 0b1100000), "fcvt.wu.s": (0b00001, 0b1100000),
                "fcvt.l.s": (0b00010, 0b1100000), "fcvt.lu.s": (0b00011, 0b1100000),
                "fcvt.w.d": (0b00000, 0b1100001), "fcvt.wu.d": (0b00001, 0b1100001),
                "fcvt.l.d": (0b00010, 0b1100001), "fcvt.lu.d": (0b00011, 0b1100001)}
F_FROM_INT_OPS = {"fcvt.s.w": (0b00000, 0b1101000), "fcvt.s.wu": (0b00001, 0b1101000),
                   "fcvt.s.l": (0b00010, 0b1101000), "fcvt.s.lu": (0b00011, 0b1101000),
                   "fcvt.d.w": (0b00000, 0b1101001), "fcvt.d.wu": (0b00001, 0b1101001),
                   "fcvt.d.l": (0b00010, 0b1101001), "fcvt.d.lu": (0b00011, 0b1101001)}
F_CLASS_OPS = {"fclass.s": 0b1110000, "fclass.d": 0b1110001}
F_MV_X_OPS = {"fmv.x.w": 0b1110000, "fmv.x.d": 0b1110001}
F_MV_FROM_X_OPS = {"fmv.w.x": 0b1111000, "fmv.d.x": 0b1111001}


def r4_type(opcode, rm, rs3, fmt, rs2, rs1, rd):
    return (rs3 << 27) | (fmt << 25) | (rs2 << 20) | (rs1 << 15) | \
           (rm << 12) | (rd << 7) | opcode


def r_type_fp(opcode, funct3, funct7, rd, rs1, rs2):
    return r_type(opcode, funct3, funct7, rd, rs1, rs2)


# ---------------------------------------------------------------------------
# RVV (scoped: VLEN=128, SEW=32, LMUL=1 only -- see RV64I/README.md)
# ---------------------------------------------------------------------------
OPCODE_OP_V = 0b1010111

V_FUNCT6 = {"vadd": 0b000000, "vsub": 0b000010, "vand": 0b001001,
            "vor": 0b001010, "vxor": 0b001011, "vmul": 0b100101,
            "vmseq": 0b011000, "vmsne": 0b011001, "vmslt": 0b011011,
            "vmsltu": 0b011010, "vmsle": 0b011101, "vmsleu": 0b011100,
            "vmin": 0b000101, "vminu": 0b000100, "vmax": 0b000111, "vmaxu": 0b000110,
            # Divide/remainder (OPMVV/OPMVX group, func3=010/110 -- same
            # group as vmul). Shifts (OPIVV/OPIVX/OPIVI group, func3=
            # 000/100/011 -- same group as vadd) reuse funct6=0b100101,
            # numerically identical to vmul's -- that's real RVV, not a
            # collision, since func3 disambiguates the two groups (see
            # decode_control_unit.v's OP_V case, which decodes these in
            # two separate funct6 tables for exactly this reason).
            "vdivu": 0b100000, "vdiv": 0b100001, "vremu": 0b100010, "vrem": 0b100011,
            "vsll": 0b100101, "vsrl": 0b101000, "vsra": 0b101001}
# form -> func3. VMUL/VMIN*/VMAX*/VDIV*/VREM* have no .vi form (matches
# real RVV); this scoped implementation gives every compare a .vi form
# too, for simplicity (real RVV omits .vi for vmslt/vmsltu specifically --
# a stated deviation).
V_FORM_FUNC3 = {
    ("vadd", "vv"): 0b000, ("vadd", "vx"): 0b100, ("vadd", "vi"): 0b011,
    ("vsub", "vv"): 0b000, ("vsub", "vx"): 0b100,
    ("vand", "vv"): 0b000, ("vand", "vx"): 0b100, ("vand", "vi"): 0b011,
    ("vor", "vv"): 0b000, ("vor", "vx"): 0b100, ("vor", "vi"): 0b011,
    ("vxor", "vv"): 0b000, ("vxor", "vx"): 0b100, ("vxor", "vi"): 0b011,
    ("vmul", "vv"): 0b010, ("vmul", "vx"): 0b110,
    ("vmseq", "vv"): 0b000, ("vmseq", "vx"): 0b100, ("vmseq", "vi"): 0b011,
    ("vmsne", "vv"): 0b000, ("vmsne", "vx"): 0b100, ("vmsne", "vi"): 0b011,
    ("vmslt", "vv"): 0b000, ("vmslt", "vx"): 0b100, ("vmslt", "vi"): 0b011,
    ("vmsltu", "vv"): 0b000, ("vmsltu", "vx"): 0b100, ("vmsltu", "vi"): 0b011,
    ("vmsle", "vv"): 0b000, ("vmsle", "vx"): 0b100, ("vmsle", "vi"): 0b011,
    ("vmsleu", "vv"): 0b000, ("vmsleu", "vx"): 0b100, ("vmsleu", "vi"): 0b011,
    ("vmin", "vv"): 0b000, ("vmin", "vx"): 0b100,
    ("vminu", "vv"): 0b000, ("vminu", "vx"): 0b100,
    ("vmax", "vv"): 0b000, ("vmax", "vx"): 0b100,
    ("vmaxu", "vv"): 0b000, ("vmaxu", "vx"): 0b100,
    ("vdivu", "vv"): 0b010, ("vdivu", "vx"): 0b110,
    ("vdiv", "vv"): 0b010, ("vdiv", "vx"): 0b110,
    ("vremu", "vv"): 0b010, ("vremu", "vx"): 0b110,
    ("vrem", "vv"): 0b010, ("vrem", "vx"): 0b110,
    ("vsll", "vv"): 0b000, ("vsll", "vx"): 0b100, ("vsll", "vi"): 0b011,
    ("vsrl", "vv"): 0b000, ("vsrl", "vx"): 0b100, ("vsrl", "vi"): 0b011,
    ("vsra", "vv"): 0b000, ("vsra", "vx"): 0b100, ("vsra", "vi"): 0b011,
}
# vredOP.vs vd, vs2, vs1 -- OPMVV (func3=010), funct6 selects the reduce op.
V_RED_FUNCT6 = {"vredsum": 0b000000, "vredand": 0b000001, "vredor": 0b000010,
                "vredxor": 0b000011, "vredminu": 0b000100, "vredmin": 0b000101,
                "vredmaxu": 0b000110, "vredmax": 0b000111}


def v_type(funct6, vm, vs2, rs1_field, funct3, vd):
    return (funct6 << 26) | (vm << 25) | (vs2 << 20) | (rs1_field << 15) | \
           (funct3 << 12) | (vd << 7) | OPCODE_OP_V


def vreg(x):
    x = x.strip().rstrip(',')
    if not (x.startswith('v') and x[1:].isdigit() and 0 <= int(x[1:]) <= 31):
        raise ValueError(f"unknown vector register '{x}'")
    return int(x[1:])


# ---------------------------------------------------------------------------
# RV64C (compressed) encoders. Each mirrors the corresponding case in
# compressed_decoder.v exactly, in reverse: given the fields a real 32-bit
# instruction would carry, scatter them into the 16-bit compressed layout.
# ---------------------------------------------------------------------------

def creg(r):
    """3-bit compressed register field: only x8-x15 are encodable."""
    if not (8 <= r <= 15):
        raise ValueError(f"x{r} is not compressed-encodable (must be x8-x15)")
    return r - 8


def c_field(v, bits, signed=True):
    mask = (1 << bits) - 1
    lo = -(1 << (bits - 1)) if signed else 0
    hi = (1 << (bits - 1)) - 1 if signed else mask
    if not (lo <= v <= hi):
        raise ValueError(f"value {v} does not fit in a {bits}-bit "
                          f"{'signed' if signed else 'unsigned'} compressed field ({lo}..{hi})")
    return v & mask


def c_addi4spn(rd, nzuimm):
    u = c_field(nzuimm, 10, signed=False)
    return (0b000 << 13) | (((u >> 4) & 0x3) << 11) | (((u >> 6) & 0xF) << 7) | \
           (((u >> 2) & 0x1) << 6) | (((u >> 3) & 0x1) << 5) | (creg(rd) << 2) | 0b00


def c_lw_sw(op2, f3, rd_or_rs2, rs1, offv):
    o = c_field(offv, 7, signed=False)
    return (f3 << 13) | (((o >> 6) & 0x1) << 5) | (((o >> 3) & 0x7) << 10) | \
           (((o >> 2) & 0x1) << 6) | (creg(rs1) << 7) | (creg(rd_or_rs2) << 2) | op2


def c_ld_sd(op2, f3, rd_or_rs2, rs1, offv):
    o = c_field(offv, 8, signed=False)
    return (f3 << 13) | (((o >> 6) & 0x3) << 5) | (((o >> 3) & 0x7) << 10) | \
           (creg(rs1) << 7) | (creg(rd_or_rs2) << 2) | op2


def c_i_imm6(f3, rd_rs1, immv):
    v = c_field(immv, 6)
    return (f3 << 13) | (((v >> 5) & 0x1) << 12) | (rd_rs1 << 7) | ((v & 0x1F) << 2) | 0b01


def c_addi16sp(nzimm):
    # decoder: {{2{c[12]}},c[12],c[4:3],c[5],c[2],c[6],4'b0} -> imm[9]=c12,
    # imm[8:7]=c[4:3], imm[6]=c[5], imm[5]=c[2], imm[4]=c[6]
    v = c_field(nzimm, 10)  # multiple of 16, sign-extended range
    return (0b011 << 13) | (((v >> 9) & 0x1) << 12) | (2 << 7) | \
           (((v >> 7) & 0x3) << 3) | (((v >> 6) & 0x1) << 5) | (((v >> 5) & 0x1) << 2) | (((v >> 4) & 0x1) << 6) | 0b01


def c_lui(rd, nzimm18):
    v = c_field(nzimm18, 18, signed=False)
    return (0b011 << 13) | (((v >> 17) & 0x1) << 12) | (rd << 7) | (((v >> 12) & 0x1F) << 2) | 0b01


def c_shift_imm(rd_rs1_3, f2, shamt):
    s = c_field(shamt, 6, signed=False)
    return (0b100 << 13) | (((s >> 5) & 0x1) << 12) | (f2 << 10) | (creg(rd_rs1_3) << 7) | ((s & 0x1F) << 2) | 0b01


def c_andi(rd_rs1_3, immv):
    v = c_field(immv, 6)
    return (0b100 << 13) | (((v >> 5) & 0x1) << 12) | (0b10 << 10) | (creg(rd_rs1_3) << 7) | ((v & 0x1F) << 2) | 0b01


def c_ra_ra(rd_rs1_3, rs2_3, bit12, f2):
    return (0b100 << 13) | (bit12 << 12) | (0b11 << 10) | (creg(rd_rs1_3) << 7) | (f2 << 5) | (creg(rs2_3) << 2) | 0b01


def c_j(offv):
    # decoder: {{9{c12}},c12,c8,c[10:9],c6,c7,c2,c11,c[5:3],1'b0}
    # -> imm[10]=c12(sign,10 copies incl imm11), imm[10]=c8, imm[9:8]=c[10:9],
    #    imm[7]=c6, imm[6]=c7, imm[5]=c2, imm[4]=c11, imm[3:1]=c[5:3]
    o = c_field(offv, 12)  # bits[11:0], bit0 always 0
    c12 = (o >> 11) & 1
    c11 = (o >> 4) & 1
    c10_9 = (o >> 8) & 0x3
    c8 = (o >> 10) & 1
    c7 = (o >> 6) & 1
    c6 = (o >> 7) & 1
    c5_3 = (o >> 1) & 0x7
    c2 = (o >> 5) & 1
    return (0b101 << 13) | (c12 << 12) | (c11 << 11) | (c10_9 << 9) | (c8 << 8) | \
           (c7 << 7) | (c6 << 6) | (c5_3 << 3) | (c2 << 2) | 0b01


def c_branch(f3, rs1_3, offv):
    # decoder: {{4{c12}},c12,c[6:5],c2,c[11:10],c[4:3],1'b0}
    # -> imm[8]=c12(sign), imm[7:6]=c[6:5], imm[5]=c2, imm[4:3]=c[11:10],
    #    imm[2:1]=c[4:3]
    o = c_field(offv, 9)  # bits[8:0], bit0 always 0
    c12 = (o >> 8) & 1
    c6_5 = (o >> 6) & 0x3
    c2 = (o >> 5) & 1
    c11_10 = (o >> 3) & 0x3
    c4_3 = (o >> 1) & 0x3
    return (f3 << 13) | (c12 << 12) | (c11_10 << 10) | (creg(rs1_3) << 7) | \
           (c6_5 << 5) | (c4_3 << 3) | (c2 << 2) | 0b01


def c_lwsp(rd, offv):
    o = c_field(offv, 8, signed=False)
    return (0b010 << 13) | (((o >> 5) & 1) << 12) | (rd << 7) | \
           (((o >> 2) & 0x7) << 4) | (((o >> 6) & 0x3) << 2) | 0b10


def c_ldsp(rd, offv):
    o = c_field(offv, 9, signed=False)
    return (0b011 << 13) | (((o >> 5) & 1) << 12) | (rd << 7) | \
           (((o >> 3) & 0x3) << 5) | (((o >> 6) & 0x7) << 2) | 0b10


def c_swsp(rs2, offv):
    o = c_field(offv, 8, signed=False)
    return (0b110 << 13) | (((o >> 2) & 0xF) << 9) | (((o >> 6) & 0x3) << 7) | (rs2 << 2) | 0b10


def c_sdsp(rs2, offv):
    o = c_field(offv, 9, signed=False)
    return (0b111 << 13) | (((o >> 3) & 0x7) << 10) | (((o >> 6) & 0x7) << 7) | (rs2 << 2) | 0b10


def c_slli(rd, shamt):
    s = c_field(shamt, 6, signed=False)
    return (0b000 << 13) | (((s >> 5) & 1) << 12) | (rd << 7) | ((s & 0x1F) << 2) | 0b10


def c_cr(bit15_12, rd_rs1, rs2):
    return (bit15_12 << 12) | (rd_rs1 << 7) | (rs2 << 2) | 0b10
I_OPS = {
    "addi": 0b000, "slti": 0b010, "sltiu": 0b011, "xori": 0b100,
    "ori": 0b110, "andi": 0b111,
}
OPIMM32_OPS = {"addiw": 0b000}
SHIFT_I_OPS_64 = {"slli": (0b001, 0b000000), "srli": (0b101, 0b000000), "srai": (0b101, 0b010000)}
SHIFT_IW_OPS = {"slliw": (0b001, 0b0000000), "srliw": (0b101, 0b0000000), "sraiw": (0b101, 0b0100000)}
LOAD_OPS = {"lb": 0b000, "lh": 0b001, "lw": 0b010, "ld": 0b011, "lbu": 0b100, "lhu": 0b101, "lwu": 0b110}
STORE_OPS = {"sb": 0b000, "sh": 0b001, "sw": 0b010, "sd": 0b011}
BRANCH_OPS = {"beq": 0b000, "bne": 0b001, "blt": 0b100, "bge": 0b101, "bltu": 0b110, "bgeu": 0b111}


class Assembler64:
    def __init__(self):
        self.lines = []
        self.labels = {}

    def parse(self, text):
        for raw in text.splitlines():
            line = raw.split('#', 1)[0].strip()
            if not line:
                continue
            label = None
            if ':' in line:
                maybe_label, rest = line.split(':', 1)
                if re.match(r'^\w+$', maybe_label.strip()):
                    label = maybe_label.strip()
                    line = rest.strip()
            if not line:
                if label:
                    self.lines.append((label, None, []))
                continue
            parts = line.split(None, 1)
            mnem = parts[0].lower()
            args = [a.strip() for a in parts[1].split(',')] if len(parts) > 1 else []
            self.lines.append((label, mnem, args))

    @staticmethod
    def _instr_size(mnem):
        if mnem == "li":
            return 32  # 8 x 4-byte instructions
        if mnem.startswith("c."):
            return 2
        return 4

    def assemble(self, text):
        """Returns a list of (value, size_in_bytes) items -- size is 2 for
        an actual compressed instruction, 4 for a real 32-bit one (li's 8
        constituent instructions are each their own 4-byte item)."""
        self.parse(text)
        addr = 0
        expanded = []
        for label, mnem, args in self.lines:
            if label:
                self.labels[label] = addr
            if mnem is None:
                continue
            expanded.append((addr, mnem, args))
            addr += self._instr_size(mnem)

        items = []
        for addr, mnem, args in expanded:
            items.extend(self._encode(addr, mnem, args))
        return items

    @staticmethod
    def _build32_pair(rd, v32):
        """lui+addi sequence loading v32 into rd's low 32 bits (sign-extends
        into bits[63:32] if v32's bit 31 is set -- correct RV64I LUI
        semantics, but the caller must not rely on rd's upper bits until
        it has been explicitly masked/shifted, see _li64_words)."""
        v32 &= 0xFFFFFFFF
        upper = (v32 + 0x800) & 0xFFFFF000
        lower = v32 - upper
        lower &= 0xFFF
        if lower & 0x800:
            lower -= 0x1000
        return [u_type(OPCODE_LUI, rd, upper), i_type(OPCODE_I, 0b000, rd, rd, lower & 0xFFF)]

    def _li64_words(self, rd, v):
        """Builds an arbitrary 64-bit constant into rd, always as a fixed
        8-instruction sequence (matches the fixed expansion count used in
        the address-assignment pass): build the high and low 32-bit halves
        independently via _build32_pair, shift the high half into position,
        mask the low half back down to 32 bits (undoing any incidental
        sign-extension from its own lui+addi), then OR them together.
        Uses x28 (t3) as a scratch register -- avoid relying on x28 surviving
        across an `li` call."""
        SCRATCH = 28
        v &= 0xFFFFFFFFFFFFFFFF
        hi32 = (v >> 32) & 0xFFFFFFFF
        lo32 = v & 0xFFFFFFFF
        words = []
        words += self._build32_pair(rd, hi32)
        words.append(i_type(OPCODE_I, 0b001, rd, rd, (0b000000 << 6) | 32))       # slli rd, rd, 32
        words += self._build32_pair(SCRATCH, lo32)
        words.append(i_type(OPCODE_I, 0b001, SCRATCH, SCRATCH, (0b000000 << 6) | 32))  # slli scratch,scratch,32
        words.append(i_type(OPCODE_I, 0b101, SCRATCH, SCRATCH, (0b000000 << 6) | 32))  # srli scratch,scratch,32
        words.append(r_type(OPCODE_R, 0b110, 0b0000000, rd, rd, SCRATCH))         # or rd, rd, scratch
        return words

    def _resolve(self, label_or_imm, cur_addr):
        if label_or_imm in self.labels:
            return self.labels[label_or_imm] - cur_addr
        return int(label_or_imm, 0)

    def _encode(self, addr, mnem, args):
        if mnem == ".word":
            # Raw pre-encoded 32-bit value, e.g. lifted verbatim from a
            # real compiler's disassembly output -- lets a real toolchain's
            # machine code be spliced into an otherwise hand-written test
            # program (labels/jal targets around it resolve normally,
            # since this only affects _encode, not the two-pass address
            # bookkeeping in assemble()/_instr_size()).
            return [(int(args[0], 0) & 0xFFFFFFFF, 4)]

        if mnem.startswith("c."):
            return [(self._encode_compressed(addr, mnem, args), 2)]

        if mnem in R_OPS:
            f3, f7 = R_OPS[mnem]
            rd, rs1, rs2 = reg(args[0]), reg(args[1]), reg(args[2])
            return [(r_type(OPCODE_R, f3, f7, rd, rs1, rs2), 4)]
        if mnem in OP32_OPS:
            f3, f7 = OP32_OPS[mnem]
            rd, rs1, rs2 = reg(args[0]), reg(args[1]), reg(args[2])
            return [(r_type(OPCODE_OP_32, f3, f7, rd, rs1, rs2), 4)]
        if mnem in M_OPS:
            f3 = M_OPS[mnem]
            rd, rs1, rs2 = reg(args[0]), reg(args[1]), reg(args[2])
            return [(r_type(OPCODE_R, f3, RVM_FUNCT7, rd, rs1, rs2), 4)]
        if mnem in M_OPS_32:
            f3 = M_OPS_32[mnem]
            rd, rs1, rs2 = reg(args[0]), reg(args[1]), reg(args[2])
            return [(r_type(OPCODE_OP_32, f3, RVM_FUNCT7, rd, rs1, rs2), 4)]
        if mnem in A_OPS_LR:
            f3 = A_OPS_LR[mnem]
            rd = reg(args[0])
            rs1 = reg(parse_paren_reg(args[1]))
            return [(a_type(OPCODE_AMO, f3, AMO_FUNCT5_LR, rd, rs1, 0), 4)]
        if mnem in A_OPS_SC:
            f3 = A_OPS_SC[mnem]
            rd, rs2 = reg(args[0]), reg(args[1])
            rs1 = reg(parse_paren_reg(args[2]))
            return [(a_type(OPCODE_AMO, f3, AMO_FUNCT5_SC, rd, rs1, rs2), 4)]
        if mnem in A_OPS_AMO:
            f3, f5 = A_OPS_AMO[mnem]
            rd, rs2 = reg(args[0]), reg(args[1])
            rs1 = reg(parse_paren_reg(args[2]))
            return [(a_type(OPCODE_AMO, f3, f5, rd, rs1, rs2), 4)]
        if mnem == "flw":
            rd = freg(args[0])
            off, base = parse_mem_operand(args[1])
            return [(i_type(OPCODE_LOAD_FP, 0b010, rd, reg(base), imm(off, 12)), 4)]
        if mnem == "fsw":
            rs2 = freg(args[0])
            off, base = parse_mem_operand(args[1])
            return [(s_type(OPCODE_STORE_FP, 0b010, reg(base), rs2, imm(off, 12)), 4)]
        if mnem == "fld":
            rd = freg(args[0])
            off, base = parse_mem_operand(args[1])
            return [(i_type(OPCODE_LOAD_FP, 0b011, rd, reg(base), imm(off, 12)), 4)]
        if mnem == "fsd":
            rs2 = freg(args[0])
            off, base = parse_mem_operand(args[1])
            return [(s_type(OPCODE_STORE_FP, 0b011, reg(base), rs2, imm(off, 12)), 4)]
        if mnem in F_R4_OPS:
            opc, fmt = F_R4_OPS[mnem]
            rd, rs1, rs2, rs3 = freg(args[0]), freg(args[1]), freg(args[2]), freg(args[3])
            return [(r4_type(opc, 0b000, rs3, fmt, rs2, rs1, rd), 4)]  # rm=000 (RNE); only RNE is implemented
        if mnem in F_OP_OPS:
            f7, f3, rs2_fixed = F_OP_OPS[mnem]
            if rs2_fixed is not None:  # fsqrt.s/.d: rd, rs1 (rs2 field is fixed at 0)
                rd, rs1 = freg(args[0]), freg(args[1])
                rs2 = rs2_fixed
            else:
                rd, rs1, rs2 = freg(args[0]), freg(args[1]), freg(args[2])
            fn3 = f3 if f3 is not None else 0b000  # rm=000 (RNE) for arithmetic ops
            return [(r_type_fp(OPCODE_OP_FP, fn3, f7, rd, rs1, rs2), 4)]
        if mnem in F_CMP_OPS:
            f3, f7 = F_CMP_OPS[mnem]
            rd, rs1, rs2 = reg(args[0]), freg(args[1]), freg(args[2])
            return [(r_type_fp(OPCODE_OP_FP, f3, f7, rd, rs1, rs2), 4)]
        if mnem in F_TO_INT_OPS:
            rs2, f7 = F_TO_INT_OPS[mnem]
            rd, rs1 = reg(args[0]), freg(args[1])
            return [(r_type_fp(OPCODE_OP_FP, 0b000, f7, rd, rs1, rs2), 4)]
        if mnem in F_FROM_INT_OPS:
            rs2, f7 = F_FROM_INT_OPS[mnem]
            rd, rs1 = freg(args[0]), reg(args[1])
            return [(r_type_fp(OPCODE_OP_FP, 0b000, f7, rd, rs1, rs2), 4)]
        if mnem in F_CLASS_OPS:
            f7 = F_CLASS_OPS[mnem]
            rd, rs1 = reg(args[0]), freg(args[1])
            return [(r_type_fp(OPCODE_OP_FP, 0b001, f7, rd, rs1, 0b00000), 4)]
        if mnem in F_MV_X_OPS:
            f7 = F_MV_X_OPS[mnem]
            rd, rs1 = reg(args[0]), freg(args[1])
            return [(r_type_fp(OPCODE_OP_FP, 0b000, f7, rd, rs1, 0b00000), 4)]
        if mnem in F_MV_FROM_X_OPS:
            f7 = F_MV_FROM_X_OPS[mnem]
            rd, rs1 = freg(args[0]), reg(args[1])
            return [(r_type_fp(OPCODE_OP_FP, 0b000, f7, rd, rs1, 0b00000), 4)]
        if mnem == "fcvt.s.d":
            rd, rs1 = freg(args[0]), freg(args[1])
            return [(r_type_fp(OPCODE_OP_FP, 0b000, 0b0100000, rd, rs1, 0b00001), 4)]
        if mnem == "fcvt.d.s":
            rd, rs1 = freg(args[0]), freg(args[1])
            return [(r_type_fp(OPCODE_OP_FP, 0b000, 0b0100001, rd, rs1, 0b00000), 4)]
        if mnem == "vsetvli":
            rd, rs1 = reg(args[0]), reg(args[1])
            zimm = 0b00000010000  # SEW=32 (010), LMUL=1 (000) -- the only config this core supports
            return [((zimm << 20) | (rs1 << 15) | (0b111 << 12) | (rd << 7) | OPCODE_OP_V, 4)]
        if mnem == "vle32.v":
            vd = vreg(args[0])
            rs1 = reg(parse_paren_reg(args[1]))
            return [((0b000 << 29) | (0b00 << 26) | (0b1 << 25) | (0b00000 << 20) |
                     (rs1 << 15) | (0b110 << 12) | (vd << 7) | OPCODE_LOAD_FP, 4)]
        if mnem == "vse32.v":
            vs3 = vreg(args[0])
            rs1 = reg(parse_paren_reg(args[1]))
            return [((0b000 << 29) | (0b00 << 26) | (0b1 << 25) | (0b00000 << 20) |
                     (rs1 << 15) | (0b110 << 12) | (vs3 << 7) | OPCODE_STORE_FP, 4)]
        if mnem.split('.')[0] in V_RED_FUNCT6 and mnem.endswith(".vs"):
            base = mnem[:-3]
            f6 = V_RED_FUNCT6[base]
            vd, vs2, vs1 = vreg(args[0]), vreg(args[1]), vreg(args[2])
            vm = 0 if (len(args) > 3 and args[3].strip() == "v0.t") else 1
            return [(v_type(f6, vm, vs2, vs1, 0b010, vd), 4)]  # OPMVV
        if '.' in mnem and mnem.split('.')[0] in V_FUNCT6:
            base, form = mnem.split('.', 1)
            if (base, form) not in V_FORM_FUNC3:
                raise ValueError(f"'{mnem}' has no {form} form")
            f6 = V_FUNCT6[base]
            f3 = V_FORM_FUNC3[(base, form)]
            vd, vs2 = vreg(args[0]), vreg(args[1])
            if form == "vv":
                rs1_field = vreg(args[2])
            elif form == "vx":
                rs1_field = reg(args[2])
            else:  # vi
                rs1_field = imm(args[2], 5) & 0x1F
            # Optional trailing ",v0.t" marks the instruction as masked
            # (vm=0); omitting it means unmasked (vm=1), the RVV default.
            vm = 0 if (len(args) > 3 and args[3].strip() == "v0.t") else 1
            return [(v_type(f6, vm, vs2, rs1_field, f3, vd), 4)]
        if mnem in I_OPS:
            f3 = I_OPS[mnem]
            rd, rs1 = reg(args[0]), reg(args[1])
            iv = imm(args[2], 12, signed=(mnem not in ("sltiu",)))
            return [(i_type(OPCODE_I, f3, rd, rs1, iv), 4)]
        if mnem in OPIMM32_OPS:
            f3 = OPIMM32_OPS[mnem]
            rd, rs1 = reg(args[0]), reg(args[1])
            iv = imm(args[2], 12)
            return [(i_type(OPCODE_OP_IMM_32, f3, rd, rs1, iv), 4)]
        if mnem in SHIFT_I_OPS_64:
            f3, f6 = SHIFT_I_OPS_64[mnem]
            rd, rs1 = reg(args[0]), reg(args[1])
            shamt = int(args[2], 0) & 0x3F  # 6-bit shamt, 0-63
            return [(i_type(OPCODE_I, f3, rd, rs1, (f6 << 6) | shamt), 4)]
        if mnem in SHIFT_IW_OPS:
            f3, f7 = SHIFT_IW_OPS[mnem]
            rd, rs1 = reg(args[0]), reg(args[1])
            shamt = int(args[2], 0) & 0x1F  # 5-bit shamt, 0-31
            return [(i_type(OPCODE_OP_IMM_32, f3, rd, rs1, (f7 << 5) | shamt), 4)]
        if mnem in LOAD_OPS:
            f3 = LOAD_OPS[mnem]
            rd = reg(args[0])
            off, base = parse_mem_operand(args[1])
            return [(i_type(OPCODE_LOAD, f3, rd, reg(base), imm(off, 12)), 4)]
        if mnem in STORE_OPS:
            f3 = STORE_OPS[mnem]
            rs2 = reg(args[0])
            off, base = parse_mem_operand(args[1])
            return [(s_type(OPCODE_S, f3, reg(base), rs2, imm(off, 12)), 4)]
        if mnem in BRANCH_OPS:
            f3 = BRANCH_OPS[mnem]
            rs1, rs2 = reg(args[0]), reg(args[1])
            off = self._resolve(args[2], addr)
            return [(b_type(OPCODE_B, f3, rs1, rs2, imm(off, 13)), 4)]
        if mnem == "beqz":
            rs1 = reg(args[0]); off = self._resolve(args[1], addr)
            return [(b_type(OPCODE_B, 0b000, rs1, 0, imm(off, 13)), 4)]
        if mnem == "bnez":
            rs1 = reg(args[0]); off = self._resolve(args[1], addr)
            return [(b_type(OPCODE_B, 0b001, rs1, 0, imm(off, 13)), 4)]
        if mnem == "jal":
            if len(args) == 1:
                rd, target = 1, args[0]
            else:
                rd, target = reg(args[0]), args[1]
            off = self._resolve(target, addr)
            return [(j_type(OPCODE_JAL, rd, imm(off, 21)), 4)]
        if mnem == "j":
            off = self._resolve(args[0], addr)
            return [(j_type(OPCODE_JAL, 0, imm(off, 21)), 4)]
        if mnem == "jalr":
            if len(args) == 1:
                rd, rs1, off = 1, args[0], "0"
            elif len(args) == 2:
                rd, rs1, off = reg(args[0]), args[1], "0"
            else:
                rd, rs1, off = reg(args[0]), args[1], args[2]
            return [(i_type(OPCODE_JALR, 0, rd, reg(rs1), imm(off, 12)), 4)]
        if mnem == "lui":
            rd = reg(args[0])
            v = int(args[1], 0)
            return [(u_type(OPCODE_LUI, rd, v << 12 if v <= 0xFFFFF else v), 4)]
        if mnem == "auipc":
            rd = reg(args[0])
            v = int(args[1], 0)
            return [(u_type(OPCODE_AUIPC, rd, v << 12 if v <= 0xFFFFF else v), 4)]
        if mnem == "li":
            return [(w, 4) for w in self._li64_words(reg(args[0]), int(args[1], 0))]
        if mnem == "mv":
            rd, rs1 = reg(args[0]), reg(args[1])
            return [(i_type(OPCODE_I, 0b000, rd, rs1, 0), 4)]
        if mnem == "nop":
            return [(i_type(OPCODE_I, 0b000, 0, 0, 0), 4)]
        if mnem == "ecall":
            return [(i_type(OPCODE_SYSTEM, 0b000, 0, 0, 0), 4)]
        if mnem == "ebreak":
            return [(i_type(OPCODE_SYSTEM, 0b000, 0, 0, 1), 4)]
        raise ValueError(f"unknown mnemonic '{mnem}'")

    def _encode_compressed(self, addr, mnem, args):
        if mnem == "c.addi4spn":
            rd = reg(args[0]); nzuimm = int(args[1], 0)
            return c_addi4spn(rd, nzuimm)
        if mnem == "c.lw":
            rd = reg(args[0]); off, base = parse_mem_operand(args[1])
            return c_lw_sw(0b00, 0b010, rd, reg(base), int(off, 0))
        if mnem == "c.sw":
            rs2 = reg(args[0]); off, base = parse_mem_operand(args[1])
            return c_lw_sw(0b00, 0b110, rs2, reg(base), int(off, 0))
        if mnem == "c.ld":
            rd = reg(args[0]); off, base = parse_mem_operand(args[1])
            return c_ld_sd(0b00, 0b011, rd, reg(base), int(off, 0))
        if mnem == "c.sd":
            rs2 = reg(args[0]); off, base = parse_mem_operand(args[1])
            return c_ld_sd(0b00, 0b111, rs2, reg(base), int(off, 0))
        if mnem == "c.nop":
            return c_i_imm6(0b000, 0, 0)
        if mnem == "c.addi":
            rd = reg(args[0]); v = int(args[1], 0)
            return c_i_imm6(0b000, rd, v)
        if mnem == "c.addiw":
            rd = reg(args[0]); v = int(args[1], 0)
            return c_i_imm6(0b001, rd, v)
        if mnem == "c.li":
            rd = reg(args[0]); v = int(args[1], 0)
            return c_i_imm6(0b010, rd, v)
        if mnem == "c.addi16sp":
            v = int(args[0], 0)
            return c_addi16sp(v)
        if mnem == "c.lui":
            rd = reg(args[0]); v = int(args[1], 0)
            return c_lui(rd, (v & 0x3F) << 12)
        if mnem == "c.srli":
            rd = reg(args[0]); s = int(args[1], 0)
            return c_shift_imm(rd, 0b00, s)
        if mnem == "c.srai":
            rd = reg(args[0]); s = int(args[1], 0)
            return c_shift_imm(rd, 0b01, s)
        if mnem == "c.andi":
            rd = reg(args[0]); v = int(args[1], 0)
            return c_andi(rd, v)
        if mnem == "c.sub":
            rd, rs2 = reg(args[0]), reg(args[1])
            return c_ra_ra(rd, rs2, 0, 0b00)
        if mnem == "c.xor":
            rd, rs2 = reg(args[0]), reg(args[1])
            return c_ra_ra(rd, rs2, 0, 0b01)
        if mnem == "c.or":
            rd, rs2 = reg(args[0]), reg(args[1])
            return c_ra_ra(rd, rs2, 0, 0b10)
        if mnem == "c.and":
            rd, rs2 = reg(args[0]), reg(args[1])
            return c_ra_ra(rd, rs2, 0, 0b11)
        if mnem == "c.subw":
            rd, rs2 = reg(args[0]), reg(args[1])
            return c_ra_ra(rd, rs2, 1, 0b00)
        if mnem == "c.addw":
            rd, rs2 = reg(args[0]), reg(args[1])
            return c_ra_ra(rd, rs2, 1, 0b01)
        if mnem == "c.j":
            off = self._resolve(args[0], addr)
            return c_j(off)
        if mnem == "c.beqz":
            rs1 = reg(args[0]); off = self._resolve(args[1], addr)
            return c_branch(0b110, rs1, off)
        if mnem == "c.bnez":
            rs1 = reg(args[0]); off = self._resolve(args[1], addr)
            return c_branch(0b111, rs1, off)
        if mnem == "c.slli":
            rd = reg(args[0]); s = int(args[1], 0)
            return c_slli(rd, s)
        if mnem == "c.lwsp":
            rd = reg(args[0]); off = int(args[1], 0)
            return c_lwsp(rd, off)
        if mnem == "c.ldsp":
            rd = reg(args[0]); off = int(args[1], 0)
            return c_ldsp(rd, off)
        if mnem == "c.swsp":
            rs2 = reg(args[0]); off = int(args[1], 0)
            return c_swsp(rs2, off)
        if mnem == "c.sdsp":
            rs2 = reg(args[0]); off = int(args[1], 0)
            return c_sdsp(rs2, off)
        if mnem == "c.jr":
            rs1 = reg(args[0])
            return c_cr(0b1000, rs1, 0)
        if mnem == "c.jalr":
            rs1 = reg(args[0])
            return c_cr(0b1001, rs1, 0)
        if mnem == "c.mv":
            rd, rs2 = reg(args[0]), reg(args[1])
            return c_cr(0b1000, rd, rs2)
        if mnem == "c.add":
            rd, rs2 = reg(args[0]), reg(args[1])
            return c_cr(0b1001, rd, rs2)
        if mnem == "c.ebreak":
            return c_cr(0b1001, 0, 0)
        raise ValueError(f"unknown compressed mnemonic '{mnem}'")


def assemble_to_mem(text):
    """Returns a list of (value, size_in_bytes) items -- feed to
    write_imem_halfwords, not write_mem (which expects plain ints and is
    for data memory)."""
    a = Assembler64()
    return a.assemble(text)


def write_mem(path, words):
    with open(path, "w") as f:
        for w in words:
            f.write(f"{w & 0xFFFFFFFF:08x}\n")


def write_imem_halfwords(path, items):
    """instruction_fetch.v addresses instruction memory by halfword (RV64C
    mixes 16- and 32-bit instructions, so instructions are only guaranteed
    2-byte aligned). `items` is a list of (value, size_in_bytes) pairs from
    Assembler64.assemble(): a 4-byte item is split into two little-endian
    halfword lines (low half at the lower address first); a 2-byte item
    (an actual compressed instruction) is written as a single halfword
    line unchanged."""
    with open(path, "w") as f:
        for value, size in items:
            value &= 0xFFFFFFFF
            if size == 2:
                f.write(f"{value & 0xFFFF:04x}\n")
            else:
                f.write(f"{value & 0xFFFF:04x}\n")
                f.write(f"{(value >> 16) & 0xFFFF:04x}\n")
