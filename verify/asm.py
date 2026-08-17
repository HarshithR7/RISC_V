"""
Minimal RV32I assembler used to generate .mem files (one 8-hex-digit
instruction per line, suitable for $readmemh) for testing the RISC_V core
in RISC_V.srcs/sources_1/new/.

Not a general-purpose toolchain -- just enough of RV32I (plus pseudo-ops
li/mv/nop/j/ecall) to hand-write small test programs and get correctly
encoded machine code, rather than encoding hex by hand.
"""
import re

REGS = {f"x{i}": i for i in range(32)}
ABI = {
    "zero": 0, "ra": 1, "sp": 2, "gp": 3, "tp": 4,
    "t0": 5, "t1": 6, "t2": 7, "s0": 8, "fp": 8, "s1": 9,
    "a0": 10, "a1": 11, "a2": 12, "a3": 13, "a4": 14, "a5": 15, "a6": 16, "a7": 17,
    "s2": 18, "s3": 19, "s4": 20, "s5": 21, "s6": 22, "s7": 23, "s8": 24, "s9": 25,
    "s10": 26, "s11": 27, "t3": 28, "t4": 29, "t5": 30, "t6": 31,
}
REGS.update(ABI)


def reg(x):
    x = x.strip().rstrip(',')
    if x not in REGS:
        raise ValueError(f"unknown register '{x}'")
    return REGS[x]


def imm(x, bits=12, signed=True):
    if isinstance(x, int):
        v = x
    else:
        x = x.strip().rstrip(',')
        v = int(x, 0)
    mask = (1 << bits) - 1
    if signed:
        lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
        if not (lo <= v <= hi):
            raise ValueError(f"immediate {v} out of range for {bits}-bit signed field")
    return v & mask


def u32(v):
    return v & 0xFFFFFFFF


def r_type(opcode, funct3, funct7, rd, rs1, rs2):
    return u32((funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode)


def i_type(opcode, funct3, rd, rs1, immv):
    return u32(((immv & 0xFFF) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode)


def s_type(opcode, funct3, rs1, rs2, immv):
    imm11_5 = (immv >> 5) & 0x7F
    imm4_0 = immv & 0x1F
    return u32((imm11_5 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (imm4_0 << 7) | opcode)


def b_type(opcode, funct3, rs1, rs2, immv):
    # immv is a byte offset, must be even, 13-bit signed range
    imm12 = (immv >> 12) & 0x1
    imm11 = (immv >> 11) & 0x1
    imm10_5 = (immv >> 5) & 0x3F
    imm4_1 = (immv >> 1) & 0xF
    return u32((imm12 << 31) | (imm10_5 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) |
               (imm4_1 << 8) | (imm11 << 7) | opcode)


def u_type(opcode, rd, immv):
    return u32((immv & 0xFFFFF000) | (rd << 7) | opcode)


def j_type(opcode, rd, immv):
    imm20 = (immv >> 20) & 0x1
    imm19_12 = (immv >> 12) & 0xFF
    imm11 = (immv >> 11) & 0x1
    imm10_1 = (immv >> 1) & 0x3FF
    return u32((imm20 << 31) | (imm10_1 << 21) | (imm11 << 20) | (imm19_12 << 12) | (rd << 7) | opcode)


OPCODE_R = 0b0110011
OPCODE_I = 0b0010011
OPCODE_LOAD = 0b0000011
OPCODE_S = 0b0100011
OPCODE_B = 0b1100011
OPCODE_JAL = 0b1101111
OPCODE_JALR = 0b1100111
OPCODE_LUI = 0b0110111
OPCODE_AUIPC = 0b0010111
OPCODE_SYSTEM = 0b1110011

R_OPS = {
    "add":  (0b000, 0b0000000), "sub":  (0b000, 0b0100000),
    "sll":  (0b001, 0b0000000), "slt":  (0b010, 0b0000000),
    "sltu": (0b011, 0b0000000), "xor":  (0b100, 0b0000000),
    "srl":  (0b101, 0b0000000), "sra":  (0b101, 0b0100000),
    "or":   (0b110, 0b0000000), "and":  (0b111, 0b0000000),
}
I_OPS = {
    "addi": 0b000, "slti": 0b010, "sltiu": 0b011, "xori": 0b100,
    "ori": 0b110, "andi": 0b111,
}
SHIFT_I_OPS = {"slli": (0b001, 0b0000000), "srli": (0b101, 0b0000000), "srai": (0b101, 0b0100000)}
LOAD_OPS = {"lb": 0b000, "lh": 0b001, "lw": 0b010, "lbu": 0b100, "lhu": 0b101}
STORE_OPS = {"sb": 0b000, "sh": 0b001, "sw": 0b010}
BRANCH_OPS = {"beq": 0b000, "bne": 0b001, "blt": 0b100, "bge": 0b101, "bltu": 0b110, "bgeu": 0b111}


def parse_mem_operand(s):
    # "offset(reg)"
    m = re.match(r"\s*(-?\w+)\s*\(\s*(\w+)\s*\)\s*", s)
    if not m:
        raise ValueError(f"bad memory operand '{s}'")
    return m.group(1), m.group(2)


class Assembler:
    def __init__(self):
        self.lines = []       # (label_or_None, mnemonic, args[])
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

    def assemble(self, text):
        self.parse(text)
        # First pass: assign addresses (expand pseudo-ops to fixed instruction counts)
        addr = 0
        expanded = []
        for label, mnem, args in self.lines:
            if label:
                self.labels[label] = addr
            if mnem is None:
                continue
            n = self._expansion_count(mnem)
            expanded.append((addr, mnem, args))
            addr += 4 * n

        # Second pass: encode
        words = []
        for addr, mnem, args in expanded:
            words.extend(self._encode(addr, mnem, args))
        return words

    def _expansion_count(self, mnem):
        if mnem == "li":
            return 2  # lui+addi (conservative; collapses fine for our imm ranges)
        if mnem in ("mv", "nop", "j", "ecall", "ebreak", "beqz", "bnez"):
            return 1
        return 1

    def _resolve(self, label_or_imm, cur_addr):
        if label_or_imm in self.labels:
            return self.labels[label_or_imm] - cur_addr
        return int(label_or_imm, 0)

    def _encode(self, addr, mnem, args):
        if mnem in R_OPS:
            f3, f7 = R_OPS[mnem]
            rd, rs1, rs2 = reg(args[0]), reg(args[1]), reg(args[2])
            return [r_type(OPCODE_R, f3, f7, rd, rs1, rs2)]
        if mnem in I_OPS:
            f3 = I_OPS[mnem]
            rd, rs1 = reg(args[0]), reg(args[1])
            iv = imm(args[2], 12, signed=(mnem not in ("sltiu",)))
            return [i_type(OPCODE_I, f3, rd, rs1, iv)]
        if mnem in SHIFT_I_OPS:
            f3, f7 = SHIFT_I_OPS[mnem]
            rd, rs1 = reg(args[0]), reg(args[1])
            shamt = int(args[2], 0) & 0x1F
            return [i_type(OPCODE_I, f3, rd, rs1, (f7 << 5) | shamt)]
        if mnem in LOAD_OPS:
            f3 = LOAD_OPS[mnem]
            rd = reg(args[0])
            off, base = parse_mem_operand(args[1])
            return [i_type(OPCODE_LOAD, f3, rd, reg(base), imm(off, 12))]
        if mnem in STORE_OPS:
            f3 = STORE_OPS[mnem]
            rs2 = reg(args[0])
            off, base = parse_mem_operand(args[1])
            return [s_type(OPCODE_S, f3, reg(base), rs2, imm(off, 12))]
        if mnem in BRANCH_OPS:
            f3 = BRANCH_OPS[mnem]
            rs1, rs2 = reg(args[0]), reg(args[1])
            off = self._resolve(args[2], addr)
            return [b_type(OPCODE_B, f3, rs1, rs2, imm(off, 13))]
        if mnem == "beqz":
            rs1 = reg(args[0]); off = self._resolve(args[1], addr)
            return [b_type(OPCODE_B, 0b000, rs1, 0, imm(off, 13))]
        if mnem == "bnez":
            rs1 = reg(args[0]); off = self._resolve(args[1], addr)
            return [b_type(OPCODE_B, 0b001, rs1, 0, imm(off, 13))]
        if mnem == "jal":
            if len(args) == 1:
                rd, target = 1, args[0]
            else:
                rd, target = reg(args[0]), args[1]
            off = self._resolve(target, addr)
            return [j_type(OPCODE_JAL, rd, imm(off, 21))]
        if mnem == "j":
            off = self._resolve(args[0], addr)
            return [j_type(OPCODE_JAL, 0, imm(off, 21))]
        if mnem == "jalr":
            if len(args) == 1:
                rd, rs1, off = 1, args[0], "0"
            elif len(args) == 2:
                rd, rs1, off = reg(args[0]), args[1], "0"
            else:
                rd, rs1, off = reg(args[0]), args[1], args[2]
            return [i_type(OPCODE_JALR, 0, rd, reg(rs1), imm(off, 12))]
        if mnem == "lui":
            rd = reg(args[0])
            v = int(args[1], 0)
            return [u_type(OPCODE_LUI, rd, v << 12 if v <= 0xFFFFF else v)]
        if mnem == "auipc":
            rd = reg(args[0])
            v = int(args[1], 0)
            return [u_type(OPCODE_AUIPC, rd, v << 12 if v <= 0xFFFFF else v)]
        if mnem == "li":
            rd = reg(args[0])
            v = int(args[1], 0) & 0xFFFFFFFF
            upper = (v + 0x800) & 0xFFFFF000
            lower = v - upper
            lower &= 0xFFF
            if lower & 0x800:
                lower -= 0x1000
            return [u_type(OPCODE_LUI, rd, upper), i_type(OPCODE_I, 0b000, rd, rd, lower & 0xFFF)]
        if mnem == "mv":
            rd, rs1 = reg(args[0]), reg(args[1])
            return [i_type(OPCODE_I, 0b000, rd, rs1, 0)]
        if mnem == "nop":
            return [i_type(OPCODE_I, 0b000, 0, 0, 0)]
        if mnem == "ecall":
            return [i_type(OPCODE_SYSTEM, 0b000, 0, 0, 0)]
        if mnem == "ebreak":
            return [i_type(OPCODE_SYSTEM, 0b000, 0, 0, 1)]
        raise ValueError(f"unknown mnemonic '{mnem}'")


def assemble_to_mem(text, words_out=None):
    asm = Assembler()
    words = asm.assemble(text)
    if words_out is not None:
        words = words + [0] * max(0, words_out - len(words))
    return words


def write_mem(path, words):
    with open(path, "w") as f:
        for w in words:
            f.write(f"{w & 0xFFFFFFFF:08x}\n")
