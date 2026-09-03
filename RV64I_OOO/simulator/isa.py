"""
Shared RV64I+M instruction representation for the performance-modeling
ladder (analytical / trace-driven / cycle-level -- see
simulator/bench_compare.py). Rather than writing a second assembler,
`assemble()` calls the existing, already-proven RV64I/verify/asm64.py
(the same one verify/bench_ooo.py already uses for the real RTL flow) to
get real machine code, then decodes those words here -- this guarantees
every model in this directory processes the *exact* instruction stream
the RTL executes, including `li`'s real 8-instruction expansion, not a
1-instruction pseudo-op abstraction that would silently desync
instruction counts from the RTL comparison this whole ladder exists to
support.

Decode logic mirrors RV64I_OOO/src/decode_ooo.v field-for-field (opcode/
func3/func7 tables, R/I/S/B/U/J immediate generation, the RVM_FUNCT7
mul/div split, LUI/AUIPC/SYSTEM special-casing) -- scoped to RV64I+M
only, matching this core's own scope; OP_V (vector) decodes to an inert
no-op class, the same "unrecognized: no register/memory effect"
fallback decode_ooo.v itself uses.
"""
import os
import sys
from dataclasses import dataclass, field

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
RV64I_VERIFY = os.path.join(THIS_DIR, "..", "..", "RV64I", "verify")
sys.path.insert(0, RV64I_VERIFY)
import asm64 as asm  # noqa: E402

# ---- Opcodes (RV64I_OOO/src/decode_ooo.v) ----------------------------------
R_TYPE    = 0b0110011
I_TYPE    = 0b0010011
S_TYPE    = 0b0100011
B_TYPE    = 0b1100011
LUI_TYPE  = 0b0110111
AUIPC     = 0b0010111
JAL       = 0b1101111
JALR      = 0b1100111
LOAD      = 0b0000011
OP_IMM_32 = 0b0011011
OP_32     = 0b0111011
SYSTEM    = 0b1110011
OP_V      = 0b1010111

RVM_FUNCT7 = 0b0000001

# Confirmed op latencies (see this file's own header + the plan's
# "Confirmed operation latencies" section): ALU/MUL are both 1-cycle
# combinational in the RTL (alu_rs.v/mul_rs.v); DIV is div_fu.v's real
# multi-cycle iterative divider. Branch resolution is 1 cycle
# (branch_rs.v resolves the same cycle its operands become ready). Load
# latency here models an L1 *hit* (1 cycle, l1_cache.v) -- none of the
# shared benchmarks (simulator/workloads.py) touch memory, so a miss
# model isn't needed yet; a future workload that does would need this
# extended, not silently mis-modeled.
LAT_ALU = 1
LAT_MUL = 1
LAT_DIV = 64
LAT_BRANCH = 1
LAT_LOAD = 1
LAT_STORE = 1


@dataclass
class Instr:
    idx: int              # program-order index
    mnemonic: str
    rd: int
    rs1: int
    rs2: int
    imm: int
    is_alu: bool = False
    is_mul: bool = False
    is_div: bool = False
    is_branch: bool = False
    is_jal: bool = False
    is_jalr: bool = False
    is_load: bool = False
    is_store: bool = False
    is_ecall: bool = False
    reg_write: bool = False
    word_op: bool = False
    src2_is_imm: bool = False  # mirrors decode_ooo.v's own field name --
                                # True means `rs2`'s bit pattern is not a
                                # real register dependency at all (either
                                # genuinely the immediate, as in
                                # decode_ooo.v, or -- for LOAD -- simply
                                # never consumed by anything downstream;
                                # see isa.py's LOAD case below for why
                                # both get folded into this one flag)
    src1_is_dep: bool = True   # False for JAL (no register operands at
                                # all -- see branch_rs.v's own header)
                                # and LUI/AUIPC (src1_is_zero/src1_is_pc
                                # in decode_ooo.v -- x0/pc, never a real
                                # rename dependency)

    @property
    def is_muldiv(self):
        return self.is_mul or self.is_div

    @property
    def opcode_class(self):
        for name in ("is_ecall", "is_branch", "is_jal", "is_jalr", "is_load",
                      "is_store", "is_div", "is_mul", "is_alu"):
            if getattr(self, name):
                return name[3:]
        return "nop"

    def latency(self):
        return {
            "alu": LAT_ALU, "mul": LAT_MUL, "div": LAT_DIV,
            "branch": LAT_BRANCH, "jal": LAT_BRANCH, "jalr": LAT_BRANCH,
            "load": LAT_LOAD, "store": LAT_STORE, "ecall": LAT_ALU,
            "nop": LAT_ALU,
        }[self.opcode_class]


def _sext(value, bits):
    sign = 1 << (bits - 1)
    return (value & (sign - 1)) - (value & sign)


def _decode_one(word, idx):
    opcode = word & 0x7F
    rd = (word >> 7) & 0x1F
    func3 = (word >> 12) & 0x7
    rs1 = (word >> 15) & 0x1F
    rs2 = (word >> 20) & 0x1F
    func7 = (word >> 25) & 0x7F

    i = Instr(idx=idx, mnemonic="?", rd=rd, rs1=rs1, rs2=rs2, imm=0)

    if opcode in (LOAD, I_TYPE, JALR, OP_IMM_32):
        i.imm = _sext(word >> 20, 12)
    elif opcode == S_TYPE:
        i.imm = _sext(((word >> 25) << 5) | rd, 12)
    elif opcode == B_TYPE:
        imm = (((word >> 31) & 1) << 12) | (((word >> 7) & 1) << 11) | \
              (((word >> 25) & 0x3F) << 5) | (((word >> 8) & 0xF) << 1)
        i.imm = _sext(imm, 13)
    elif opcode == JAL:
        imm = (((word >> 31) & 1) << 20) | (((word >> 12) & 0xFF) << 12) | \
              (((word >> 20) & 1) << 11) | (((word >> 21) & 0x3FF) << 1)
        i.imm = _sext(imm, 21)
    elif opcode in (LUI_TYPE, AUIPC):
        i.imm = _sext(word & 0xFFFFF000, 32)

    if opcode == R_TYPE:
        i.reg_write = True
        if func7 == RVM_FUNCT7:
            i.is_mul = func3 < 4       # MUL/MULH/MULHSU/MULHU (funct3 0-3)
            i.is_div = func3 >= 4      # DIV/DIVU/REM/REMU (funct3 4-7)
            i.mnemonic = "mul" if i.is_mul else "div"
        else:
            i.is_alu = True
            i.mnemonic = "add"  # exact ALU op doesn't matter for timing
    elif opcode == OP_32:
        i.reg_write = True
        i.word_op = True
        if func7 == RVM_FUNCT7:
            i.is_mul = func3 < 4
            i.is_div = func3 >= 4
            i.mnemonic = "mulw" if i.is_mul else "divw"
        else:
            i.is_alu = True
            i.mnemonic = "addw"
    elif opcode == I_TYPE:
        i.reg_write = True
        i.is_alu = True
        i.src2_is_imm = True  # bits[24:20] are part of the imm, not rs2
        i.mnemonic = "addi"
    elif opcode == OP_IMM_32:
        i.reg_write = True
        i.is_alu = True
        i.word_op = True
        i.src2_is_imm = True
        i.mnemonic = "addiw"
    elif opcode == LOAD:
        i.reg_write = True
        i.is_load = True
        # Same I-type immediate shape as I_TYPE (bits[24:20] are imm, not
        # rs2) -- decode_ooo.v doesn't even bother setting src2_is_imm
        # here since nothing downstream ever reads a load's "src2" at
        # all (lsq.v's alloc port only takes alloc_base_ready/val/tag,
        # fed from rs1 -- see riscv64_ooo_proc.v's lsq_i instantiation) --
        # folded into the same flag here since the practical effect
        # (rs2 is not a real dependency) is identical.
        i.src2_is_imm = True
        i.mnemonic = "load"
    elif opcode == S_TYPE:
        i.is_store = True
        i.mnemonic = "store"
    elif opcode == B_TYPE:
        i.is_branch = True
        i.mnemonic = "branch"
    elif opcode == JAL:
        i.reg_write = True
        i.is_jal = True
        # branch_rs.v's own header: "JAL needs no register operands at
        # all (target = pc+imm, return value = pc+4)" -- riscv64_ooo_proc.v
        # unconditionally treats src1 as ready via `d0_is_jal ||
        # lane0_src1_ready`, ignoring the raw rs1 bit pattern entirely.
        i.src1_is_dep = False
        i.src2_is_imm = True
        i.mnemonic = "jal"
    elif opcode == JALR:
        i.reg_write = True
        i.is_jalr = True
        i.src2_is_imm = True  # JALR's "src2" is the immediate offset
        i.mnemonic = "jalr"
    elif opcode == LUI_TYPE:
        i.reg_write = True
        i.is_alu = True
        i.src1_is_dep = False  # src1_is_zero in decode_ooo.v
        i.src2_is_imm = True   # bits[24:20] are part of the U-type imm here
        i.mnemonic = "lui"
    elif opcode == AUIPC:
        i.reg_write = True
        i.is_alu = True
        i.src1_is_dep = False  # src1_is_pc in decode_ooo.v
        i.src2_is_imm = True   # bits[24:20] are part of the U-type imm here
        i.mnemonic = "auipc"
    elif opcode == SYSTEM:
        i.is_ecall = True
        i.is_alu = True
        i.src1_is_dep = False
        i.src2_is_imm = True
        i.mnemonic = "ecall"
    elif opcode == OP_V:
        # Scoped out (see decode_ooo.v's own OP_V comment) -- no
        # register/memory effect, matches its "unrecognized" fallback.
        i.mnemonic = "vec-nop"
    else:
        i.mnemonic = "nop"

    return i


def assemble(asm_text):
    """Assembles asm_text via the real RV64I+M assembler and decodes the
    resulting machine code into a list of Instr, in program order. This
    is the exact instruction stream the RTL executes -- see this file's
    header for why that fidelity matters."""
    items = asm.assemble_to_mem(asm_text)
    instrs = []
    idx = 0
    for value, size in items:
        if size != 4:
            # This core has no compressed-instruction support (see
            # riscv64_ooo_proc.v's own scope notes) -- none of the
            # shared benchmarks should ever produce one.
            raise ValueError(f"unsupported {size}-byte (compressed) instruction in workload")
        instrs.append(_decode_one(value, idx))
        idx += 1
    return instrs
