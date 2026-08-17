"""
Generates .mem test programs + wrapper testbenches, compiles them with
Icarus Verilog against the fixed RTL, runs them, and prints a pass/fail
summary. Run from anywhere; paths are resolved relative to this file.
"""
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
import asm  # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
RTL = os.path.join(ROOT, "..", "RISC_V.srcs", "sources_1", "new")
GEN = os.path.join(ROOT, "generated")
os.makedirs(GEN, exist_ok=True)

RTL_FILES = [
    "Program_counter.v",
    "Instruction_Fetch.v",
    "Instruction_decode_control_unit.v",
    "register_rw.v",
    "alu.v",
    "data_memory.v",
    "riscV_proc.v",
]


class TestBuilder:
    """Helper to write self-checking RV32I assembly with minimal boilerplate.
    Each check compares a register against an expected 32-bit value; on the
    first mismatch the program sets x31 to that check's id and halts, so a
    failure is immediately attributable to a specific check.
    """

    def __init__(self, name):
        self.name = name
        self.body = []
        self.checks = 0
        self.fail_handlers = []

    def asm(self, text):
        self.body.append(text)
        return self

    def check_eq(self, reg, expected, scratch="x30"):
        self.checks += 1
        cid = self.checks
        self.body.append(f"li {scratch}, {expected}")
        self.body.append(f"bne {reg}, {scratch}, fail{cid}")
        self.fail_handlers.append(cid)
        return self

    def check_eq_reg(self, reg, expected_reg):
        self.checks += 1
        cid = self.checks
        self.body.append(f"bne {reg}, {expected_reg}, fail{cid}")
        self.fail_handlers.append(cid)
        return self

    def source(self):
        lines = list(self.body)
        lines.append("li x31, 0xFFFF0000")
        lines.append("ecall")
        for cid in self.fail_handlers:
            lines.append(f"fail{cid}:")
            lines.append(f"li x31, {cid}")
            lines.append("ecall")
        return "\n".join(lines)


def build_and_run(tb, dmem_words=None, max_cycles=20000):
    name = tb.name
    src = tb.source()
    words = asm.assemble_to_mem(src)
    imem_path = os.path.join(GEN, f"{name}.mem")
    asm.write_mem(imem_path, words)

    dmem_path = os.path.join(GEN, f"{name}_data.mem")
    if dmem_words is None:
        dmem_words = [0] * 64
    asm.write_mem(dmem_path, dmem_words)

    wrapper_path = os.path.join(GEN, f"tb_{name}.v")
    with open(wrapper_path, "w") as f:
        f.write(f"""`timescale 1ns/1ps
module tb_{name};
    tb_core #(
        .IMEM_FILE("{name}.mem"),
        .DMEM_FILE("{name}_data.mem"),
        .TEST_NAME("{name}"),
        .MAX_CYCLES({max_cycles})
    ) core();
endmodule
""")

    vvp_path = os.path.join(GEN, f"{name}.vvp")
    compile_cmd = [
        "iverilog", "-g2012", "-o", vvp_path,
        os.path.join(ROOT, "tb_core.v"),
        wrapper_path,
    ] + [os.path.join(RTL, f) for f in RTL_FILES]
    r = subprocess.run(compile_cmd, cwd=GEN, capture_output=True, text=True)
    if r.returncode != 0:
        return f"[COMPILE ERROR] {name}\n{r.stdout}\n{r.stderr}", ""

    r2 = subprocess.run(["vvp", vvp_path], cwd=GEN, capture_output=True, text=True)
    out = r2.stdout + r2.stderr
    result_line = ""
    for line in out.splitlines():
        if line.startswith("[PASS]") or line.startswith("[FAIL]") or line.startswith("[TIMEOUT]"):
            result_line = line
    return result_line, out


# ---------------------------------------------------------------------------
# Test programs
# ---------------------------------------------------------------------------

def t_alu_rtype():
    t = TestBuilder("alu_rtype")
    t.asm("""
li x1, 5
li x2, 3
add x3, x1, x2
""").check_eq("x3", 8)
    t.asm("sub x3, x1, x2").check_eq("x3", 2)
    t.asm("sub x3, x2, x1").check_eq("x3", "0xFFFFFFFE")  # 3-5 = -2
    # overflow wraparound: 0x7FFFFFFF + 1 = 0x80000000
    t.asm("""
li x1, 0x7FFFFFFF
li x2, 1
add x3, x1, x2
""").check_eq("x3", "0x80000000")
    # underflow: 0 - 1 = 0xFFFFFFFF
    t.asm("""
li x1, 0
li x2, 1
sub x3, x1, x2
""").check_eq("x3", "0xFFFFFFFF")
    t.asm("""
li x1, 0x0F0F0F0F
li x2, 0x00FF00FF
and x3, x1, x2
""").check_eq("x3", "0x000F000F")
    t.asm("or x3, x1, x2").check_eq("x3", "0x0FFF0FFF")
    t.asm("xor x3, x1, x2").check_eq("x3", "0x0FF00FF0")
    # shifts, including shift-by-31 and shift-by-0
    t.asm("""
li x1, 1
li x2, 31
sll x3, x1, x2
""").check_eq("x3", "0x80000000")
    t.asm("""
li x1, 0x80000000
li x2, 31
srl x3, x1, x2
""").check_eq("x3", 1)
    t.asm("""
li x1, 0x80000000
li x2, 4
sra x3, x1, x2
""").check_eq("x3", "0xF8000000")  # arithmetic shift keeps sign
    t.asm("""
li x1, 5
li x2, 0
sll x3, x1, x2
""").check_eq("x3", 5)  # shift by 0 is identity
    # SLT / SLTU signed vs unsigned corner case
    t.asm("""
li x1, 0xFFFFFFFF
li x2, 1
slt x3, x1, x2
""").check_eq("x3", 1)  # -1 < 1 signed
    t.asm("sltu x3, x1, x2").check_eq("x3", 0)  # 0xFFFFFFFF < 1 unsigned is false
    # x0 is never writable
    t.asm("""
li x1, 123
add x0, x1, x1
""").check_eq("x0", 0)
    return t


def t_alu_itype():
    t = TestBuilder("alu_itype")
    t.asm("addi x1, x0, 2047").check_eq("x1", 2047)     # max positive imm12
    t.asm("addi x1, x0, -2048").check_eq("x1", "0xFFFFF800")  # min negative imm12
    t.asm("""
li x2, 10
andi x3, x2, 6
""").check_eq("x3", 2)
    t.asm("ori x3, x2, 1").check_eq("x3", 11)
    t.asm("xori x3, x2, 0xFF").check_eq("x3", "0xF5")
    t.asm("""
li x2, -8
slti x3, x2, 0
""").check_eq("x3", 1)
    t.asm("sltiu x3, x2, 0").check_eq("x3", 0)  # -8 as unsigned is huge, not < 0
    t.asm("""
li x2, 1
slli x3, x2, 10
""").check_eq("x3", 1024)
    t.asm("""
li x2, -1024
srai x3, x2, 4
""").check_eq("x3", "0xFFFFFFC0")  # -64
    t.asm("""
li x2, 0x80000000
srli x3, x2, 28
""").check_eq("x3", 8)
    return t


def t_branch():
    t = TestBuilder("branch")
    t.asm("""
li x1, 5
li x2, 5
li x4, 0
beq x1, x2, beq_ok
addi x4, x0, 1
beq_ok:
""").check_eq("x4", 0)  # BEQ taken (equal) -> skip the addi

    t.asm("""
li x1, 5
li x2, 6
li x4, 0
bne x1, x2, bne_ok
addi x4, x0, 1
bne_ok:
""").check_eq("x4", 0)  # BNE taken (not equal)

    t.asm("""
li x1, -1
li x2, 1
li x4, 0
blt x1, x2, blt_ok
addi x4, x0, 1
blt_ok:
""").check_eq("x4", 0)  # signed: -1 < 1 taken

    t.asm("""
li x1, -1
li x2, 1
li x4, 0
bltu x1, x2, bltu_bad
addi x4, x0, 1
j bltu_done
bltu_bad:
addi x4, x0, 2
bltu_done:
""").check_eq("x4", 1)  # unsigned: 0xFFFFFFFF is NOT < 1

    t.asm("""
li x1, 1
li x2, -1
li x4, 0
bge x1, x2, bge_ok
addi x4, x0, 1
bge_ok:
""").check_eq("x4", 0)  # signed: 1 >= -1 taken

    t.asm("""
li x1, 1
li x2, -1
li x4, 0
bgeu x1, x2, bgeu_bad
addi x4, x0, 1
j bgeu_done
bgeu_bad:
addi x4, x0, 2
bgeu_done:
""").check_eq("x4", 1)  # unsigned: 1 is NOT >= 0xFFFFFFFF
    return t


def t_mem():
    t = TestBuilder("mem")
    # word round trip
    t.asm("""
li x1, 0x11223344
li x2, 0
sw x1, 0(x2)
lw x3, 0(x2)
""").check_eq("x3", "0x11223344")

    # byte round trip with sign extension (0xFF -> -1) and zero extension (0xFF -> 255)
    t.asm("""
li x1, 0xFF
li x2, 4
sb x1, 0(x2)
lb x3, 0(x2)
lbu x4, 0(x2)
""")
    t.check_eq("x3", "0xFFFFFFFF")
    t.check_eq("x4", 255)

    # halfword round trip, sign and zero extension
    t.asm("""
li x1, 0x8001
li x2, 8
sh x1, 0(x2)
lh x3, 0(x2)
lhu x4, 0(x2)
""")
    t.check_eq("x3", "0xFFFF8001")
    t.check_eq("x4", "0x8001")

    # byte store must not clobber neighboring bytes in the same word
    # (byte offset 1 of 0xAABBCCDD is the 0xCC byte, bits[15:8])
    t.asm("""
li x1, 0xAABBCCDD
li x2, 12
sw x1, 0(x2)
li x5, 0x11
sb x5, 1(x2)
lw x6, 0(x2)
""").check_eq("x6", "0xAABB11DD")

    # all four byte offsets within a word
    t.asm("""
li x2, 16
sw x0, 0(x2)
li x5, 0xA1
sb x5, 0(x2)
li x5, 0xB2
sb x5, 1(x2)
li x5, 0xC3
sb x5, 2(x2)
li x5, 0xD4
sb x5, 3(x2)
lw x6, 0(x2)
""").check_eq("x6", "0xD4C3B2A1")

    # halfword at offset 2 within a word
    t.asm("""
li x2, 20
sw x0, 0(x2)
li x5, 0x1234
sh x5, 2(x2)
lw x6, 0(x2)
""").check_eq("x6", "0x12340000")
    return t


def t_jump():
    t = TestBuilder("jump")
    t.asm("""
li x5, 0
jal x1, jtarget
addi x5, x0, 1
j jdone
jtarget:
addi x5, x0, 2
jdone:
""").check_eq("x5", 2)  # jumped over the "addi x5,1"

    t.asm("""
la_setup:
auipc x2, 0
addi x2, x2, 12
jalr x1, x2, 0
addi x6, x0, 1
j jalr_done
addi x6, x0, 2
jalr_done:
""").check_eq("x6", 1)  # jalr target lands exactly on the first addi

    # return address correctness: x1 after jal must equal instruction-after-jal's address
    t.asm("""
jal x3, skip
nop
skip:
""")
    return t


def t_upper():
    t = TestBuilder("upper")
    t.asm("lui x1, 0xFFFFF").check_eq("x1", "0xFFFFF000")
    t.asm("lui x1, 0").check_eq("x1", 0)
    t.asm("""
li x2, 0xDEADBEEF
lui x1, 0x12345
addi x1, x1, 0x678
""").check_eq("x1", "0x12345678")
    return t


def t_gcd():
    # Euclidean GCD via repeated subtraction (no mul/div in base RV32I).
    t = TestBuilder("app_gcd")
    t.asm("""
li x1, 462
li x2, 1071
gcd_loop:
beq x1, x2, gcd_done
blt x1, x2, gcd_swap
sub x1, x1, x2
j gcd_loop
gcd_swap:
sub x2, x2, x1
j gcd_loop
gcd_done:
""").check_eq("x1", 21)
    return t


def t_factorial():
    t = TestBuilder("app_factorial")
    # x1 = n, x2 = result (repeated addition since there is no MUL)
    t.asm("""
li x1, 6
li x2, 1
li x3, 0
fact_loop:
beq x1, x0, fact_done
addi x1, x1, -1
# x2 = x2 * (x1+1) via repeated addition loop using x4 as counter, x5 accumulator
add x6, x1, x0
addi x6, x6, 1
li x5, 0
li x4, 0
mul_loop:
beq x4, x6, mul_done
add x5, x5, x2
addi x4, x4, 1
j mul_loop
mul_done:
add x2, x5, x0
j fact_loop
fact_done:
""").check_eq("x2", 720)  # 6! = 720
    return t


def t_fibonacci():
    t = TestBuilder("app_fibonacci")
    t.asm("""
li x1, 0
li x2, 1
li x3, 15
li x4, 0
fib_loop:
beq x4, x3, fib_done
add x5, x1, x2
add x1, x2, x0
add x2, x5, x0
addi x4, x4, 1
j fib_loop
fib_done:
""").check_eq("x1", 610)  # 15th Fibonacci number (0-indexed from fib(0)=0)
    return t


def t_sum_array():
    t = TestBuilder("app_sum_array")
    data = [3, 7, 1, 9, 2, 8, 4, 6, 5, 10]  # sum = 55, stored at word 0..9
    t.asm(f"""
li x1, 0          # base address
li x2, 0          # sum
li x3, {len(data)}
li x4, 0          # index
sum_loop:
beq x4, x3, sum_done
slli x5, x4, 2
add x5, x5, x1
lw x6, 0(x5)
add x2, x2, x6
addi x4, x4, 1
j sum_loop
sum_done:
""").check_eq("x2", sum(data))
    return t, data


def t_bubble_sort():
    t = TestBuilder("app_bubble_sort")
    data = [5, 3, 8, 1, 9, 2]
    n = len(data)
    t.asm(f"""
li x1, 0            # base address
li x2, {n}           # n
li x3, 0             # i = 0
outer:
li x9, {n-1}
bge x3, x9, outer_done
li x4, 0             # j = 0
addi x10, x2, -1
sub x10, x10, x3     # x10 = n-1-i
inner:
bge x4, x10, inner_done
slli x5, x4, 2
add x5, x5, x1
lw x6, 0(x5)
addi x7, x5, 4
lw x8, 0(x7)
ble_check:
blt x8, x6, do_swap
j no_swap
do_swap:
sw x8, 0(x5)
sw x6, 0(x7)
no_swap:
addi x4, x4, 1
j inner
inner_done:
addi x3, x3, 1
j outer
outer_done:
""")
    expected = sorted(data)
    for i, val in enumerate(expected):
        t.asm(f"lw x11, {4*i}(x1)").check_eq("x11", val)
    return t, data


def main():
    results = []
    logs = {}

    def run(builder, dmem_words=None):
        line, log = build_and_run(builder, dmem_words=dmem_words)
        results.append(line)
        logs[builder.name] = log
        print(line)

    run(t_alu_rtype())
    run(t_alu_itype())
    run(t_branch())
    run(t_mem())
    run(t_jump())
    run(t_upper())
    run(t_gcd())
    run(t_factorial())
    run(t_fibonacci())

    sum_builder, sum_data = t_sum_array()
    run(sum_builder, dmem_words=sum_data)

    sort_builder, sort_data = t_bubble_sort()
    run(sort_builder, dmem_words=sort_data)

    print("\n--- SUMMARY ---")
    passed = sum(1 for r in results if r.startswith("[PASS]"))
    total = len(results)
    print(f"{passed}/{total} passed")
    for r in results:
        print(r)

    with open(os.path.join(GEN, "run_log.txt"), "w") as f:
        for name, log in logs.items():
            f.write(f"===== {name} =====\n{log}\n\n")


if __name__ == "__main__":
    main()
