"""
Generates .mem test programs + wrapper testbenches for the RV64I core,
compiles them with Icarus Verilog, runs them, and prints a pass/fail summary.
"""
import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
import asm64 as asm  # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
RTL = os.path.join(ROOT, "..", "src")
GEN = os.path.join(ROOT, "generated")
os.makedirs(GEN, exist_ok=True)

RTL_FILES = [
    "program_counter.v",
    "instruction_fetch.v",
    "compressed_decoder.v",
    "decode_control_unit.v",
    "register_file.v",
    "fp_register_file.v",
    "fpu.v",
    "vector_register_file.v",
    "vector_alu.v",
    "execute.v",
    "data_memory.v",
    "riscv64_proc.v",
]


class TestBuilder:
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

    def source(self):
        lines = list(self.body)
        lines.append("li x31, 0xFFFF0000")
        lines.append("ecall")
        for cid in self.fail_handlers:
            lines.append(f"fail{cid}:")
            lines.append(f"li x31, {cid}")
            lines.append("ecall")
        return "\n".join(lines)


def build_and_run(tb, dmem_words=None, max_cycles=40000):
    name = tb.name
    src = tb.source()
    items = asm.assemble_to_mem(src)
    imem_path = os.path.join(GEN, f"{name}.mem")
    asm.write_imem_halfwords(imem_path, items)

    dmem_path = os.path.join(GEN, f"{name}_data.mem")
    if dmem_words is None:
        dmem_words = [0] * 64
    asm.write_mem(dmem_path, dmem_words)

    wrapper_path = os.path.join(GEN, f"tb_{name}.v")
    with open(wrapper_path, "w") as f:
        f.write(f"""`timescale 1ns/1ps
module tb_{name};
    tb_core64 #(
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
        os.path.join(ROOT, "tb_core64.v"),
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
    t = TestBuilder("alu_rtype64")
    t.asm("""
li x1, 5
li x2, 3
add x3, x1, x2
""").check_eq("x3", 8)
    # 64-bit signed overflow: 0x7FFFFFFFFFFFFFFF + 1 wraps to 0x8000000000000000
    t.asm("""
li x1, 0x7FFFFFFF
slli x1, x1, 32
li x4, 0xFFFFFFFF
or x1, x1, x4
addi x1, x1, 1
""").check_eq("x1", "0x8000000000000000")
    # shift by 63 (only representable with a 6-bit shamt)
    t.asm("""
li x1, 1
li x2, 63
sll x3, x1, x2
""").check_eq("x3", "0x8000000000000000")
    # signed vs unsigned SLT on the all-ones 64-bit pattern
    t.asm("""
li x1, -1
li x2, 1
slt x3, x1, x2
sltu x4, x1, x2
""")
    t.check_eq("x3", 1)  # -1 < 1 signed
    t.check_eq("x4", 0)  # 0xFFFF...FFFF is not < 1 unsigned
    t.asm("addi x0, x1, 5").check_eq("x0", 0)
    return t


def t_alu_itype():
    t = TestBuilder("alu_itype64")
    t.asm("addi x1, x0, 2047").check_eq("x1", 2047)
    t.asm("addi x1, x0, -2048").check_eq("x1", "0xFFFFFFFFFFFFF800")
    # 6-bit shift amount: shift by 40 (impossible to encode in RV32I's 5-bit field)
    t.asm("""
li x1, 1
slli x1, x1, 40
""").check_eq("x1", "0x10000000000")
    t.asm("""
li x1, 0x8000000000000000
srli x1, x1, 4
""").check_eq("x1", "0x0800000000000000")
    t.asm("""
li x1, 0x8000000000000000
srai x1, x1, 4
""").check_eq("x1", "0xF800000000000000")  # arithmetic shift preserves sign
    return t


def t_word_ops():
    t = TestBuilder("word_ops")
    # ADDW: 32-bit overflow wraps *within the word*, then sign-extends --
    # this is the whole point of the W-suffixed instructions and the main
    # thing that differs from a plain 64-bit ADD.
    t.asm("""
li x1, 0x7FFFFFFF
li x2, 1
addw x3, x1, x2
""").check_eq("x3", "0xFFFFFFFF80000000")  # NOT 0x80000000 (which a 64-bit add would give)
    # Same, but with garbage in the upper 32 bits of the source register --
    # proves ADDW truly truncates to 32 bits rather than happening to work
    # because the upper bits were already zero.
    t.asm("""
li x4, 0x7FFFFFFF
slli x5, x4, 32
or x4, x4, x5
li x2, 1
addw x3, x4, x2
""").check_eq("x3", "0xFFFFFFFF80000000")
    t.asm("""
li x1, 5
li x2, 8
subw x3, x1, x2
""").check_eq("x3", "0xFFFFFFFFFFFFFFFD")  # -3, sign-extended
    t.asm("""
li x1, 1
li x2, 31
sllw x3, x1, x2
""").check_eq("x3", "0xFFFFFFFF80000000")  # bit 31 set -> word result is negative, sign-extends
    t.asm("""
li x1, 0x80000000
srlw x3, x1, x0
""").check_eq("x3", "0xFFFFFFFF80000000")  # SRLW is a *logical* 32-bit shift, but the 32-bit
    # result (0x80000000) still has its own sign bit set, so the 64-bit
    # write-back is still sign-extended per the spec -- SRLW isn't SRLIU.
    t.asm("""
li x1, 0x80000000
li x2, 4
sraw x3, x1, x2
""").check_eq("x3", "0xFFFFFFFFF8000000")
    t.asm("addiw x3, x0, -1").check_eq("x3", "0xFFFFFFFFFFFFFFFF")
    t.asm("""
li x1, 1
slliw x3, x1, 31
""").check_eq("x3", "0xFFFFFFFF80000000")
    return t


def t_branch():
    t = TestBuilder("branch64")
    t.asm("""
li x1, -1
li x2, 1
li x5, 0
bltu x1, x2, bad
addi x5, x0, 1
j done
bad:
addi x5, x0, 2
done:
""").check_eq("x5", 1)  # unsigned: 0xFFFF...FFFF is not < 1
    t.asm("""
li x1, -1
li x2, 1
li x5, 0
blt x1, x2, ok
addi x5, x0, 1
ok:
""").check_eq("x5", 0)  # signed: -1 < 1
    return t


def t_jump():
    t = TestBuilder("jump64")
    t.asm("""
li x5, 0
jal x1, target
addi x5, x0, 1
j done
target:
addi x5, x0, 2
done:
""").check_eq("x5", 2)
    return t


def t_upper():
    t = TestBuilder("upper64")
    # LUI sign-extends to 64 bits when bit 31 of the result is set
    t.asm("lui x1, 0xFFFFF").check_eq("x1", "0xFFFFFFFFFFFFF000")
    t.asm("lui x1, 0x7FFFF").check_eq("x1", "0x000000007FFFF000")
    return t


def t_mem():
    t = TestBuilder("mem64")
    # li only builds 32-bit-range constants, so build the 64-bit doubleword
    # pattern to store with shifts instead.
    t.asm("""
li x1, 0x11223344
slli x1, x1, 32
li x6, 0x55667788
or x1, x1, x6
li x2, 0
sd x1, 0(x2)
ld x3, 0(x2)
""").check_eq("x3", "0x1122334455667788")

    # LW sign-extends, LWU zero-extends -- the key RV64I-specific distinction
    t.asm("""
li x1, 0x80000001
li x2, 8
sw x1, 0(x2)
lw x4, 0(x2)
lwu x5, 0(x2)
""")
    t.check_eq("x4", "0xFFFFFFFF80000001")
    t.check_eq("x5", "0x0000000080000001")

    # byte/halfword sign vs zero extension into a 64-bit register
    t.asm("""
li x1, 0xFF
li x2, 16
sb x1, 0(x2)
lb x6, 0(x2)
lbu x7, 0(x2)
""")
    t.check_eq("x6", "0xFFFFFFFFFFFFFFFF")
    t.check_eq("x7", "0xFF")
    return t


def t_gcd():
    t = TestBuilder("app_gcd64")
    t.asm("""
li x1, 462
li x2, 1071
loop:
beq x1, x2, done
blt x1, x2, swap
sub x1, x1, x2
j loop
swap:
sub x2, x2, x1
j loop
done:
""").check_eq("x1", 21)
    return t


def shift_add_multiply_asm(a_reg, b_reg, result_reg, scratch1, scratch2, label_prefix):
    """result = a * b using shift-and-add (RV64I has no MUL).
    Copies operands into scratch registers before zeroing result_reg, so
    it's safe even when result_reg aliases a_reg or b_reg."""
    return f"""
mv {scratch1}, {a_reg}
mv {scratch2}, {b_reg}
li {result_reg}, 0
{label_prefix}_loop:
beq {scratch2}, x0, {label_prefix}_done
andi x29, {scratch2}, 1
beq x29, x0, {label_prefix}_skip
add {result_reg}, {result_reg}, {scratch1}
{label_prefix}_skip:
slli {scratch1}, {scratch1}, 1
srli {scratch2}, {scratch2}, 1
j {label_prefix}_loop
{label_prefix}_done:
"""


def t_factorial():
    # 15! = 1307674368000, which overflows 32 bits -- this specifically
    # exercises 64-bit-width arithmetic that the RV32I core could not do.
    t = TestBuilder("app_factorial64")
    t.asm("""
li x1, 15
li x2, 1
fact_loop:
beq x1, x0, fact_done
""")
    t.asm(shift_add_multiply_asm("x2", "x1", "x2", "x10", "x11", "mul1"))
    t.asm("""
addi x1, x1, -1
j fact_loop
fact_done:
""").check_eq("x2", math.factorial(15))
    return t


def t_fibonacci():
    # fib(50) = 12586269025, overflows 32 bits.
    t = TestBuilder("app_fibonacci64")
    n = 50
    fib = [0, 1]
    for i in range(2, n + 1):
        fib.append(fib[-1] + fib[-2])
    t.asm(f"""
li x1, 0
li x2, 1
li x3, {n}
li x4, 0
loop:
beq x4, x3, done
add x5, x1, x2
add x1, x2, x0
add x2, x5, x0
addi x4, x4, 1
j loop
done:
""").check_eq("x1", fib[n])
    return t


def t_sum_array():
    t = TestBuilder("app_sum_array64")
    data = [3, 7, 1, 9, 2, 8, 4, 6, 5, 10]
    t.asm(f"""
li x1, 0
li x2, 0
li x3, {len(data)}
li x4, 0
loop:
beq x4, x3, done
slli x5, x4, 3
add x5, x5, x1
ld x6, 0(x5)
add x2, x2, x6
addi x4, x4, 1
j loop
done:
""").check_eq("x2", sum(data))
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
    run(t_word_ops())
    run(t_branch())
    run(t_mem())
    run(t_jump())
    run(t_upper())
    run(t_gcd())
    run(t_factorial())
    run(t_fibonacci())

    sum_builder, sum_data = t_sum_array()
    # write_mem writes one $readmemh token per line; data_memory's array is
    # 64 bits wide, so each 8-hex-digit token zero-extends into one full
    # doubleword -- matches sum_array64's 8-byte-stride addressing.
    run(sum_builder, dmem_words=sum_data)

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
