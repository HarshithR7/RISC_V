"""
Test suite for the RV64M extension (MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU
and the *W 32-bit forms). Reuses the RV64I test harness (tb_core64.v,
build_tests64.py's TestBuilder/build_and_run).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build_tests64 import TestBuilder, build_and_run  # noqa: E402


def t_mul():
    t = TestBuilder("m_mul")
    t.asm("""
li x1, 6
li x2, 7
mul x3, x1, x2
""").check_eq("x3", 42)
    # negative x positive
    t.asm("""
li x1, -6
li x2, 7
mul x3, x1, x2
""").check_eq("x3", "0xFFFFFFFFFFFFFFD6")  # -42
    # negative x negative
    t.asm("""
li x1, -6
li x2, -7
mul x3, x1, x2
""").check_eq("x3", 42)
    # MUL only keeps the low 64 bits: a product that overflows 64 bits
    # truncates the same way regardless of operand signedness.
    t.asm("""
li x1, 0x7FFFFFFF
slli x1, x1, 32
li x4, 0xFFFFFFFF
or x1, x1, x4
mul x3, x1, x1
""")  # x1 = 0x7FFFFFFFFFFFFFFF; low 64 bits of its square
    return t


def t_mulh():
    t = TestBuilder("m_mulh")
    # MULHU: (2^64 - 1) * (2^64 - 1) = 2^128 - 2^65 + 1; upper 64 bits = 0xFFFFFFFFFFFFFFFE
    t.asm("""
li x1, -1
mulhu x3, x1, x1
""").check_eq("x3", "0xFFFFFFFFFFFFFFFE")
    # MULH: (-1) * (-1) signed = 1, upper 64 bits of a small positive product = 0
    t.asm("""
li x1, -1
mulh x3, x1, x1
""").check_eq("x3", 0)
    # MULHSU: -1 (signed) * 0xFFFFFFFFFFFFFFFF (as UNSIGNED, i.e. 2^64-1)
    # = -(2^64-1); upper 64 bits of that (128-bit two's complement) = 0xFFFFFFFFFFFFFFFF
    t.asm("""
li x1, -1
mulhsu x3, x1, x1
""").check_eq("x3", "0xFFFFFFFFFFFFFFFF")
    # A case where MULH and MULHU clearly diverge: 0x8000000000000000 (MIN_INT64)
    # squared. Signed: MIN*MIN = 2^126 (positive, huge). Unsigned: same bit
    # pattern squared as a huge positive number = 2^126 too -- pick operands
    # where they actually differ instead: -2 * 0x8000000000000000 unsigned.
    t.asm("""
li x1, -2
li x2, 0x7FFFFFFF
slli x2, x2, 32
mulh x3, x1, x2
mulhu x4, x1, x2
""")
    t.check_eq("x3", "0xFFFFFFFFFFFFFFFF")  # signed: -2 * positive = negative -> upper bits all 1
    t.check_eq("x4", "0x7FFFFFFEFFFFFFFF")  # unsigned: x1=0xFFFF...FFFE, a huge positive product
    return t


def t_div_rem():
    t = TestBuilder("m_div_rem")
    t.asm("""
li x1, 17
li x2, 5
div x3, x1, x2
rem x4, x1, x2
""")
    t.check_eq("x3", 3)
    t.check_eq("x4", 2)
    # REM takes the sign of the dividend (truncating division), per spec
    t.asm("""
li x1, -17
li x2, 5
div x3, x1, x2
rem x4, x1, x2
""")
    t.check_eq("x3", "0xFFFFFFFFFFFFFFFD")  # -3 (truncated toward zero, not floored)
    t.check_eq("x4", "0xFFFFFFFFFFFFFFFE")  # -2 (sign of dividend)
    # Division by zero: architecturally defined, not a trap.
    t.asm("""
li x1, 42
li x2, 0
div x3, x1, x2
divu x4, x1, x2
rem x5, x1, x2
remu x6, x1, x2
""")
    t.check_eq("x3", "0xFFFFFFFFFFFFFFFF")  # DIV by 0 -> -1
    t.check_eq("x4", "0xFFFFFFFFFFFFFFFF")  # DIVU by 0 -> all ones
    t.check_eq("x5", 42)                    # REM by 0 -> dividend
    t.check_eq("x6", 42)                    # REMU by 0 -> dividend
    # Signed overflow: MIN_INT64 / -1 -> MIN_INT64 (not a trap, no overflow flag)
    t.asm("""
li x1, 0x80000000
slli x1, x1, 32
li x2, -1
div x3, x1, x2
rem x4, x1, x2
""")
    t.check_eq("x3", "0x8000000000000000")
    t.check_eq("x4", 0)
    # DIVU / REMU: unsigned comparison matters
    t.asm("""
li x1, -1
li x2, 2
divu x3, x1, x2
remu x4, x1, x2
""")
    t.check_eq("x3", "0x7FFFFFFFFFFFFFFF")  # (2^64-1)/2 = 0x7FFF...FFFF
    t.check_eq("x4", 1)
    return t


def t_word_ops():
    t = TestBuilder("m_word_ops")
    # MULW: overflow wraps within 32 bits, then sign-extends -- same story
    # as ADDW/etc.
    t.asm("""
li x1, 0x10000
li x2, 0x10000
mulw x3, x1, x2
""").check_eq("x3", 0)  # 0x100000000 truncated to 32 bits = 0
    t.asm("""
li x1, 200000
li x2, 100000
mulw x3, x1, x2
""").check_eq("x3", "0xFFFFFFFFA817C800")  # product mod 2^32 has bit31 set -> sign-extends
    # DIVW/REMW special cases at 32-bit width
    t.asm("""
li x1, 0x80000000
li x2, -1
divw x3, x1, x2
remw x4, x1, x2
""")
    t.check_eq("x3", "0xFFFFFFFF80000000")  # 32-bit MIN_INT / -1 -> MIN_INT, sign-extended
    t.check_eq("x4", 0)
    t.asm("""
li x1, 7
li x2, 0
divw x3, x1, x2
remw x4, x1, x2
""")
    t.check_eq("x3", "0xFFFFFFFFFFFFFFFF")
    t.check_eq("x4", 7)
    return t


def t_app_factorial_real_mul():
    # Same 15! as app_factorial64, but using real MUL instead of the
    # shift-and-add fallback -- demonstrates the M extension replacing that
    # workaround, and should reach the answer in far fewer cycles.
    t = TestBuilder("m_app_factorial")
    t.asm("""
li x1, 15
li x2, 1
loop:
beq x1, x0, done
mul x2, x2, x1
addi x1, x1, -1
j loop
done:
""").check_eq("x2", math.factorial(15))
    return t


def main():
    results = []
    logs = {}

    def run(builder):
        line, log = build_and_run(builder)
        results.append(line)
        logs[builder.name] = log
        print(line)

    run(t_mul())
    run(t_mulh())
    run(t_div_rem())
    run(t_word_ops())
    run(t_app_factorial_real_mul())

    print("\n--- SUMMARY ---")
    passed = sum(1 for r in results if r.startswith("[PASS]"))
    total = len(results)
    print(f"{passed}/{total} passed")
    for r in results:
        print(r)


if __name__ == "__main__":
    main()
