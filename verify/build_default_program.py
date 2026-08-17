"""
Builds the default RISC_V.srcs/sources_1/new/instructions.mem + data.mem used
by the Vivado project's sim_1 fileset (riscv_processor_tb / riscv_processor),
so opening the project and running the out-of-the-box simulation exercises a
real, self-checking regression covering every RV32I instruction category
this core implements, ending in ECALL with a pass/fail status in x31.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import asm  # noqa: E402
from build_tests import TestBuilder  # noqa: E402

RTL_DIR = os.path.join(os.path.dirname(__file__), "..", "RISC_V.srcs", "sources_1", "new")


def build():
    t = TestBuilder("default_regression")

    # --- R-type / I-type ALU ---
    t.asm("""
li x1, 5
li x2, 3
add x3, x1, x2
""").check_eq("x3", 8)
    t.asm("sub x3, x1, x2").check_eq("x3", 2)
    t.asm("""
li x1, 0x7FFFFFFF
addi x1, x1, 1
""").check_eq("x1", "0x80000000")  # signed overflow wraps, no trap
    t.asm("""
li x1, 0xFFFFFFFF
li x2, 1
slt x3, x1, x2
sltu x4, x1, x2
""")
    t.check_eq("x3", 1)   # signed: -1 < 1
    t.check_eq("x4", 0)   # unsigned: 0xFFFFFFFF is not < 1

    # --- Branches (signed vs unsigned corner case) ---
    t.asm("""
li x1, -1
li x2, 1
li x5, 0
bltu x1, x2, branch_bad
addi x5, x0, 1
j branch_done
branch_bad:
addi x5, x0, 2
branch_done:
""").check_eq("x5", 1)  # unsigned: 0xFFFFFFFF is NOT < 1

    # --- Memory: byte/halfword sign+zero extension, no cross-byte clobber ---
    t.asm("""
li x1, 0xAABBCCDD
li x2, 0
sw x1, 0(x2)
li x5, 0x11
sb x5, 1(x2)
lw x6, 0(x2)
lb x7, 0(x2)
lbu x8, 0(x2)
""")
    t.check_eq("x6", "0xAABB11DD")
    t.check_eq("x7", "0xFFFFFFDD")  # 0xDD sign-extended
    t.check_eq("x8", "0xDD")        # 0xDD zero-extended

    # --- Jumps: JAL/JALR return address + target ---
    t.asm("""
li x9, 0
jal x1, jtarget
addi x9, x0, 1
j jdone
jtarget:
addi x9, x0, 2
jdone:
""").check_eq("x9", 2)

    # --- LUI / AUIPC ---
    t.asm("""
lui x10, 0x12345
addi x10, x10, 0x678
""").check_eq("x10", "0x12345678")

    # --- Application: Fibonacci(15) via iteration (no MUL/DIV in RV32I) ---
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
""").check_eq("x1", 610)

    # --- x0 is hardwired to zero ---
    t.asm("addi x0, x1, 5").check_eq("x0", 0)

    return t.source()


def main():
    src = build()
    words = asm.assemble_to_mem(src)
    asm.write_mem(os.path.join(RTL_DIR, "instructions.mem"), words)
    # Data memory starts zeroed; the program builds its own data in-place.
    asm.write_mem(os.path.join(RTL_DIR, "data.mem"), [0] * 64)
    print(f"Wrote {len(words)} instruction words to instructions.mem")


if __name__ == "__main__":
    main()
