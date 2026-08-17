"""
Test suite for the RV64A extension (LR/SC, AMOSWAP/ADD/XOR/AND/OR/MIN/MAX/
MINU/MAXU, word and doubleword forms). Reuses the RV64I test harness.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build_tests64 import TestBuilder, build_and_run  # noqa: E402


def t_amo_arith():
    t = TestBuilder("a_amo_arith")
    # AMOADD.D: rd gets the OLD value, memory gets old+rs2
    t.asm("""
li x1, 100
li x2, 0
sd x1, 0(x2)
li x3, 5
amoadd.d x4, x3, (x2)
ld x5, 0(x2)
""")
    t.check_eq("x4", 100)  # returned: old value
    t.check_eq("x5", 105)  # memory: old + rs2

    # AMOSWAP.D
    t.asm("""
li x1, 7
li x2, 8
sd x1, 8(x2)
li x3, 99
addi x6, x2, 8
amoswap.d x4, x3, (x6)
ld x5, 8(x2)
""")
    t.check_eq("x4", 7)
    t.check_eq("x5", 99)

    # AMOXOR.D / AMOAND.D / AMOOR.D
    t.asm("""
li x2, 16
li x1, 0xF0
sd x1, 0(x2)
li x3, 0x0F
amoxor.d x4, x3, (x2)
ld x5, 0(x2)
""")
    t.check_eq("x4", "0xF0")
    t.check_eq("x5", "0xFF")

    t.asm("""
li x2, 24
li x1, 0xFF
sd x1, 0(x2)
li x3, 0x0F
amoand.d x4, x3, (x2)
ld x5, 0(x2)
""")
    t.check_eq("x5", "0x0F")

    t.asm("""
li x2, 32
li x1, 0xF0
sd x1, 0(x2)
li x3, 0x0F
amoor.d x4, x3, (x2)
ld x5, 0(x2)
""")
    t.check_eq("x5", "0xFF")
    return t


def t_amo_minmax():
    t = TestBuilder("a_amo_minmax")
    # signed vs unsigned min/max on a negative number: -1 stored, compared
    # against 1. Signed: -1 is the min. Unsigned: -1 (all-ones) is huge, so
    # 1 is the min.
    t.asm("""
li x2, 0
li x1, -1
sd x1, 0(x2)
li x3, 1
amomin.d x4, x3, (x2)
ld x5, 0(x2)
""")
    t.check_eq("x5", "0xFFFFFFFFFFFFFFFF")  # signed min(-1,1) = -1, stays

    t.asm("""
li x2, 8
li x1, -1
sd x1, 0(x2)
li x3, 1
amominu.d x4, x3, (x2)
ld x5, 0(x2)
""")
    t.check_eq("x5", 1)  # unsigned min(huge, 1) = 1

    t.asm("""
li x2, 16
li x1, -1
sd x1, 0(x2)
li x3, 1
amomax.d x4, x3, (x2)
ld x5, 0(x2)
""")
    t.check_eq("x5", 1)  # signed max(-1,1) = 1

    t.asm("""
li x2, 24
li x1, -1
sd x1, 0(x2)
li x3, 1
amomaxu.d x4, x3, (x2)
ld x5, 0(x2)
""")
    t.check_eq("x5", "0xFFFFFFFFFFFFFFFF")  # unsigned max(huge, 1) = huge
    return t


def t_amo_word():
    t = TestBuilder("a_amo_word")
    # AMOADD.W: rd sign-extends the 32-bit old value; memory write only
    # touches the low 32 bits (neighboring word in the doubleword untouched).
    t.asm("""
li x2, 0
li x1, 0x80000000
li x6, 5
slli x6, x6, 32
or x1, x1, x6
sd x1, 0(x2)
li x3, 1
amoadd.w x4, x3, (x2)
ld x5, 0(x2)
""")
    t.check_eq("x4", "0xFFFFFFFF80000000")  # old low word (0x80000000) sign-extended
    t.check_eq("x5", "0x0000000580000001")  # low word incremented, high word (the 5) untouched
    return t


def t_lr_sc_success():
    t = TestBuilder("a_lr_sc_success")
    t.asm("""
li x2, 0
li x1, 42
sd x1, 0(x2)
lr.d x3, (x2)
li x4, 99
sc.d x5, x4, (x2)
ld x6, 0(x2)
""")
    t.check_eq("x3", 42)  # LR loads the current value
    t.check_eq("x5", 0)   # SC succeeds immediately after a matching LR -> 0
    t.check_eq("x6", 99)  # memory actually updated
    return t


def t_lr_sc_fail_other_store():
    t = TestBuilder("a_lr_sc_fail_store")
    t.asm("""
li x2, 0
li x1, 42
sd x1, 0(x2)
lr.d x3, (x2)
li x7, 0
li x8, 999
sd x8, 8(x7)
li x4, 99
sc.d x5, x4, (x2)
ld x6, 0(x2)
""")
    t.check_eq("x5", 1)   # an intervening store invalidates the reservation -> SC fails
    t.check_eq("x6", 42)  # memory unchanged since SC failed
    return t


def t_lr_sc_fail_wrong_addr():
    t = TestBuilder("a_lr_sc_fail_addr")
    t.asm("""
li x2, 0
li x9, 8
li x1, 42
sd x1, 0(x2)
li x1, 77
sd x1, 8(x2)
lr.d x3, (x2)
li x4, 99
sc.d x5, x4, (x9)
ld x6, 0(x2)
ld x7, 8(x2)
""")
    t.check_eq("x5", 1)   # SC to a different address than the LR reservation -> fails
    t.check_eq("x6", 42)  # neither location was touched
    t.check_eq("x7", 77)
    return t


def main():
    results = []
    logs = {}

    def run(builder):
        line, log = build_and_run(builder)
        results.append(line)
        logs[builder.name] = log
        print(line)

    run(t_amo_arith())
    run(t_amo_minmax())
    run(t_amo_word())
    run(t_lr_sc_success())
    run(t_lr_sc_fail_other_store())
    run(t_lr_sc_fail_wrong_addr())

    print("\n--- SUMMARY ---")
    passed = sum(1 for r in results if r.startswith("[PASS]"))
    total = len(results)
    print(f"{passed}/{total} passed")
    for r in results:
        print(r)


if __name__ == "__main__":
    main()
