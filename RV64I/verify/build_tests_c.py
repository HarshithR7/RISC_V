"""
Test suite for the RV64C extension (compressed 16-bit instructions).
Every test freely mixes c.* (compressed) and regular 32-bit instructions in
the same program -- exactly how real RVC code looks -- which exercises the
halfword-granularity instruction fetch and the compressed_decoder expansion
together, not just the decoder in isolation.

c.li/c.addi/c.andi only encode a 6-bit signed immediate (-32..31); building
anything larger uses the regular `li` (which the assembler already knows
how to do for arbitrary 64-bit constants) followed by `c.mv` into the
target register -- exactly what a real compiler does when a constant
doesn't fit the compressed form.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build_tests64 import TestBuilder, build_and_run  # noqa: E402


def t_c_arith():
    t = TestBuilder("c_arith")
    # c.li / c.addi / c.mv / c.add on full 5-bit registers
    t.asm("""
c.li x5, 10
c.addi x5, 5
c.li x6, 0
c.mv x6, x5
c.add x6, x5
""").check_eq("x6", 30)  # x5 becomes 15, x6 = 15 (mv) then 15+15 = 30

    # c.sub/c.xor/c.or/c.and on compressed (x8-x15) registers
    t.asm("""
c.li x8, 20
c.li x9, 8
c.sub x8, x9
""").check_eq("x8", 12)

    t.asm("""
li x1, 0xF0
c.mv x10, x1
li x1, 0x0F
c.mv x11, x1
c.xor x10, x11
""").check_eq("x10", "0xFF")

    t.asm("""
li x1, 0xF0
c.mv x10, x1
li x1, 0xFF
c.mv x11, x1
c.and x10, x11
""").check_eq("x10", "0xF0")

    t.asm("""
li x1, 0xF0
c.mv x10, x1
li x1, 0x0F
c.mv x11, x1
c.or x10, x11
""").check_eq("x10", "0xFF")
    return t


def t_c_word_ops():
    t = TestBuilder("c_word_ops")
    t.asm("""
li x1, 100
c.mv x5, x1
c.addiw x5, 23
""").check_eq("x5", 123)
    t.asm("""
li x1, 100
c.mv x8, x1
c.li x9, 23
c.addw x8, x9
""").check_eq("x8", 123)
    t.asm("""
li x1, 100
c.mv x8, x1
c.li x9, 23
c.subw x8, x9
""").check_eq("x8", 77)
    # ADDW-style 32-bit overflow wrap + sign-extend, same corner case as
    # the regular addw test, now via the compressed encoding.
    t.asm("""
li x1, 0x7FFFFFFF
c.mv x8, x1
c.li x9, 1
c.addw x8, x9
""").check_eq("x8", "0xFFFFFFFF80000000")
    return t


def t_c_shift_upper():
    t = TestBuilder("c_shift_upper")
    t.asm("""
c.li x5, 1
c.slli x5, 10
""").check_eq("x5", 1024)
    t.asm("""
li x1, 0x80
c.mv x8, x1
c.srli x8, 4
""").check_eq("x8", 8)
    t.asm("""
li x1, 0x8000000000000000
c.mv x8, x1
c.srai x8, 4
""").check_eq("x8", "0xF800000000000000")
    t.asm("""
li x1, 0xFF
c.mv x8, x1
c.andi x8, 0x0F
""").check_eq("x8", 0x0F)
    # c.lui: builds bits[17:12] of the result (here, a small positive value)
    t.asm("c.lui x5, 5").check_eq("x5", "0x5000")
    # c.addi16sp: adjusts sp by a multiple of 16
    t.asm("""
c.mv x9, x2
c.addi16sp -32
addi x10, x2, 32
c.mv x2, x9
""").check_eq("x10", "0x0")  # sp restored via x10 = (sp-32)+32 = original sp = 0 in this test
    return t


def t_c_mem():
    t = TestBuilder("c_mem")
    # c.sw/c.lw via compressed base register (x8-x15), c.addi4spn to build a
    # compressed-encodable pointer from sp
    t.asm("""
c.addi4spn x8, 16
li x1, 0x1234
c.mv x9, x1
c.sw x9, 0(x8)
c.lw x10, 0(x8)
""").check_eq("x10", "0x1234")

    t.asm("""
c.addi4spn x8, 24
li x1, 0x1122334455667788
sd x1, 0(x8)
c.ld x9, 0(x8)
""").check_eq("x9", "0x1122334455667788")

    # *SP forms
    t.asm("""
li x1, 0xABCD
c.mv x5, x1
c.swsp x5, 32
c.lwsp x6, 32
""").check_eq("x6", "0xABCD")

    t.asm("""
li x1, 0x1122334455667788
c.sdsp x1, 40
c.ldsp x7, 40
""").check_eq("x7", "0x1122334455667788")
    return t


def t_c_branch_jump():
    t = TestBuilder("c_branch_jump")
    t.asm("""
c.li x8, 0
c.li x9, 5
c.beqz x9, bad
c.li x8, 1
c.j done
bad:
c.li x8, 2
done:
""").check_eq("x8", 1)  # x9 != 0, so beqz not taken

    t.asm("""
c.li x8, 0
c.li x9, 0
c.bnez x9, bad
c.li x8, 1
c.j done2
bad:
c.li x8, 2
done2:
""").check_eq("x8", 1)  # x9 == 0, so bnez not taken

    # c.jr / c.jalr. Address layout: c.li(2)+auipc(4)+addi(4)+c.jr(2)+
    # c.li(2)+c.j(2)+c.li(2) -- auipc is at offset 2, and the target
    # "c.li x10,2" is at offset 2+4+4+2+2+2=16, so the addi needs +14
    # (16-2) added to auipc's own address to land there.
    t.asm("""
c.li x10, 0
auipc x11, 0
addi x11, x11, 14
c.jr x11
c.li x10, 1
c.j jr_done
c.li x10, 2
jr_done:
""").check_eq("x10", 2)  # c.jr lands exactly on "c.li x10,2"
    return t


def t_c_mixed_program():
    # A small application mixing compressed and regular 32-bit instructions
    # freely (as a real compiler would): GCD via compressed instructions
    # for the common case, regular ones for anything not compressible
    # (462/1071 don't fit c.li's 6-bit immediate).
    t = TestBuilder("c_mixed_gcd")
    t.asm("""
li x8, 462
li x9, 1071
loop:
beq x8, x9, done
blt x8, x9, swap
c.sub x8, x9
c.j loop
swap:
sub x9, x9, x8
c.j loop
done:
""").check_eq("x8", 21)
    return t


def main():
    results = []
    logs = {}

    def run(builder):
        line, log = build_and_run(builder)
        results.append(line)
        logs[builder.name] = log
        print(line)

    run(t_c_arith())
    run(t_c_word_ops())
    run(t_c_shift_upper())
    run(t_c_mem())
    run(t_c_branch_jump())
    run(t_c_mixed_program())

    print("\n--- SUMMARY ---")
    passed = sum(1 for r in results if r.startswith("[PASS]"))
    total = len(results)
    print(f"{passed}/{total} passed")
    for r in results:
        print(r)


if __name__ == "__main__":
    main()
