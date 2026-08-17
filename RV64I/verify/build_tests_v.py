"""
Test suite for the scoped RVV implementation (VLEN=128, SEW=32, LMUL=1
only -- see RV64I/README.md for why). There's no way to directly compare a
vector register in the check_eq machinery (same issue as F/D registers),
so every test reads results back out through scalar loads
(vse32.v -> lw) or vle32.v into memory a normal `lw` can then check.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build_tests64 import TestBuilder, build_and_run  # noqa: E402


def build_v_regs(t, base_addr, *vecs):
    """Loads each 4-element list in `vecs` into consecutive vector
    registers v1, v2, ... via memory (li + sw x4, then vle32.v). Uses x11
    (set to 4 = VLMAX, requesting the full vector) as vsetvli's AVL source
    -- NOT the address register: vsetvli's rs1 must hold the *requested
    vector length*, not a memory address. (An earlier version of this
    helper accidentally passed the address register instead, which meant
    vl silently became 0 whenever a vector's backing address happened to
    be 0 -- since rs1 was some register x6 holding the value 0, not
    literally x0, the "rs1==x0 means VLMAX" special case didn't trigger,
    and it computed vl=min(0,4)=0 instead.)"""
    addr = base_addr
    lines = ["li x11, 4"]
    for vi, vec in enumerate(vecs, start=1):
        for i, val in enumerate(vec):
            lines.append(f"li x5, {val}")
            lines.append(f"sw x5, {addr + i*4}(x0)")
        lines.append(f"li x6, {addr}")
        lines.append("vsetvli x7, x11, e32,m1")  # note: real syntax differs; asm64.py's vsetvli ignores the vtype text
        lines.append(f"vle32.v v{vi}, (x6)")
        addr += 32  # generous spacing between vectors' backing memory
    t.asm("\n".join(lines))
    return t


def check_vreg(t, vreg_num, expected, scratch_addr=512):
    """Stores vN to memory via vse32.v, then checks each of the 4 lanes
    with a normal lw + check_eq. Every expected value used in this suite
    is small and non-negative, so LW's sign-extension never actually
    triggers -- no need for a general sign-extension helper here."""
    t.asm(f"""
li x6, {scratch_addr}
vse32.v v{vreg_num}, (x6)
""")
    for i, val in enumerate(expected):
        assert 0 <= val < 0x80000000, "test helper assumes non-negative, non-sign-bit-set values"
        # x6 already holds scratch_addr; the offset here is just i*4 (not
        # scratch_addr+i*4 -- that would double-count the base address).
        t.asm(f"lw x8, {i*4}(x6)")
        t.check_eq("x8", val)
    return t


def t_v_arith():
    t = TestBuilder("v_arith")
    build_v_regs(t, 0, [1, 2, 3, 4], [10, 20, 30, 40])
    t.asm("vadd.vv v3, v1, v2")
    check_vreg(t, 3, [11, 22, 33, 44])
    return t


def t_v_sub_logic():
    t = TestBuilder("v_sub_logic")
    build_v_regs(t, 0, [10, 20, 30, 40], [1, 2, 3, 4])
    t.asm("vsub.vv v3, v1, v2")
    check_vreg(t, 3, [9, 18, 27, 36])

    build_v_regs(t, 128, [0xFF, 0xF0, 0x0F, 0xAA], [0x0F, 0x0F, 0xF0, 0x55])
    t.asm("vand.vv v3, v1, v2")
    check_vreg(t, 3, [0x0F, 0x00, 0x00, 0x00])
    return t


def t_v_scalar_imm():
    t = TestBuilder("v_scalar_imm")
    build_v_regs(t, 0, [1, 2, 3, 4])
    t.asm("li x9, 100\nvadd.vx v2, v1, x9")
    check_vreg(t, 2, [101, 102, 103, 104])

    build_v_regs(t, 128, [1, 2, 3, 4])
    t.asm("vadd.vi v2, v1, 5")
    check_vreg(t, 2, [6, 7, 8, 9])
    return t


def t_v_mul():
    t = TestBuilder("v_mul")
    build_v_regs(t, 0, [2, 3, 4, 5], [10, 10, 10, 10])
    t.asm("vmul.vv v3, v1, v2")
    check_vreg(t, 3, [20, 30, 40, 50])
    return t


def t_v_partial_vl():
    # vl < VLMAX: only the first vl elements are touched by vle32.v/vse32.v;
    # the rest of the destination stays whatever it was (memory) or reads
    # as 0 (tail-agnostic register fill).
    t = TestBuilder("v_partial_vl")
    t.asm("""
li x5, 0xAAAA
sw x5, 512(x0)
li x5, 0xBBBB
sw x5, 516(x0)
li x5, 0xCCCC
sw x5, 520(x0)
li x5, 0xDDDD
sw x5, 524(x0)
li x6, 512
li x10, 2
vsetvli x7, x10, e32,m1
vle32.v v1, (x6)
""")
    # element 0 and 1 loaded normally; elements 2,3 are tail-agnostic
    # (zero-filled by this implementation) since vl=2
    t.asm("""
li x6, 640
vse32.v v1, (x6)
lw x8, 640(x0)
""").check_eq("x8", "0xAAAA")
    t.asm("lw x8, 644(x0)").check_eq("x8", "0xBBBB")
    t.asm("lw x8, 648(x0)").check_eq("x8", 0)  # beyond vl=2 -> zero-filled on load
    return t


def t_v_app_dotprod():
    # A small application: sum(a[i]*b[i]) using vmul + a scalar reduction
    # loop over the vector result (no vector reduction instruction in this
    # scoped implementation).
    t = TestBuilder("v_app_dotprod")
    a = [1, 2, 3, 4]
    b = [10, 20, 30, 40]
    build_v_regs(t, 0, a, b)
    t.asm("""
vmul.vv v3, v1, v2
li x6, 640
vse32.v v3, (x6)
li x9, 0
lw x10, 640(x0)
add x9, x9, x10
lw x10, 644(x0)
add x9, x9, x10
lw x10, 648(x0)
add x9, x9, x10
lw x10, 652(x0)
add x9, x9, x10
""").check_eq("x9", sum(x * y for x, y in zip(a, b)))
    return t


def main():
    results = []
    logs = {}

    def run(builder):
        line, log = build_and_run(builder)
        results.append(line)
        logs[builder.name] = log
        print(line)

    run(t_v_arith())
    run(t_v_sub_logic())
    run(t_v_scalar_imm())
    run(t_v_mul())
    run(t_v_partial_vl())
    run(t_v_app_dotprod())

    print("\n--- SUMMARY ---")
    passed = sum(1 for r in results if r.startswith("[PASS]"))
    total = len(results)
    print(f"{passed}/{total} passed")
    for r in results:
        print(r)


if __name__ == "__main__":
    main()
