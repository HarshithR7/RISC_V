"""
Test suite for RVV Phase 2's remaining pieces: divide/remainder (vdivu/
vdiv/vremu/vrem) and shifts (vsll/vsrl/vsra). Compares, masks, min/max,
and reductions were already covered by build_tests_v2.py -- this suite
covers what was still missing to finish "Phase 2" of the roadmap.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build_tests64 import TestBuilder, build_and_run  # noqa: E402
from build_tests_v import build_v_regs, check_vreg  # noqa: E402


def u32(x):
    return x & 0xFFFFFFFF


def s32_to_s64hex(x):
    """A 32-bit two's-complement pattern, sign-extended to a 64-bit hex
    literal string. `lw` always sign-extends its 32-bit result to 64 bits
    (like every other load in this core), but `check_eq`'s `li` pseudo-op
    builds exactly the 64-bit value it's handed -- so comparing a lane
    with a negative value against a plain unsigned decimal (e.g.
    `u32(-5)` = 4294967291) would build the wrong scratch constant
    (zero-extended, not sign-extended) and fail even when the hardware is
    correct. Same `li` gotcha already documented in RV64I/README.md.
    """
    x &= 0xFFFFFFFF
    if x & 0x80000000:
        x |= 0xFFFFFFFF00000000
    return f"0x{x:016X}"


def check_vreg_signed(t, vreg_num, expected, scratch_addr=512):
    """Like build_tests_v.check_vreg, but for lanes that may have the
    sign bit set -- check_vreg's helper explicitly assumes non-negative
    values (see its assert) since nothing before this suite needed
    negative vector results. Reads each lane via `lw` and compares
    against its sign-extended 64-bit form via s32_to_s64hex.
    """
    t.asm(f"""
li x6, {scratch_addr}
vse32.v v{vreg_num}, (x6)
""")
    for i, val in enumerate(expected):
        t.asm(f"lw x8, {i*4}(x6)")
        t.check_eq("x8", s32_to_s64hex(val))
    return t


def t_v_divrem():
    t = TestBuilder("v_divrem")
    build_v_regs(t, 0, [20, 21, 7, 100], [4, 5, 3, 9])
    t.asm("vdivu.vv v3, v1, v2")
    check_vreg(t, 3, [5, 4, 2, 11])  # unsigned integer division, truncating
    t.asm("vremu.vv v3, v1, v2")
    check_vreg(t, 3, [0, 1, 1, 1])

    # Signed divide, including negative operands and results.
    build_v_regs(t, 128, [u32(-20), 21, u32(-7), 9], [4, 5, u32(-3), u32(-2)])
    t.asm("vdiv.vv v3, v1, v2")
    check_vreg_signed(t, 3, [u32(-5), 4, 2, u32(-4)])
    # -20/4=-5, 21/5=4, -7/-3=2, 9/-2=-4 -- all truncate toward 0, not floor
    return t


def t_v_div_zero_and_overflow():
    t = TestBuilder("v_div_zero_and_overflow")
    # Divide by zero: vdivu/vdiv -> all-ones quotient; vremu/vrem ->
    # dividend itself. Per-spec defined results, not a trap.
    build_v_regs(t, 0, [42, u32(-42), 0, 5], [0, 0, 0, 0])
    t.asm("vdivu.vv v3, v1, v2")
    check_vreg_signed(t, 3, [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF])
    t.asm("vdiv.vv v3, v1, v2")
    check_vreg_signed(t, 3, [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF])
    t.asm("vremu.vv v3, v1, v2")
    check_vreg_signed(t, 3, [42, u32(-42), 0, 5])
    t.asm("vrem.vv v3, v1, v2")
    check_vreg_signed(t, 3, [42, u32(-42), 0, 5])

    # Signed overflow: MIN_INT / -1 -> quotient = MIN_INT (not a trap or
    # wraparound to 0); MIN_INT % -1 -> remainder = 0.
    build_v_regs(t, 128, [0x80000000, 1, 1, 1], [u32(-1), 1, 1, 1])
    t.asm("vdiv.vv v3, v1, v2")
    check_vreg_signed(t, 3, [0x80000000, 1, 1, 1])
    build_v_regs(t, 256, [0x80000000, 1, 1, 1], [u32(-1), 1, 1, 1])
    t.asm("vrem.vv v3, v1, v2")
    check_vreg_signed(t, 3, [0, 0, 0, 0])
    return t


def t_v_shift():
    t = TestBuilder("v_shift")
    build_v_regs(t, 0, [1, 2, 4, 8])
    t.asm("vsll.vi v3, v1, 4")
    check_vreg(t, 3, [16, 32, 64, 128])

    build_v_regs(t, 128, [0x80000000, 0xFF000000, 4, 1024])
    t.asm("vsrl.vi v3, v1, 4")
    check_vreg(t, 3, [0x08000000, 0x0FF00000, 0, 64])  # logical: zero-fills from the top

    build_v_regs(t, 256, [0x80000000, 0xFFFFFFF0, 4, 1024])
    t.asm("vsra.vi v3, v1, 4")
    check_vreg_signed(t, 3, [0xF8000000, 0xFFFFFFFF, 0, 64])  # arithmetic: sign-extends from the top

    # Shift amount masking: RVV shift amounts (like scalar RISC-V) use
    # only the low log2(SEW)=5 bits -- a .vx shift amount of 33 behaves
    # identically to a shift amount of 1, not "shift out everything".
    build_v_regs(t, 384, [1, 1, 1, 1])
    t.asm("li x9, 33\nvsll.vx v3, v1, x9")
    check_vreg(t, 3, [2, 2, 2, 2])
    return t


def t_v_app_avg_filter():
    # A small application: average two vectors via add+shift (a common
    # vectorized idiom -- (a+b)>>1 avoids the intermediate overflow a
    # naive divide-by-2 read would have for values near 2^31, though this
    # test uses small values so both approaches would agree; it's here to
    # exercise add and shift composing in one instruction sequence).
    t = TestBuilder("v_app_avg_filter")
    build_v_regs(t, 0, [10, 20, 30, 40], [20, 40, 50, 80])
    t.asm("""
vadd.vv v3, v1, v2
vsrl.vi v3, v3, 1
""")
    check_vreg(t, 3, [15, 30, 40, 60])
    return t


def main():
    results = []
    logs = {}

    def run(builder):
        line, log = build_and_run(builder)
        results.append(line)
        logs[builder.name] = log
        print(line)

    run(t_v_divrem())
    run(t_v_div_zero_and_overflow())
    run(t_v_shift())
    run(t_v_app_avg_filter())

    print("\n--- SUMMARY ---")
    passed = sum(1 for r in results if r.startswith("[PASS]"))
    total = len(results)
    print(f"{passed}/{total} passed")
    for r in results:
        print(r)


if __name__ == "__main__":
    main()
