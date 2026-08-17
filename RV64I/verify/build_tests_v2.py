"""
Test suite for RVV Tier 2: masking (v0.t), vector compares (-> masks),
elementwise min/max, and reductions. Builds on build_tests_v.py's helpers.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build_tests64 import TestBuilder, build_and_run  # noqa: E402
from build_tests_v import build_v_regs, check_vreg  # noqa: E402


def check_vmask(t, vreg_num, expected_bits, scratch_addr=512):
    """Compare (vmseq/vmslt/...) results pack all 4 lane booleans into
    vd's low bits (lane 0 only), matching vcmp_result's {124'b0, bits}
    layout -- NOT one scalar per lane like arithmetic ops. check_vreg
    checks per-lane scalars, so it silently mis-checks compare results
    (only lane 0's word is meaningful; treating expected_bits as 4
    separate per-lane words happens to line up only when a single low
    bit is set). This reads just element 0 and compares the packed
    value directly.
    """
    packed = 0
    for i, b in enumerate(expected_bits):
        packed |= (b & 1) << i
    t.asm(f"""
li x6, {scratch_addr}
vse32.v v{vreg_num}, (x6)
lw x8, 0(x6)
""")
    t.check_eq("x8", packed)
    return t


def t_v_masking():
    t = TestBuilder("v_masking")
    # Masks are stored packed -- one bit per lane in v0's low 4 bits
    # (bits[3:0] of the whole 128-bit register, i.e. the low 4 bits of
    # lane 0), matching the format vcmp_result packs compare results
    # into. So loading the mask is a single-element vle32.v (vl=1) of
    # the packed value 0b0101, NOT a 4-element load of one 0/1 per lane
    # (that would put each 0/1 in its own 32-bit lane, which is not
    # where the hardware's mask-read logic looks).
    # mask = 0b0101: lanes 0 and 2 active, 1 and 3 not.
    t.asm("""
li x5, 0b0101
sw x5, 800(x0)
li x10, 1
vsetvli x7, x10, e32,m1
li x6, 800
vle32.v v0, (x6)
""")
    build_v_regs(t, 0, [1, 2, 3, 4], [10, 20, 30, 40])
    t.asm("vadd.vv v3, v1, v2, v0.t")
    # lanes 0,2 get the sum; lanes 1,3 are masked out -> agnostic zero
    check_vreg(t, 3, [11, 0, 33, 0])
    return t


def t_v_compare():
    t = TestBuilder("v_compare")
    build_v_regs(t, 0, [5, 10, 15, 20], [10, 10, 10, 10])
    t.asm("vmslt.vv v3, v1, v2")  # v1 < v2 ? -> [1,0,0,0] (only 5<10)
    check_vmask(t, 3, [1, 0, 0, 0])

    build_v_regs(t, 128, [5, 10, 15, 20], [10, 10, 10, 10])
    t.asm("vmseq.vv v3, v1, v2")  # equal only at lane 1 (10==10)
    check_vmask(t, 3, [0, 1, 0, 0])

    build_v_regs(t, 256, [1, 2, 3, 4])
    t.asm("vmsltu.vi v3, v1, 3")  # unsigned < 3 -> lanes with value 1,2
    check_vmask(t, 3, [1, 1, 0, 0])
    return t


def t_v_minmax():
    t = TestBuilder("v_minmax")
    build_v_regs(t, 0, [5, 20, 3, 40], [10, 10, 10, 10])
    t.asm("vmin.vv v3, v1, v2")
    check_vreg(t, 3, [5, 10, 3, 10])
    t.asm("vmax.vv v3, v1, v2")
    check_vreg(t, 3, [10, 20, 10, 40])
    return t


def t_v_reduce():
    t = TestBuilder("v_reduce")
    build_v_regs(t, 0, [1, 2, 3, 4])
    # vredsum.vs vd, vs2, vs1: sum(vs2) + vs1[0]. Use v2 (all zero from
    # reset... but v2 isn't loaded here) -- load a zero seed explicitly.
    t.asm("""
li x5, 0
sw x5, 512(x0)
sw x5, 516(x0)
sw x5, 520(x0)
sw x5, 524(x0)
li x6, 512
vle32.v v2, (x6)
vredsum.vs v3, v1, v2
""")
    check_vreg(t, 3, [10, 0, 0, 0])  # 1+2+3+4 = 10, in element 0 only

    build_v_regs(t, 640, [5, 20, 3, 40])
    t.asm("""
li x5, 0
sw x5, 512(x0)
li x6, 512
vle32.v v2, (x6)
vredmax.vs v3, v1, v2
""")
    check_vreg(t, 3, [40, 0, 0, 0])

    build_v_regs(t, 768, [5, 20, 3, 40])
    t.asm("""
li x5, 100
sw x5, 512(x0)
li x6, 512
vle32.v v2, (x6)
vredmin.vs v3, v1, v2
""")
    check_vreg(t, 3, [3, 0, 0, 0])  # min(5,20,3,40,seed=100) = 3
    return t


def t_v_app_masked_filter():
    # A small application: zero out array elements below a threshold using
    # a compare-generated mask, then masked vadd (add 0, i.e. identity) to
    # demonstrate mask reuse across two different instructions.
    t = TestBuilder("v_app_masked_filter")
    build_v_regs(t, 0, [1, 8, 3, 9])
    t.asm("""
li x9, 5
vmslt.vi v0, v1, 5
vadd.vi v2, v1, 0, v0.t
""")
    # elements < 5 (1,3, at lanes 0,2) get copied via masked add-zero;
    # elements >= 5 (8,9, at lanes 1,3) are masked out -> agnostic zero
    check_vreg(t, 2, [1, 0, 3, 0])
    return t


def main():
    results = []
    logs = {}

    def run(builder):
        line, log = build_and_run(builder)
        results.append(line)
        logs[builder.name] = log
        print(line)

    run(t_v_masking())
    run(t_v_compare())
    run(t_v_minmax())
    run(t_v_reduce())
    run(t_v_app_masked_filter())

    print("\n--- SUMMARY ---")
    passed = sum(1 for r in results if r.startswith("[PASS]"))
    total = len(results)
    print(f"{passed}/{total} passed")
    for r in results:
        print(r)


if __name__ == "__main__":
    main()
