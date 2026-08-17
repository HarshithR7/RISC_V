"""
RVV performance benchmark suite: for each benchmark, builds a scalar
(plain RV64I loop) and a vectorized (RVV) version of the same
computation, runs both through the simulator, and reports cycles,
instructions, vector instructions, memory operations, CPI, and speedup.

Methodology: correctness and performance are measured by two SEPARATE
programs per variant, not one. A single combined program (loop + a
`lw`+`check_eq` per output element) would report a cycle count polluted
by verification overhead having nothing to do with the loop itself.
Instead:
  - the "measure" program is just [preloaded data] + [the loop] + an
    unconditional pass -- its cycle count is the loop's real cost, full
    stop.
  - the "verify" program is [preloaded data] + [the loop] + real
    `check_eq` comparisons against Python-computed expected values --
    its only job is to prove the loop that got measured is actually
    correct; its cycle count is discarded.
Both programs share the exact same loop assembly text, so "verify passed"
is real evidence about the exact code whose cycles were measured, not a
similar-looking twin.

Input arrays are preloaded directly into data memory (bench_common's
pack_i32_array/preload_mem) rather than built with `li`+`sw` at runtime,
because `li` alone is 8 instructions per call in this assembler (see
asm64.py's `_li64_words`) -- an O(n) setup loop would swamp the cycle
counts for any n small enough to be a readable benchmark.

This core is single-cycle with no stalls, so CPI is always exactly 1.0
-- printed for completeness, not because it varies. The number that
actually reflects the vector unit's value is the speedup column.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build_tests64 import TestBuilder  # noqa: E402
from bench_common import preload_mem, build_and_run_bench, build_and_run_core as build_and_run  # noqa: E402

N = 16  # 4 full VLMAX=4 stripmine groups, no partial tail -- keeps the
        # Level 1/2 numbers simple to reason about by hand.


def scalar_binop(op, a_base, b_base, c_base, n):
    return f"""
li x5, {a_base}
li x6, {b_base}
li x7, {c_base}
li x8, {a_base + n * 4}
sb_{op}:
lw x28, 0(x5)
lw x29, 0(x6)
{op} x28, x28, x29
sw x28, 0(x7)
addi x5, x5, 4
addi x6, x6, 4
addi x7, x7, 4
bne x5, x8, sb_{op}
"""


def vector_binop(vop, a_base, b_base, c_base, n):
    tag = vop.replace(".", "_")
    return f"""
li x5, {a_base}
li x6, {b_base}
li x7, {c_base}
li x9, {n}
vb_{tag}:
vsetvli x10, x9, e32,m1
vle32.v v1, (x5)
vle32.v v2, (x6)
{vop} v1, v1, v2
vse32.v v1, (x7)
slli x11, x10, 2
sub x9, x9, x10
add x5, x5, x11
add x6, x6, x11
add x7, x7, x11
bnez x9, vb_{tag}
"""


def run_level1(op, vop, expect_fn):
    a_base, b_base, c_base = 0, 64, 128
    a = [100 + i for i in range(N)]
    b = [i + 1 for i in range(N)]
    dmem = preload_mem((a_base, a), (b_base, b))
    expected = [expect_fn(a[i], b[i]) & 0xFFFFFFFF for i in range(N)]

    scalar_asm = scalar_binop(op, a_base, b_base, c_base, N)
    vector_asm = vector_binop(vop, a_base, b_base, c_base, N)

    # Correctness first.
    for tag, body in [("scalar", scalar_asm), ("vector", vector_asm)]:
        vt = TestBuilder(f"l1_{op}_{tag}_verify")
        vt.asm(body)
        for i in range(N):
            vt.asm(f"lw x8, {c_base + i * 4}(x0)")
            vt.check_eq("x8", expected[i])
        line, log = build_and_run(vt, dmem_words=dmem)
        if not line.startswith("[PASS]"):
            raise RuntimeError(f"{vt.name} FAILED: {line}\n{log[-2000:]}")

    # Then pure cycle measurement.
    ms = TestBuilder(f"l1_{op}_scalar_measure")
    ms.asm(scalar_asm)
    mv = TestBuilder(f"l1_{op}_vector_measure")
    mv.asm(vector_asm)
    scalar_stats = build_and_run_bench(ms, dmem)
    vector_stats = build_and_run_bench(mv, dmem)
    return scalar_stats, vector_stats


def dot_product(a_base, b_base, n):
    return f"""
li x5, {a_base}
li x6, {b_base}
li x8, {a_base + n * 4}
li x20, 0
dot_scalar:
lw x28, 0(x5)
lw x29, 0(x6)
mul x28, x28, x29
add x20, x20, x28
addi x5, x5, 4
addi x6, x6, 4
bne x5, x8, dot_scalar
""", f"""
li x5, {a_base}
li x6, {b_base}
li x9, {n}
li x21, 3000
vle32.v v3, (x21)
dot_vector:
vsetvli x10, x9, e32,m1
vle32.v v1, (x5)
vle32.v v2, (x6)
vmul.vv v1, v1, v2
vredsum.vs v3, v1, v3
slli x11, x10, 2
sub x9, x9, x10
add x5, x5, x11
add x6, x6, x11
bnez x9, dot_vector
"""


def axpy(x_base, y_base, a_const, n):
    return f"""
li x5, {x_base}
li x6, {y_base}
li x8, {x_base + n * 4}
li x9, {a_const}
axpy_scalar:
lw x28, 0(x5)
mul x28, x28, x9
lw x29, 0(x6)
add x28, x28, x29
sw x28, 0(x6)
addi x5, x5, 4
addi x6, x6, 4
bne x5, x8, axpy_scalar
""", f"""
li x5, {x_base}
li x6, {y_base}
li x9, {n}
li x12, {a_const}
axpy_vector:
vsetvli x10, x9, e32,m1
vle32.v v1, (x5)
vle32.v v2, (x6)
vmul.vx v1, v1, x12
vadd.vv v1, v1, v2
vse32.v v1, (x6)
slli x11, x10, 2
sub x9, x9, x10
add x5, x5, x11
add x6, x6, x11
bnez x9, axpy_vector
"""


def reduction(a_base, n):
    return f"""
li x5, {a_base}
li x8, {a_base + n * 4}
li x20, 0
red_scalar:
lw x28, 0(x5)
add x20, x20, x28
addi x5, x5, 4
bne x5, x8, red_scalar
""", f"""
li x5, {a_base}
li x9, {n}
li x21, 3000
vle32.v v3, (x21)
red_vector:
vsetvli x10, x9, e32,m1
vle32.v v1, (x5)
vredsum.vs v3, v1, v3
slli x11, x10, 2
sub x9, x9, x10
add x5, x5, x11
bnez x9, red_vector
"""


def run_level2():
    results = {}

    # -- dot product --
    a_base, b_base = 0, 64
    a = [1 + i for i in range(N)]
    b = [2 + i for i in range(N)]
    dmem = preload_mem((a_base, a), (b_base, b))
    expected_dot = sum(a[i] * b[i] for i in range(N)) & 0xFFFFFFFF
    scalar_asm, vector_asm = dot_product(a_base, b_base, N)

    vt = TestBuilder("l2_dot_scalar_verify")
    vt.asm(scalar_asm).check_eq("x20", expected_dot)
    line, log = build_and_run(vt, dmem_words=dmem)
    if not line.startswith("[PASS]"):
        raise RuntimeError(f"l2_dot_scalar_verify FAILED: {line}\n{log[-2000:]}")

    vt = TestBuilder("l2_dot_vector_verify")
    vt.asm(vector_asm)
    vt.asm("""
li x22, 3200
vse32.v v3, (x22)
lw x8, 0(x22)
""").check_eq("x8", expected_dot)
    line, log = build_and_run(vt, dmem_words=dmem)
    if not line.startswith("[PASS]"):
        raise RuntimeError(f"l2_dot_vector_verify FAILED: {line}\n{log[-2000:]}")

    ms, mv = TestBuilder("l2_dot_scalar_measure"), TestBuilder("l2_dot_vector_measure")
    ms.asm(scalar_asm)
    mv.asm(vector_asm)
    results["dot_product"] = (build_and_run_bench(ms, dmem), build_and_run_bench(mv, dmem))

    # -- AXPY: y = a*x + y --
    x_base, y_base, a_const = 0, 64, 3
    x = [1 + i for i in range(N)]
    y = [10 + i for i in range(N)]
    dmem = preload_mem((x_base, x), (y_base, y))
    expected_y = [(a_const * x[i] + y[i]) & 0xFFFFFFFF for i in range(N)]
    scalar_asm, vector_asm = axpy(x_base, y_base, a_const, N)

    for tag, body in [("scalar", scalar_asm), ("vector", vector_asm)]:
        vt = TestBuilder(f"l2_axpy_{tag}_verify")
        vt.asm(body)
        for i in range(N):
            vt.asm(f"lw x8, {y_base + i * 4}(x0)")
            vt.check_eq("x8", expected_y[i])
        line, log = build_and_run(vt, dmem_words=dmem)
        if not line.startswith("[PASS]"):
            raise RuntimeError(f"l2_axpy_{tag}_verify FAILED: {line}\n{log[-2000:]}")

    ms, mv = TestBuilder("l2_axpy_scalar_measure"), TestBuilder("l2_axpy_vector_measure")
    ms.asm(scalar_asm)
    mv.asm(vector_asm)
    results["axpy"] = (build_and_run_bench(ms, dmem), build_and_run_bench(mv, dmem))

    # -- reduction: sum(A) --
    a_base = 0
    a = [1 + i for i in range(N)]
    dmem = preload_mem((a_base, a))
    expected_sum = sum(a) & 0xFFFFFFFF
    scalar_asm, vector_asm = reduction(a_base, N)

    vt = TestBuilder("l2_reduce_scalar_verify")
    vt.asm(scalar_asm).check_eq("x20", expected_sum)
    line, log = build_and_run(vt, dmem_words=dmem)
    if not line.startswith("[PASS]"):
        raise RuntimeError(f"l2_reduce_scalar_verify FAILED: {line}\n{log[-2000:]}")

    vt = TestBuilder("l2_reduce_vector_verify")
    vt.asm(vector_asm)
    vt.asm("""
li x22, 3200
vse32.v v3, (x22)
lw x8, 0(x22)
""").check_eq("x8", expected_sum)
    line, log = build_and_run(vt, dmem_words=dmem)
    if not line.startswith("[PASS]"):
        raise RuntimeError(f"l2_reduce_vector_verify FAILED: {line}\n{log[-2000:]}")

    ms, mv = TestBuilder("l2_reduce_scalar_measure"), TestBuilder("l2_reduce_vector_measure")
    ms.asm(scalar_asm)
    mv.asm(vector_asm)
    results["reduction"] = (build_and_run_bench(ms, dmem), build_and_run_bench(mv, dmem))

    return results


LEVEL1_OPS = [
    ("add", "vadd.vv", lambda a, b: a + b),
    ("sub", "vsub.vv", lambda a, b: a - b),
    ("mul", "vmul.vv", lambda a, b: a * b),
    ("and", "vand.vv", lambda a, b: a & b),
    ("or",  "vor.vv",  lambda a, b: a | b),
    ("xor", "vxor.vv", lambda a, b: a ^ b),
]


def fmt_row(name, s, v):
    speedup = s["cycles"] / v["cycles"]
    return (f"| {name} | {s['cycles']} | {s['instrs']} | 0 | {s['mem_ops']} | 1.00 "
            f"| {v['cycles']} | {v['instrs']} | {v['vec_instrs']} | {v['mem_ops']} | 1.00 "
            f"| {speedup:.2f}x |")


def fmt_bw_row(name, s, v):
    """Bytes/cycle is the frequency-independent number (real, derived
    directly from bytes_moved/cycles); bytes/second at any specific clock
    is only as credible as the frequency assumed, so it's reported
    separately and explicitly labeled, not baked into this table.
    """
    s_bpc = s["bytes_moved"] / s["cycles"]
    v_bpc = v["bytes_moved"] / v["cycles"]
    return (f"| {name} | {s['bytes_moved']} | {s_bpc:.2f} "
            f"| {v['bytes_moved']} | {v_bpc:.2f} | {v_bpc / s_bpc:.2f}x |")


def main():
    print(f"N = {N} elements per benchmark\n")
    rows = []

    print("Level 1: elementwise arithmetic/logic")
    for op, vop, fn in LEVEL1_OPS:
        s, v = run_level1(op, vop, fn)
        row = fmt_row(op, s, v)
        print(row)
        rows.append(("L1", op, s, v))

    print("\nLevel 2: dot product, AXPY, reduction")
    l2 = run_level2()
    for name in ["dot_product", "axpy", "reduction"]:
        s, v = l2[name]
        row = fmt_row(name, s, v)
        print(row)
        rows.append(("L2", name, s, v))

    print("\n--- markdown table ---")
    print("| Benchmark | Scalar cycles | Scalar instrs | Scalar vec instrs | Scalar mem ops | Scalar CPI | Vector cycles | Vector instrs | Vector instrs (vec) | Vector mem ops | Vector CPI | Speedup |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for level, name, s, v in rows:
        print(fmt_row(name, s, v))

    print("\n--- memory bandwidth (bytes/cycle -- frequency-independent) ---")
    print("| Benchmark | Scalar bytes moved | Scalar bytes/cycle | Vector bytes moved | Vector bytes/cycle | Bandwidth ratio |")
    print("|---|---|---|---|---|---|")
    for level, name, s, v in rows:
        print(fmt_bw_row(name, s, v))


if __name__ == "__main__":
    main()
