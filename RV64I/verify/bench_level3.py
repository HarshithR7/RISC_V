"""
Level 3 benchmarks: matrix-vector multiply, matrix multiplication, 1D
convolution, and a separable 2D image filter -- same scalar-vs-vector,
measure-vs-verify methodology as bench_rvv.py's Level 1/2 (see that
file's module docstring for the full rationale). All algorithms here are
deliberately restructured around this scoped core's real constraints
(unit-stride-only vector memory access, 8-byte/doubleword alignment
requirement) rather than assuming an idealized vector unit that doesn't
have them -- see each benchmark's comment for what that restructuring
looks like and why.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build_tests64 import TestBuilder  # noqa: E402
from bench_common import preload_mem, build_and_run_bench, build_and_run_core as build_and_run  # noqa: E402


# ---------------------------------------------------------------------------
# Matrix-vector multiply: y = A @ x, A is MxK, x is length K, y is length M.
# Each row's dot product reuses the same vmul.vv + chained vredsum.vs
# pattern as bench_rvv.py's dot_product -- no new architectural issue here
# (row access is unit-stride, x is reused unmodified for every row).
# ---------------------------------------------------------------------------
M1, K1 = 4, 16


def matvec_asm(a_base, x_base, y_base, m, k):
    scalar = f"""
li x5, {a_base}
li x6, {x_base}
li x7, {y_base}
li x8, {x_base + k * 4}
li x24, {a_base + m * k * 4}
mv_scalar_outer:
add x20, x0, x0
mv_scalar_inner:
lw x28, 0(x5)
lw x29, 0(x6)
mul x28, x28, x29
add x20, x20, x28
addi x5, x5, 4
addi x6, x6, 4
bne x6, x8, mv_scalar_inner
sw x20, 0(x7)
addi x7, x7, 4
addi x6, x6, {-k * 4}
bne x5, x24, mv_scalar_outer
"""
    vector = f"""
li x5, {a_base}
li x6, {x_base}
li x7, {y_base}
li x24, {a_base + m * k * 4}
li x21, 4096
li x22, 4104
mv_vector_outer:
li x9, {k}
vle32.v v3, (x21)
mv_vector_inner:
vsetvli x10, x9, e32,m1
vle32.v v1, (x5)
vle32.v v2, (x6)
vmul.vv v1, v1, v2
vredsum.vs v3, v1, v3
slli x11, x10, 2
sub x9, x9, x10
add x5, x5, x11
add x6, x6, x11
bnez x9, mv_vector_inner
vse32.v v3, (x22)
lw x28, 0(x22)
sw x28, 0(x7)
addi x7, x7, 4
addi x6, x6, {-k * 4}
bne x5, x24, mv_vector_outer
"""
    return scalar, vector


def run_matvec():
    a_base, x_base, y_base = 0, 512, 576
    A = [[1 + (i * K1 + j) % 13 for j in range(K1)] for i in range(M1)]
    x = [2 + j for j in range(K1)]
    a_flat = [A[i][j] for i in range(M1) for j in range(K1)]
    dmem = preload_mem((a_base, a_flat), (x_base, x))
    expected_y = [sum(A[i][j] * x[j] for j in range(K1)) & 0xFFFFFFFF for i in range(M1)]

    scalar_asm, vector_asm = matvec_asm(a_base, x_base, y_base, M1, K1)

    for tag, body in [("scalar", scalar_asm), ("vector", vector_asm)]:
        vt = TestBuilder(f"l3_matvec_{tag}_verify")
        vt.asm(body)
        for i in range(M1):
            vt.asm(f"lw x8, {y_base + i * 4}(x0)")
            vt.check_eq("x8", expected_y[i])
        line, log = build_and_run(vt, dmem_words=dmem)
        if not line.startswith("[PASS]"):
            raise RuntimeError(f"{vt.name} FAILED: {line}\n{log[-2000:]}")

    ms, mv = TestBuilder("l3_matvec_scalar_measure"), TestBuilder("l3_matvec_vector_measure")
    ms.asm(scalar_asm)
    mv.asm(vector_asm)
    return build_and_run_bench(ms, dmem), build_and_run_bench(mv, dmem)


# ---------------------------------------------------------------------------
# Matrix multiplication: C = A @ B, A is MxK, B is KxN, C is MxN.
# A naive row-times-column formulation needs COLUMN access on B, which is
# not unit-stride -- this scoped core has no strided/indexed vector loads.
# Restructured as row-broadcast accumulation instead (a real, standard
# vectorized-GEMM technique, not a workaround invented for this limit):
# for each row i, C[i,:] = sum_k A[i,k] * B[k,:] -- every access (A
# linearly, B row-by-row, C row-by-row) is unit-stride. Each (i,k) step is
# structurally an AXPY (y = a*x + y) with a runtime-loaded scalar `a`.
# ---------------------------------------------------------------------------
M2, K2, N2 = 4, 4, 8


def matmul_asm(a_base, b_base, c_base, m, k, n):
    scalar = f"""
li x5, {a_base}
li x26, {c_base}
li x25, {a_base + m * k * 4}
mm_scalar_outer_i:
li x19, {b_base}
li x27, {k}
mm_scalar_k:
lw x28, 0(x5)
addi x5, x5, 4
add x16, x26, x0
addi x8, x16, {n * 4}
mm_scalar_j:
lw x29, 0(x19)
mul x29, x29, x28
lw x30, 0(x16)
add x30, x30, x29
sw x30, 0(x16)
addi x19, x19, 4
addi x16, x16, 4
bne x16, x8, mm_scalar_j
addi x27, x27, -1
bnez x27, mm_scalar_k
addi x26, x26, {n * 4}
bne x5, x25, mm_scalar_outer_i
"""
    vector = f"""
li x5, {a_base}
li x26, {c_base}
li x25, {a_base + m * k * 4}
mm_vector_outer_i:
li x19, {b_base}
li x27, {k}
mm_vector_k:
li x9, {n}
lw x28, 0(x5)
addi x5, x5, 4
add x16, x26, x0
mm_vector_j:
vsetvli x10, x9, e32,m1
vle32.v v1, (x19)
vmul.vx v1, v1, x28
vle32.v v2, (x16)
vadd.vv v1, v1, v2
vse32.v v1, (x16)
slli x11, x10, 2
sub x9, x9, x10
add x19, x19, x11
add x16, x16, x11
bnez x9, mm_vector_j
addi x27, x27, -1
bnez x27, mm_vector_k
addi x26, x26, {n * 4}
bne x5, x25, mm_vector_outer_i
"""
    return scalar, vector


def run_matmul():
    a_base, b_base, c_base = 0, 512, 1024
    A = [[1 + (i * K2 + j) % 5 for j in range(K2)] for i in range(M2)]
    B = [[1 + (i * N2 + j) % 7 for j in range(N2)] for i in range(K2)]
    a_flat = [A[i][j] for i in range(M2) for j in range(K2)]
    b_flat = [B[i][j] for i in range(K2) for j in range(N2)]
    dmem = preload_mem((a_base, a_flat), (b_base, b_flat))
    expected_C = [[sum(A[i][k] * B[k][j] for k in range(K2)) & 0xFFFFFFFF
                   for j in range(N2)] for i in range(M2)]

    scalar_asm, vector_asm = matmul_asm(a_base, b_base, c_base, M2, K2, N2)

    for tag, body in [("scalar", scalar_asm), ("vector", vector_asm)]:
        vt = TestBuilder(f"l3_matmul_{tag}_verify")
        vt.asm(body)
        for i in range(M2):
            for j in range(N2):
                vt.asm(f"lw x8, {c_base + (i * N2 + j) * 4}(x0)")
                vt.check_eq("x8", expected_C[i][j])
        line, log = build_and_run(vt, dmem_words=dmem)
        if not line.startswith("[PASS]"):
            raise RuntimeError(f"{vt.name} FAILED: {line}\n{log[-2000:]}")

    ms, mv = TestBuilder("l3_matmul_scalar_measure"), TestBuilder("l3_matmul_vector_measure")
    ms.asm(scalar_asm)
    mv.asm(vector_asm)
    return build_and_run_bench(ms, dmem), build_and_run_bench(mv, dmem)


# ---------------------------------------------------------------------------
# 1D convolution (FIR filter): y[i] = sum_k w[k] * x[i+k], K taps, N outputs.
# The natural vectorization -- vectorize over output position i, loop over
# taps k, reading a window of x shifted by k elements each tap -- runs
# straight into this core's 8-byte (2-element) vector-load alignment
# requirement: a shift by an ODD number of elements (k=1 here) moves the
# read address by 4 bytes, which is not doubleword-aligned, and this
# scoped core has no unaligned/strided vector load to fall back on.
#
# Fixed the same way real vectorized convolution implementations handle
# this (im2col-style data layout, not a workaround invented for this
# core): K separate, individually 8-byte-aligned copies of the shifted
# input windows are prepared in memory *before* the vectorized loop runs
# (Python-side here, standing in for what a real compiler's data-layout
# pass would do), so every tap's read is a plain aligned unit-stride load.
# The scalar version needs none of this -- byte-addressed scalar loads
# have no alignment restriction -- and reads the single unshifted array
# directly, which is also the honest comparison: the vector version's
# extra data-layout step is a real cost of this architecture, not hidden.
# ---------------------------------------------------------------------------
NCONV, KTAPS = 16, 3
CONV_W = [1, 2, 1]


def conv1d_asm(x_base, shift_bases, w, y_base, n):
    scalar = f"""
li x20, {w[0]}
li x21, {w[1]}
li x22, {w[2]}
li x5, {x_base}
li x7, {y_base}
li x24, {y_base + n * 4}
conv_scalar_loop:
lw x28, 0(x5)
mul x28, x28, x20
lw x29, 4(x5)
mul x29, x29, x21
add x28, x28, x29
lw x29, 8(x5)
mul x29, x29, x22
add x28, x28, x29
sw x28, 0(x7)
addi x5, x5, 4
addi x7, x7, 4
bne x7, x24, conv_scalar_loop
"""
    vector_taps = []
    for k in range(KTAPS):
        first = (k == 0)
        body = f"""
li x5, {shift_bases[k]}
li x7, {y_base}
li x9, {n}
cv_vector_tap{k}:
vsetvli x10, x9, e32,m1
vle32.v v1, (x5)
vmul.vx v1, v1, x2{k}
""" + ("" if first else """
vle32.v v2, (x7)
vadd.vv v1, v1, v2
""") + f"""
vse32.v v1, (x7)
slli x11, x10, 2
sub x9, x9, x10
add x5, x5, x11
add x7, x7, x11
bnez x9, cv_vector_tap{k}
"""
        vector_taps.append(body)
    vector = f"li x20, {w[0]}\nli x21, {w[1]}\nli x22, {w[2]}\n" + "".join(vector_taps)
    return scalar, vector


def run_conv1d():
    x_base = 0
    x = [1 + i for i in range(NCONV + KTAPS - 1)]
    shift_bases = [256 + k * (NCONV * 4) for k in range(KTAPS)]
    shifted = [x[k:k + NCONV] for k in range(KTAPS)]
    y_base = 448  # past x_base(0..72)/shift_bases(256..448); small enough to
                  # fit a 12-bit lw immediate directly from x0 in the verify
                  # check loop, and 8-byte aligned (vse32.v writes here)

    dmem_arrays = [(x_base, x)] + [(shift_bases[k], shifted[k]) for k in range(KTAPS)]
    dmem = preload_mem(*dmem_arrays)
    expected_y = [sum(CONV_W[k] * x[i + k] for k in range(KTAPS)) & 0xFFFFFFFF for i in range(NCONV)]

    scalar_asm, vector_asm = conv1d_asm(x_base, shift_bases, CONV_W, y_base, NCONV)

    for tag, body in [("scalar", scalar_asm), ("vector", vector_asm)]:
        vt = TestBuilder(f"l3_conv1d_{tag}_verify")
        vt.asm(body)
        for i in range(NCONV):
            vt.asm(f"lw x8, {y_base + i * 4}(x0)")
            vt.check_eq("x8", expected_y[i])
        line, log = build_and_run(vt, dmem_words=dmem)
        if not line.startswith("[PASS]"):
            raise RuntimeError(f"{vt.name} FAILED: {line}\n{log[-2000:]}")

    ms, mv = TestBuilder("l3_conv1d_scalar_measure"), TestBuilder("l3_conv1d_vector_measure")
    ms.asm(scalar_asm)
    mv.asm(vector_asm)
    return build_and_run_bench(ms, dmem), build_and_run_bench(mv, dmem)


# ---------------------------------------------------------------------------
# Separable 2D image filter (3x3 box sum), built on the same "K-tap
# weighted flat accumulate" pattern as conv1d, applied twice: a horizontal
# pass (row-aware, 3 adjacent-column taps) then a vertical pass (3
# adjacent-*row* taps). Key insight for the vertical pass: a shift by one
# whole row is `row_stride_bytes` = N*4, always a multiple of 8 for an
# even row width N -- so unlike the horizontal case, vertical taps need no
# pre-shifted copies at all; each tap is just a different constant offset
# into the *same* intermediate array. Both scalar and vector vertical
# passes reuse the identical flat_tap_sum_asm generator below with
# tap_stride_bytes=N*4 and no copies; only the horizontal pass needs
# per-tap copies (tap_stride_bytes=4, not 8-aligned) and only its scalar
# variant needs an explicit row loop (see its own comment for why the
# vector variant doesn't).
# ---------------------------------------------------------------------------
def flat_tap_sum_asm(bases, w_regs, y_base, n, tag):
    """K-tap weighted accumulate over a flat n-element region: y[i] =
    sum_k w[k]*mem[bases[k]+i]. `bases` gives each tap's OWN base address
    (already correctly spaced -- adjacent-column pre-shifted copies for a
    horizontal pass, or plain N*4-byte-apart offsets into one array for a
    vertical pass); this generator doesn't need to know which. w_regs is
    a list of register names already holding each tap's weight (caller
    loads them once, outside any loop, to avoid `li`'s x28-scratch-clobber
    trap -- see the matmul vector routine's fix above for why that order
    matters). `tag` must be unique per call site whose output ends up
    concatenated into the same assembled program (e.g. "h"/"v" for the
    image filter's horizontal/vertical passes): Assembler64's label table
    is one flat namespace per program, and this function's loop labels
    were originally unqualified (`ft_vector_tap0` etc, unconditionally) --
    calling it twice for two different passes in the same program made
    the second call's labels silently overwrite the first's in the label
    table, so the first pass's own loop branches resolved to the SECOND
    pass's code instead of looping back on themselves. Caught because
    each pass, tested in total isolation, worked -- only the combined
    two-pass program failed, which is exactly the signature of a label
    collision rather than a logic error in either pass.
    """
    scalar_taps = []
    for k, base in enumerate(bases):
        first = (k == 0)
        scalar_taps.append(f"""
li x5, {base}
li x7, {y_base}
li x24, {y_base + n * 4}
ft_scalar_{tag}_tap{k}:
lw x28, 0(x5)
mul x28, x28, {w_regs[k]}
""" + ("" if first else """
lw x29, 0(x7)
add x28, x28, x29
""") + f"""
sw x28, 0(x7)
addi x5, x5, 4
addi x7, x7, 4
bne x7, x24, ft_scalar_{tag}_tap{k}
""")
    scalar = "".join(scalar_taps)

    vector_taps = []
    for k, base in enumerate(bases):
        first = (k == 0)
        vector_taps.append(f"""
li x5, {base}
li x7, {y_base}
li x9, {n}
ft_vector_{tag}_tap{k}:
vsetvli x10, x9, e32,m1
vle32.v v1, (x5)
vmul.vx v1, v1, {w_regs[k]}
""" + ("" if first else """
vle32.v v2, (x7)
vadd.vv v1, v1, v2
""") + f"""
vse32.v v1, (x7)
slli x11, x10, 2
sub x9, x9, x10
add x5, x5, x11
add x7, x7, x11
bnez x9, ft_vector_{tag}_tap{k}
""")
    vector = "".join(vector_taps)
    return scalar, vector


IMG_R, IMG_CIN = 10, 18       # input image: 10 rows x 18 cols
IMG_N = IMG_CIN - KTAPS + 1   # 16: horizontal valid-conv output width
IMG_ROUT = IMG_R - KTAPS + 1  # 8: vertical valid-conv output height
IMG_W = [1, 1, 1]             # uniform box weights, both passes


def imgfilter_horizontal_scalar_asm(img_base, interm_base, r, c_in, n):
    """Row-aware: byte-offset taps (0/4/8) need no alignment care for
    scalar loads, but still must not read across a row boundary, so this
    walks an explicit outer loop over rows, resetting the row pointer
    between them -- unlike the vector horizontal pass (below), which
    sidesteps needing row-awareness in the *assembly* entirely by using
    already-row-correct pre-shifted copies instead.
    """
    return f"""
li x20, {IMG_W[0]}
li x21, {IMG_W[1]}
li x22, {IMG_W[2]}
li x5, {img_base}
li x7, {interm_base}
li x25, {img_base + r * c_in * 4}
imf_h_scalar_outer:
li x8, {n}
imf_h_scalar_inner:
lw x28, 0(x5)
mul x28, x28, x20
lw x29, 4(x5)
mul x29, x29, x21
add x28, x28, x29
lw x29, 8(x5)
mul x29, x29, x22
add x28, x28, x29
sw x28, 0(x7)
addi x5, x5, 4
addi x7, x7, 4
addi x8, x8, -1
bnez x8, imf_h_scalar_inner
addi x5, x5, {(c_in - n) * 4}
bne x5, x25, imf_h_scalar_outer
"""


def run_imgfilter():
    img_base = 0
    img = [[1 + (r * IMG_CIN + c) % 11 for c in range(IMG_CIN)] for r in range(IMG_R)]
    img_flat = [img[r][c] for r in range(IMG_R) for c in range(IMG_CIN)]

    # Horizontal pass's vector variant needs 3 pre-shifted, row-correct
    # copies (im2col-style, same reasoning as conv1d): h_shift_k is the
    # full RxN array where h_shift_k[r][c] = img[r][c+k].
    h_shift_bases = [4096 + k * (IMG_R * IMG_N * 4) for k in range(KTAPS)]
    h_shift_flat = [[img[r][c + k] for r in range(IMG_R) for c in range(IMG_N)]
                     for k in range(KTAPS)]

    dmem = preload_mem((img_base, img_flat),
                        *[(h_shift_bases[k], h_shift_flat[k]) for k in range(KTAPS)])

    interm_base = 16384       # R*N = 160 elements
    interm_stride = IMG_N * 4  # 64 bytes/row -- always 8-aligned (N even)
    v_bases = [interm_base + dr * interm_stride for dr in range(KTAPS)]
    out_base = 20480          # ROUT*N = 128 elements

    expected_interm = [[sum(IMG_W[k] * img[r][c + k] for k in range(KTAPS)) & 0xFFFFFFFF
                         for c in range(IMG_N)] for r in range(IMG_R)]
    expected_out = [[sum(IMG_W[dr] * expected_interm[r + dr][c] for dr in range(KTAPS)) & 0xFFFFFFFF
                      for c in range(IMG_N)] for r in range(IMG_ROUT)]

    h_scalar = imgfilter_horizontal_scalar_asm(img_base, interm_base, IMG_R, IMG_CIN, IMG_N)
    _, h_vector = flat_tap_sum_asm(h_shift_bases, ["x20", "x21", "x22"], interm_base, IMG_R * IMG_N, "h")
    h_vector = f"li x20, {IMG_W[0]}\nli x21, {IMG_W[1]}\nli x22, {IMG_W[2]}\n" + h_vector

    v_scalar, v_vector = flat_tap_sum_asm(v_bases, ["x20", "x21", "x22"], out_base, IMG_ROUT * IMG_N, "v")
    v_scalar = f"li x20, {IMG_W[0]}\nli x21, {IMG_W[1]}\nli x22, {IMG_W[2]}\n" + v_scalar
    v_vector = f"li x20, {IMG_W[0]}\nli x21, {IMG_W[1]}\nli x22, {IMG_W[2]}\n" + v_vector

    scalar_asm = h_scalar + v_scalar
    vector_asm = h_vector + v_vector

    # Checking all ROUT*N=128 output elements would need ~128 check_eq
    # calls; each is a `li`+`bne` pair, and bne's branch immediate is only
    # 13 bits signed (+-4KB) -- early checks' jumps to their fail handler
    # (placed after *all* checks, at the very end of the program) overflow
    # that range once the program gets this large. A representative
    # sample (both corners, both edges' midpoints, and the center) is
    # real, direct evidence of correctness at the boundary and interior
    # cases that would most likely expose an off-by-one in the row/column
    # bookkeeping, without exhaustively checking all 128 positions.
    sample_points = [(0, 0), (0, IMG_N - 1), (IMG_ROUT - 1, 0), (IMG_ROUT - 1, IMG_N - 1),
                      (IMG_ROUT // 2, IMG_N // 2)]
    for tag, body in [("scalar", scalar_asm), ("vector", vector_asm)]:
        vt = TestBuilder(f"l3_imgfilter_{tag}_verify")
        vt.asm(body)
        vt.asm(f"li x6, {out_base}")  # out_base exceeds the 12-bit lw
                                       # immediate range from x0 directly
        for r, c in sample_points:
            vt.asm(f"lw x8, {(r * IMG_N + c) * 4}(x6)")
            vt.check_eq("x8", expected_out[r][c])
        line, log = build_and_run(vt, dmem_words=dmem, max_cycles=200000)
        if not line.startswith("[PASS]"):
            raise RuntimeError(f"{vt.name} FAILED: {line}\n{log[-2000:]}")

    ms, mv = TestBuilder("l3_imgfilter_scalar_measure"), TestBuilder("l3_imgfilter_vector_measure")
    ms.asm(scalar_asm)
    mv.asm(vector_asm)
    return (build_and_run_bench(ms, dmem, max_cycles=200000),
            build_and_run_bench(mv, dmem, max_cycles=200000))


def main():
    print("Level 3: matrix-vector multiply, matrix multiplication, 1D convolution, image filter\n")
    rows = []
    for name, fn in [("matvec", run_matvec), ("matmul", run_matmul),
                      ("conv1d", run_conv1d), ("imgfilter", run_imgfilter)]:
        s, v = fn()
        rows.append((name, s, v))
        speedup = s["cycles"] / v["cycles"]
        s_bpc = s["bytes_moved"] / s["cycles"]
        v_bpc = v["bytes_moved"] / v["cycles"]
        print(f"{name}: scalar={s['cycles']}cyc vector={v['cycles']}cyc "
              f"vec_instrs={v['vec_instrs']} speedup={speedup:.2f}x "
              f"| bandwidth scalar={s_bpc:.2f}B/cyc vector={v_bpc:.2f}B/cyc ratio={v_bpc / s_bpc:.2f}x")

    print("\n--- markdown table ---")
    print("| Benchmark | Scalar cycles | Vector cycles | Speedup | Scalar B/cycle | Vector B/cycle | BW ratio |")
    print("|---|---|---|---|---|---|---|")
    for name, s, v in rows:
        speedup = s["cycles"] / v["cycles"]
        s_bpc = s["bytes_moved"] / s["cycles"]
        v_bpc = v["bytes_moved"] / v["cycles"]
        print(f"| {name} | {s['cycles']} | {v['cycles']} | {speedup:.2f}x "
              f"| {s_bpc:.2f} | {v_bpc:.2f} | {v_bpc / s_bpc:.2f}x |")


if __name__ == "__main__":
    main()
