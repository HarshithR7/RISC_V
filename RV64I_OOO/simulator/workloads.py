"""
Shared benchmark bodies used across every level of the performance-
modeling ladder (analytical, trace-driven, cycle-level -- see
bench_compare.py) *and* the real RTL flow (verify/bench_ooo.py, which
imports from here instead of defining these locally). One definition,
reused everywhere, is what makes "the same benchmark flows through all
four levels" a literal fact rather than an aspiration -- see
RV64I_OOO/README.md's Phase 4 section for the original benchmarks'
own design rationale (reproduced here only where it explains *why* a
benchmark is shaped the way it is, not duplicated in full).
"""
from dataclasses import dataclass


@dataclass
class Bench:
    name: str
    body: str
    expected_x5: int

    def source(self):
        """Same convention verify/bench_ooo.py's own Bench.source() used:
        x5 holds the benchmark's own result; check it, then the usual
        x31=PASS_CODE/ecall convention."""
        lines = [self.body]
        lines.append(f"li x30, {self.expected_x5}")
        lines.append("bne x5, x30, fail1")
        lines.append("li x31, 0xFFFF0000")
        lines.append("ecall")
        lines.append("fail1:")
        lines.append("li x31, 1")
        lines.append("ecall")
        return "\n".join(lines)


def repeat_independent(instr, n):
    return "\n".join([instr] * n)


def repeat_chain(base_reg, val_reg, n):
    return "\n".join([f"add {base_reg}, {base_reg}, {val_reg}"] * n)


def make_serial_chain(n=16):
    body = f"""
li x5, 0
li x6, 1
{repeat_chain('x5', 'x6', n)}
"""
    return Bench("bench_serial_chain", body, n)


def make_independent_ops(n=16):
    # Every iteration is `add x5, x1, x2` -- no data dependency between
    # any of the n instructions (only on x1/x2, both ready from the
    # start), so this is purely independent, dispatch-bound work.
    body = f"""
li x1, 1
li x2, 2
{repeat_independent('add x5, x1, x2', n)}
"""
    return Bench("bench_independent_ops", body, 3)


def make_reduction_unrolled4(n_per_acc=4):
    # Same total amount of add work as make_serial_chain(n=4*n_per_acc),
    # but split into 4 independent accumulators (a real loop-unrolling
    # technique) so the dependency-chain depth is n_per_acc, not
    # 4*n_per_acc -- the actual mechanism by which unrolling increases
    # exploitable ILP, combined at the end.
    total = 4 * n_per_acc
    body_lines = ["li x1, 1", "li x10, 0", "li x11, 0", "li x12, 0", "li x13, 0"]
    for _ in range(n_per_acc):
        body_lines += ["add x10, x10, x1", "add x11, x11, x1", "add x12, x12, x1", "add x13, x13, x1"]
    body_lines += ["add x10, x10, x11", "add x12, x12, x13", "add x10, x10, x12", "add x5, x10, x0"]
    return Bench("bench_reduction_unrolled4", "\n".join(body_lines), total)


def make_divide_overlap(n_independent=8):
    # Independent short ops *after* the divide: single-issue OoO already
    # dispatches these fast enough to execute during the divide's ~64-cycle
    # latency regardless of dispatch width (this was already the whole
    # point of the Phase 1 div_overlap test) -- measured to show exactly
    # 1.00x here, i.e. no *additional* benefit from wider dispatch on top
    # of what out-of-order execution alone already provided.
    body_lines = ["li x1, 3", "li x2, 4", "li x20, 100", "li x21, 7", "div x5, x20, x21"]
    for i in range(n_independent):
        body_lines.append(f"add x{10+i}, x1, x2")
    return Bench("bench_divide_overlap", "\n".join(body_lines), 14)


def make_mixed_bank_pairs(n_pairs=3):
    # Alternates ALU and multiply ops -- two genuinely separate
    # reservation-station banks -- rather than piling everything onto
    # alu_rs alone. alu_rs/mul_rs each only need one slot every *other*
    # cycle this way, instead of contending for the same bank's single
    # 1-per-cycle CDB drain rate every cycle -- the one pattern where
    # dual-issue's dispatch-bandwidth advantage isn't immediately
    # absorbed by a single saturated bank.
    body_lines = ["li x1, 3", "li x2, 4"]
    for i in range(n_pairs):
        body_lines.append("add x10, x1, x2")
        body_lines.append("mul x20, x1, x2")
    body_lines.append("add x5, x10, x20")
    return Bench("bench_mixed_bank_pairs", "\n".join(body_lines), 19)


def make_divide_after_independent(n_independent=2):
    # The complementary case, and the one that actually exercises
    # dispatch width: independent short ops placed *before* the divide.
    # Since commit stays exactly 1-wide (see riscv64_ooo_proc.v's
    # header), dispatch can never outrun commit for long -- the only way
    # dispatch width changes *total* wall-clock time is by changing when
    # a program-order-later long-latency instruction gets to *start*.
    # Kept deliberately small (n_independent=2, not larger): alu_rs itself
    # is only 4 entries deep and drains at the same 1-per-cycle CDB rate
    # as the ROB commits, so it saturates almost immediately too -- a
    # *larger* independent-ops count re-triggers the same "shared
    # single-drain-rate resource" ceiling this benchmark exists to get
    # underneath, this time at the reservation-station level instead of
    # the ROB level. With just 2, both fit in one dual-issue dispatch
    # cycle instead of two single-issue ones, starting the divide's
    # ~64-cycle countdown one cycle sooner -- small, but real and
    # correctly attributable to dispatch width alone.
    body_lines = ["li x1, 3", "li x2, 4"]
    for i in range(n_independent):
        body_lines.append(f"add x{10+i}, x1, x2")
    body_lines += ["li x20, 100", "li x21, 7", "div x5, x20, x21"]
    return Bench("bench_divide_after_independent", "\n".join(body_lines), 14)


def all_benchmarks():
    """The shared set every level of the modeling ladder runs against."""
    return [
        make_serial_chain(16),
        make_independent_ops(16),
        make_reduction_unrolled4(4),
        make_divide_overlap(8),
        make_divide_after_independent(2),
        make_mixed_bank_pairs(3),
    ]
