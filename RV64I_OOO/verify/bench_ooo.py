"""
Phase 4 benchmark suite: measures real cycle counts for the same hand-
assembled RV64I+M programs run on the *same* RTL twice -- once with
ENABLE_DUAL_ISSUE=1 (Phase 3's 2-wide dispatch), once with =0 (forces
lane 1 to never fire, i.e. Phase 1/2's single-issue behavior) -- everything
else (OoO execution, branch prediction/speculation, ROB/RS sizing)
identical between the two runs. This is a direct, apples-to-apples
dispatch-width comparison, not an estimate.

Reuses RV64I/verify/asm64.py unmodified, same convention as
build_tests_ooo.py.
"""
import os
import re
import subprocess
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
RV64I_VERIFY = os.path.join(THIS_DIR, "..", "..", "RV64I", "verify")
RV64I_SRC = os.path.join(THIS_DIR, "..", "..", "RV64I", "src")
OOO_SRC = os.path.join(THIS_DIR, "..", "src")
GEN = os.path.join(THIS_DIR, "generated")
os.makedirs(GEN, exist_ok=True)

sys.path.insert(0, RV64I_VERIFY)
import asm64 as asm  # noqa: E402

REUSED_RTL = ["program_counter.v", "instruction_fetch.v", "register_file.v", "data_memory.v",
              "vector_register_file.v", "vector_alu.v"]
OOO_RTL = ["decode_ooo.v", "rat.v", "vec_rat.v", "rob.v", "alu_rs.v", "branch_rs.v", "mul_rs.v", "div_rs.v",
           "div_fu.v", "lsq.v", "bht.v", "vec_rs.v", "l1_cache.v", "l2_cache.v",
           "riscv64_ooo_proc.v", "riscv64_ooo_proc_solo.v"]


class Bench:
    def __init__(self, name, body, expected_x5):
        self.name = name
        self.body = body
        self.expected_x5 = expected_x5

    def source(self):
        # x5 holds the benchmark's own result; check it, then the usual
        # x31=PASS_CODE/ecall convention.
        lines = [self.body]
        lines.append(f"li x30, {self.expected_x5}")
        lines.append("bne x5, x30, fail1")
        lines.append("li x31, 0xFFFF0000")
        lines.append("ecall")
        lines.append("fail1:")
        lines.append("li x31, 1")
        lines.append("ecall")
        return "\n".join(lines)


IDLE_THREAD_MEM = os.path.join(GEN, "idle_thread.mem")


def _ensure_idle_thread_mem():
    """Phase 7 (SMT): the core always runs two threads -- see
    build_tests_ooo.py's identical helper for why thread 1 needs a real,
    always-passing idle program by default."""
    if not os.path.exists(IDLE_THREAD_MEM):
        items = asm.assemble_to_mem("li x31, 0xFFFF0000\necall")
        asm.write_imem_halfwords(IDLE_THREAD_MEM, items)


def run_once(bench, enable_dual_issue, max_cycles=50000):
    _ensure_idle_thread_mem()
    name = f"{bench.name}_d{enable_dual_issue}"
    src = bench.source()
    items = asm.assemble_to_mem(src)
    imem_path = os.path.join(GEN, f"{name}.mem")
    asm.write_imem_halfwords(imem_path, items)
    dmem_path = os.path.join(GEN, f"{name}_data.mem")
    asm.write_mem(dmem_path, [0] * 64)

    wrapper_path = os.path.join(GEN, f"tb_{name}.v")
    with open(wrapper_path, "w") as f:
        f.write(f"""`timescale 1ns/1ps
module tb_{name};
    tb_bench_ooo #(
        .IMEM_FILE0("{name}.mem"),
        .IMEM_FILE1("idle_thread.mem"),
        .DMEM_FILE("{name}_data.mem"),
        .TEST_NAME("{name}"),
        .ENABLE_DUAL_ISSUE({enable_dual_issue}),
        .MAX_CYCLES({max_cycles})
    ) core();
endmodule
""")

    vvp_path = os.path.join(GEN, f"{name}.vvp")
    compile_cmd = (
        ["iverilog", "-g2012", "-o", vvp_path,
         os.path.join(THIS_DIR, "tb_bench_ooo.v"), wrapper_path]
        + [os.path.join(OOO_SRC, f) for f in OOO_RTL]
        + [os.path.join(RV64I_SRC, f) for f in REUSED_RTL]
    )
    r = subprocess.run(compile_cmd, cwd=GEN, capture_output=True, text=True)
    if r.returncode != 0:
        return None, f"[COMPILE ERROR] {name}\n{r.stdout}\n{r.stderr}"

    r2 = subprocess.run(["vvp", vvp_path], cwd=GEN, capture_output=True, text=True)
    out = r2.stdout + r2.stderr
    result_line = ""
    for line in out.splitlines():
        if line.startswith("[PASS]") or line.startswith("[FAIL]") or line.startswith("[TIMEOUT]"):
            result_line = line
    m = re.search(r"(\d+) cycles", result_line)
    cycles = int(m.group(1)) if m else None
    return cycles, result_line


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


def main():
    benches = [
        make_serial_chain(16),
        make_independent_ops(16),
        make_reduction_unrolled4(4),
        make_divide_overlap(8),
        make_divide_after_independent(2),
        make_mixed_bank_pairs(3),
    ]

    print(f"{'benchmark':<28} {'dual-issue':>12} {'single-issue':>14} {'speedup':>10}")
    for b in benches:
        c1, line1 = run_once(b, 1)
        c0, line0 = run_once(b, 0)
        ok1 = line1.startswith("[PASS]")
        ok0 = line0.startswith("[PASS]")
        if not (ok1 and ok0):
            print(f"{b.name:<28} FAILED: dual={line1} single={line0}")
            continue
        speedup = c0 / c1 if c1 else float("nan")
        print(f"{b.name:<28} {c1:>12} {c0:>14} {speedup:>9.2f}x")


if __name__ == "__main__":
    main()
