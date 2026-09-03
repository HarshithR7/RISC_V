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

SIMULATOR_DIR = os.path.join(THIS_DIR, "..", "simulator")
sys.path.insert(0, SIMULATOR_DIR)
# Phase: performance-modeling ladder -- the benchmark bodies themselves
# now live in simulator/workloads.py, shared with the analytical/trace-
# driven/cycle-level models (simulator/bench_compare.py) so "the same
# benchmark flows through every level" is a literal fact, not four
# independently-authored copies that could quietly drift apart.
from workloads import Bench, make_serial_chain, make_independent_ops, make_reduction_unrolled4, \
    make_divide_overlap, make_mixed_bank_pairs, make_divide_after_independent  # noqa: E402

REUSED_RTL = ["program_counter.v", "instruction_fetch.v", "register_file.v", "data_memory.v",
              "vector_register_file.v", "vector_alu.v"]
OOO_RTL = ["decode_ooo.v", "rat.v", "vec_rat.v", "rob.v", "alu_rs.v", "branch_rs.v", "mul_rs.v", "div_rs.v",
           "div_fu.v", "lsq.v", "bht.v", "ras.v", "vec_rs.v", "l1_cache.v", "l2_cache.v",
           "ecc64.v", "ecc_line.v", "ecc_register_file.v", "instruction_fetch_reg.v",
           "instruction_fetch_axi.v", "data_memory_axi.v", "icache.v", "btb.v",
           "riscv64_ooo_proc.v", "riscv64_ooo_proc_solo.v"]


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
