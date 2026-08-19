"""
Dual-core test driver: builds and runs tb_dual_core.v against
dual_core_riscv64_ooo.v -- two real riscv64_ooo_proc cores sharing one
l2_cache.v. The point of this suite specifically (as opposed to
build_tests_ooo.py's single-core suite, or tb_cache_mesi.v's isolated
protocol-only test) is proving real cross-core MESI coherency through
the *full* pipeline: a value one core's store commits must become
visible to a load on the *other* core only via an actual coherency
transaction, not by construction.

Reuses RV64I/verify/asm64.py and this project's TestBuilder convention,
same as build_tests_ooo.py.
"""
import os
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

from build_tests_ooo import TestBuilder, REUSED_RTL, OOO_RTL, _ensure_idle_thread_mem  # noqa: E402

DUAL_RTL = OOO_RTL + ["dual_core_riscv64_ooo.v"]


def _write_mem(name, tb):
    path = os.path.join(GEN, f"{name}.mem")
    asm.write_imem_halfwords(path, asm.assemble_to_mem(tb.source()))
    return f"{name}.mem"


def run_dual_core_test(name, c0t0, c1t0, max_cycles=20000, c0t1=None, c1t1=None):
    """c0t0/c1t0 are required TestBuilders (the real work); c0t1/c1t1
    default to the idle program, matching build_tests_ooo.py's single-
    core convention of not needing every thread occupied to test what
    matters."""
    _ensure_idle_thread_mem()
    c0t0_mem = _write_mem(f"{name}_c0t0", c0t0)
    c1t0_mem = _write_mem(f"{name}_c1t0", c1t0)
    c0t1_mem = _write_mem(f"{name}_c0t1", c0t1) if c0t1 else "idle_thread.mem"
    c1t1_mem = _write_mem(f"{name}_c1t1", c1t1) if c1t1 else "idle_thread.mem"

    dmem_path = os.path.join(GEN, f"{name}_data.mem")
    asm.write_mem(dmem_path, [0] * 64)

    wrapper_path = os.path.join(GEN, f"tb_{name}.v")
    with open(wrapper_path, "w") as f:
        f.write(f"""`timescale 1ns/1ps
module tb_{name};
    tb_dual_core #(
        .C0_IMEM_FILE0("{c0t0_mem}"), .C0_IMEM_FILE1("{c0t1_mem}"),
        .C1_IMEM_FILE0("{c1t0_mem}"), .C1_IMEM_FILE1("{c1t1_mem}"),
        .DMEM_FILE("{name}_data.mem"),
        .TEST_NAME("{name}"),
        .MAX_CYCLES({max_cycles})
    ) tb();
endmodule
""")

    vvp_path = os.path.join(GEN, f"{name}.vvp")
    compile_cmd = (
        ["iverilog", "-g2012", "-o", vvp_path,
         os.path.join(THIS_DIR, "tb_dual_core.v"), wrapper_path]
        + [os.path.join(OOO_SRC, f) for f in DUAL_RTL]
        + [os.path.join(RV64I_SRC, f) for f in REUSED_RTL]
    )
    r = subprocess.run(compile_cmd, cwd=GEN, capture_output=True, text=True)
    if r.returncode != 0:
        return f"[COMPILE ERROR] {name}\n{r.stdout}\n{r.stderr}"

    r2 = subprocess.run(["vvp", vvp_path], cwd=GEN, capture_output=True, text=True)
    out = r2.stdout + r2.stderr
    lines = {}
    for line in out.splitlines():
        for tag in ("C0T0", "C0T1", "C1T0", "C1T1"):
            if line.startswith(f"[PASS-{tag}]") or line.startswith(f"[FAIL-{tag}]") or line.startswith(f"[TIMEOUT-{tag}]"):
                lines[tag] = line
    all_pass = all(lines.get(t, "").startswith(f"[PASS-{t}]") for t in ("C0T0", "C1T0"))
    if all_pass:
        return f"[PASS] {name}: {lines.get('C0T0')} | {lines.get('C1T0')}"
    return f"[FAIL] {name}: {lines}\n{out[-3000:]}"


def t_producer_consumer():
    # Core 0 writes a data word, then a flag word (both stores go through
    # its own private L1, then commit via the store buffer -- see
    # lsq.v/l1_cache.v's headers). Core 1 busy-polls the flag through its
    # *own*, entirely separate L1 until it observes the write -- which can
    # only happen if core 1's load-miss on the flag address genuinely
    # triggers a coherency snoop of core 0's L1 (or a correctly-updated
    # L2, once core 0's store has drained and evicted/updated there) and
    # keeps re-polling (a fresh L1 miss/re-snoop each loop iteration,
    # since a cached copy would otherwise never observe core 0's later
    # write) until the real MESI transaction actually delivers the new
    # value. Finally checks it reads the *data* value core 0 actually
    # wrote, not stale/zero memory -- the real end-to-end proof that a
    # value written by one core's commit becomes visible on the other
    # core only via a genuine cross-core coherency transaction.
    c0 = TestBuilder("dualcore_c0_producer")
    c0.asm("""
li x5, 0x2000
li x6, 0xCAFE
sd x6, 0(x5)
li x7, 0x2008
li x8, 1
sd x8, 0(x7)
""")
    # No check_eq needed -- c0's own job is just to produce; correctness
    # is verified from c1's side.

    c1 = TestBuilder("dualcore_c1_consumer")
    c1.asm("""
li x7, 0x2008
li x10, 1
poll:
ld x9, 0(x7)
bne x9, x10, poll
li x5, 0x2000
ld x6, 0(x5)
""")
    c1.check_eq("x6", "0xCAFE")

    return run_dual_core_test("dualcore_producer_consumer", c0, c1, max_cycles=5000)


def main():
    results = []
    for fn in [t_producer_consumer]:
        line = fn()
        print(line)
        results.append(line)
    passed = sum(1 for r in results if r.startswith("[PASS]"))
    print(f"\n{passed}/{len(results)} passed")


if __name__ == "__main__":
    main()
