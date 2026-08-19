"""
Lockstep DMR test driver: builds and runs tb_lockstep.v against
lockstep_dual_core.v -- two fully independent riscv64_ooo_proc_solo
copies running the identical program, plus a continuous PC/halt
comparator (lockstep_fault). Two tests: a baseline (both copies correct,
lockstep_fault must stay 0 -- no false positives) and a fault-injection
run (tb_lockstep.v's FAULT_INJECT_CYCLE forces a one-cycle PC corruption
into core B via a hierarchical force/release, lockstep_fault must fire).

Reuses build_tests_ooo.py's TestBuilder/RTL-list conventions.
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

LOCKSTEP_RTL = OOO_RTL + ["lockstep_dual_core.v"]


def run_lockstep_test(name, tb, fault_inject_cycle=0, max_cycles=20000):
    _ensure_idle_thread_mem()
    imem_path = os.path.join(GEN, f"{name}.mem")
    asm.write_imem_halfwords(imem_path, asm.assemble_to_mem(tb.source()))
    dmem_path = os.path.join(GEN, f"{name}_data.mem")
    asm.write_mem(dmem_path, [0] * 64)

    wrapper_path = os.path.join(GEN, f"tb_{name}.v")
    with open(wrapper_path, "w") as f:
        f.write(f"""`timescale 1ns/1ps
module tb_{name};
    tb_lockstep #(
        .IMEM_FILE0("{name}.mem"), .IMEM_FILE1("idle_thread.mem"),
        .DMEM_FILE("{name}_data.mem"),
        .TEST_NAME("{name}"),
        .MAX_CYCLES({max_cycles}),
        .FAULT_INJECT_CYCLE({fault_inject_cycle})
    ) tb();
endmodule
""")

    vvp_path = os.path.join(GEN, f"{name}.vvp")
    compile_cmd = (
        ["iverilog", "-g2012", "-o", vvp_path,
         os.path.join(THIS_DIR, "tb_lockstep.v"), wrapper_path]
        + [os.path.join(OOO_SRC, f) for f in LOCKSTEP_RTL]
        + [os.path.join(RV64I_SRC, f) for f in REUSED_RTL]
    )
    r = subprocess.run(compile_cmd, cwd=GEN, capture_output=True, text=True)
    if r.returncode != 0:
        err = f"[COMPILE ERROR] {name}\n{r.stdout}\n{r.stderr}"
        return err, "", err

    r2 = subprocess.run(["vvp", vvp_path], cwd=GEN, capture_output=True, text=True)
    out = r2.stdout + r2.stderr
    t0_line = ""
    lockstep_line = ""
    for line in out.splitlines():
        if line.startswith("[PASS-T0]") or line.startswith("[FAIL-T0]") or line.startswith("[TIMEOUT-T0]"):
            t0_line = line
        if line.startswith("[LOCKSTEP]"):
            lockstep_line = line
    return t0_line, lockstep_line, out


def t_lockstep_baseline():
    # No fault injected: both cores must agree throughout a normal
    # program, and lockstep_fault must never fire -- the "redundancy
    # doesn't cry wolf" check. Without this, a fault-detection feature
    # that spuriously fires on *every* run would be worthless (or worse,
    # actively harmful in a real safety system), and only ever testing
    # the fault-injection case wouldn't catch that.
    t = TestBuilder("lockstep_baseline")
    t.asm("""
li x5, 0
li x6, 1
loop:
add x5, x5, x6
addi x6, x6, 1
li x7, 6
bne x6, x7, loop
""")
    t.check_eq("x5", 15)
    t0_line, lockstep_line, out = run_lockstep_test("lockstep_baseline", t)
    ok = t0_line.startswith("[PASS-T0]") and "fault=0" in lockstep_line
    if ok:
        return f"[PASS] lockstep_baseline: {t0_line} | {lockstep_line}"
    return f"[FAIL] lockstep_baseline: {t0_line} | {lockstep_line}\n{out[-3000:]}"


def t_lockstep_fault_detected():
    # A longer-running version of the loop (49 iterations instead of 5)
    # so that FAULT_INJECT_CYCLE below lands solidly inside the loop's
    # live execution window instead of landing after the (out-of-order-
    # fast) short loop has already retired. x7 (the loop bound, see
    # tb_lockstep.v's header for why x7 and not the loop counter or
    # accumulator) is set once, outside the loop, before the injection
    # cycle: cycle 60 was picked (via cycle-by-cycle tracing) as
    # comfortably after `li x7, 50` itself has actually committed --
    # injecting any earlier just corrupts x7's stale reset-time value of
    # 0, which `li x7, 50`'s own real commit then unconditionally
    # overwrites moments later, silently erasing the injected fault
    # before any consumer ever observes it. Core B's own x7 gets a
    # one-time hierarchical bit-flip at that cycle -- a simulated
    # soft-error single-event upset in one of the two redundant copies.
    # lockstep_fault must latch and stay set. Deliberately doesn't also
    # require core A's own PASS-T0 check to still pass -- the point of
    # this test is proving the *fault is detected*, not that core A
    # somehow "vetoes" core B's corruption (it structurally can't;
    # that's core B's own local internal state).
    t = TestBuilder("lockstep_fault")
    t.asm("""
li x5, 0
li x6, 1
li x7, 50
loop:
add x5, x5, x6
addi x6, x6, 1
bne x6, x7, loop
""")
    t.check_eq("x5", 1225)
    t0_line, lockstep_line, out = run_lockstep_test("lockstep_fault", t, fault_inject_cycle=60)
    ok = "fault=1" in lockstep_line
    if ok:
        return f"[PASS] lockstep_fault_detected: {lockstep_line}"
    return f"[FAIL] lockstep_fault_detected: {t0_line} | {lockstep_line}\n{out[-3000:]}"


def main():
    results = []
    for fn in [t_lockstep_baseline, t_lockstep_fault_detected]:
        line = fn()
        print(line)
        results.append(line)
    passed = sum(1 for r in results if r.startswith("[PASS]"))
    print(f"\n{passed}/{len(results)} passed")


if __name__ == "__main__":
    main()
