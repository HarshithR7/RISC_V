"""
Runs the shared benchmark set (simulator/workloads.py) through every
level of the performance-modeling ladder -- analytical, trace-driven,
cycle-level, and (optionally) the real RTL via verify/bench_ooo.py's
existing Icarus Verilog flow -- and prints a comparison table plus the
gap between consecutive levels, matching the methodology this whole
ladder exists to support: each level should be validated against the
one below it, and a large, unexplained gap is a signal to go look, not
something to silently smooth over.

FPGA is not available in this environment (no physical PYNQ hardware
attached to this session) -- printed as such, never a fabricated number.

Usage: `python bench_compare.py` (RTL row included, requires iverilog/
vvp on PATH -- this is slow, one Icarus compile+sim per benchmark) or
`python bench_compare.py --no-rtl` (skips the RTL row, fast).
"""
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)
VERIFY_DIR = os.path.join(THIS_DIR, "..", "verify")
sys.path.insert(0, VERIFY_DIR)

import analytical
import trace_model
import workloads
from cycle_model import cpu as cycle_cpu


def run_rtl(bench):
    import bench_ooo
    cycles, line = bench_ooo.run_once(bench, enable_dual_issue=1)
    ok = line.startswith("[PASS]")
    return cycles if ok else None


def main():
    include_rtl = "--no-rtl" not in sys.argv

    header = f"{'benchmark':<32} {'analytical':>11} {'trace':>8} {'cycle':>8}"
    if include_rtl:
        header += f" {'RTL':>8}"
    header += f" {'FPGA':>28}"
    print(header)

    rows = []
    for b in workloads.all_benchmarks():
        src = b.source()
        a = analytical.estimate(src)
        t = trace_model.simulate(src)
        c = cycle_cpu.simulate(src)
        rtl_cycles = run_rtl(b) if include_rtl else None
        rows.append((b.name, a.cycles, t.cycles, c.cycles, rtl_cycles))

        line = f"{b.name:<32} {a.cycles:>11} {t.cycles:>8} {c.cycles:>8}"
        if include_rtl:
            line += f" {rtl_cycles if rtl_cycles is not None else 'FAIL':>8}"
        line += f" {'not available (no PYNQ hw)':>28}"
        print(line)

    print()
    print("Gap analysis (cycles added at each successively more-detailed level):")
    for name, a_cyc, t_cyc, c_cyc, rtl_cyc in rows:
        parts = [f"analytical={a_cyc}", f"trace(+{t_cyc - a_cyc})={t_cyc}", f"cycle(+{c_cyc - t_cyc})={c_cyc}"]
        if include_rtl and rtl_cyc is not None:
            parts.append(f"RTL(+{rtl_cyc - c_cyc})={rtl_cyc}")
        print(f"  {name:<32} " + "  ->  ".join(parts))

    if include_rtl:
        print()
        print("Note: the cycle-level model does not yet model icache miss latency")
        print("(Phase 17) or front-end pipeline fill/drain (Phase 11's registered")
        print("fetch latch) -- these are real, expected, and currently the main")
        print("attributable sources of the cycle-model-to-RTL gap above, not bugs.")


if __name__ == "__main__":
    main()
