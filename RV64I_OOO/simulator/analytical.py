"""
Analytical (closed-form) performance model -- the fastest, least-detailed
rung of the modeling ladder (see simulator/bench_compare.py). No
simulation loop at all: a classic bottleneck/interval-analysis technique
-- estimate a resource (throughput) bound and a critical-path
(dependence-height) bound independently, then take their max, since
whichever is larger is what actually gates total execution time.

Deliberately excluded, and why: fixed front-end/back-end pipeline
latency (Phase 11's 1-cycle fetch latch, dispatch-to-commit drain,
epilogue ecall handling) is NOT modeled -- this shows up as a roughly
constant offset between this model's estimate and the real RTL-measured
cycle count for every benchmark, which is expected and worth reporting
(see bench_compare.py's gap analysis), not worth chasing with a
hand-tuned fudge constant that would make the model less honest about
what it actually captures.
"""
from dataclasses import dataclass

import isa


@dataclass
class AnalyticalResult:
    instr_count: int
    critical_path_cycles: int
    resource_bound_cycles: int
    binding_bound: str      # "critical-path" or "resource"
    cycles: int
    ipc: float


def critical_path_cycles(instrs):
    """Longest RAW-dependence-weighted path through the instruction DAG,
    in program order (all shared benchmarks are straight-line -- no
    branches/loops -- so program order is already a valid topological
    order; a future branchy workload would need this walked per basic
    block instead). x0 is excluded as a dependency source (always
    ready, hardwired zero -- same treatment lane0_src1_ready/
    lane0_src2_ready give it in riscv64_ooo_proc.v)."""
    last_writer_finish = {}  # reg -> finish cycle of its last writer
    finish = [0] * len(instrs)
    for i in instrs:
        ready_cycle = 0
        if i.rs1 != 0:
            ready_cycle = max(ready_cycle, last_writer_finish.get(i.rs1, 0))
        if i.rs2 != 0:
            ready_cycle = max(ready_cycle, last_writer_finish.get(i.rs2, 0))
        finish[i.idx] = ready_cycle + i.latency()
        if i.reg_write and i.rd != 0:
            last_writer_finish[i.rd] = finish[i.idx]
    return max(finish) if finish else 0


def resource_bound_cycles(instrs, width=3):
    """Per-functional-unit-class throughput bound: demand/rate, taking
    the actual RTL policy for `rate` (see this project's own RS-bank
    scope decisions) -- ALU issues up to `width`/cycle (Phase 16/17's
    default alu_rs.v 3-way issue path), MUL/LOAD/STORE stay 1/cycle
    regardless of width (mul_rs.v/lsq.v deliberately never widened --
    "mul is rare enough that a 2nd port isn't worth it", same for
    load/store throughput being bounded by l1_cache.v's single primary
    port), and DIV is non-pipelined -- each divide fully occupies the
    single divider for its whole latency (div_rs.v/div_fu.v's own
    documented single-outstanding, non-abortable scope), so N divides
    cost N*LAT_DIV cycles, not N/1."""
    n = len(instrs)
    n_alu = sum(1 for i in instrs if i.opcode_class in ("alu", "branch", "jal", "jalr", "ecall"))
    n_mul = sum(1 for i in instrs if i.is_mul)
    n_div = sum(1 for i in instrs if i.is_div)
    n_mem = sum(1 for i in instrs if i.is_load or i.is_store)

    bounds = {
        "dispatch/commit bandwidth": n / width if width else float("inf"),
        "alu_rs (3-way issue)": n_alu / width if width else float("inf"),
        "mul_rs (1-wide)": n_mul / 1,
        "div (non-pipelined)": n_div * isa.LAT_DIV,
        "lsq (1-wide)": n_mem / 1,
    }
    import math
    worst_name = max(bounds, key=bounds.get)
    return math.ceil(bounds[worst_name]), worst_name


def estimate(asm_text, width=3):
    instrs = isa.assemble(asm_text)
    n = len(instrs)
    cp = critical_path_cycles(instrs)
    rb, rb_name = resource_bound_cycles(instrs, width=width)
    cycles = max(cp, rb, 1)
    binding = "critical-path" if cp >= rb else f"resource ({rb_name})"
    return AnalyticalResult(
        instr_count=n,
        critical_path_cycles=cp,
        resource_bound_cycles=rb,
        binding_bound=binding,
        cycles=cycles,
        ipc=n / cycles if cycles else 0.0,
    )


if __name__ == "__main__":
    import workloads
    for b in workloads.all_benchmarks():
        r = estimate(b.source())
        print(f"{b.name:<32} cycles={r.cycles:<6} ipc={r.ipc:.3f}  "
              f"(cp={r.critical_path_cycles}, resource={r.resource_bound_cycles}, "
              f"binding={r.binding_bound})")
