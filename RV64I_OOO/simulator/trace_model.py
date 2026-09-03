"""
Trace-driven performance model -- one rung more detailed than the
analytical model, far cheaper than the cycle-level model (see
simulator/bench_compare.py). A greedy, resource-constrained list
scheduler (ASAP scheduling): each simulated cycle, admits up to `width`
oldest-ready instructions, subject to per-functional-unit-class resource
counts, without modeling RAT rename bookkeeping, the ROB's circular-
buffer mechanics, or CDB broadcast/snoop timing explicitly -- those are
what the cycle-level model adds.

Oldest-first admission within a cycle mirrors the RTL's own age-based
arbitration convention (alu_rs.v's issue pick, the top-level CDB
arbiter's "find oldest ready, mask, repeat" reduction) -- ties are
broken by program order here for the same reason they are in the RTL:
an older instruction should never be starved by a younger one that
happens to also be ready.
"""
from dataclasses import dataclass

import isa


@dataclass
class TraceResult:
    instr_count: int
    cycles: int
    ipc: float
    issue_cycle: list       # per-instruction (1-indexed) cycle it issued
    complete_cycle: list    # per-instruction cycle its result is ready


def _resource_class(instr):
    if instr.is_mul:
        return "mul"
    if instr.is_div:
        return "div"
    if instr.is_load or instr.is_store:
        return "mem"
    return "alu"  # alu, branch, jal, jalr, ecall -- all go through alu_rs.v's bank


def simulate(asm_text, width=3, alu_ports=None, mul_ports=1, div_ports=1, mem_ports=1):
    """alu_ports defaults to `width` (alu_rs.v's issue rate scales with
    dispatch width since Phase 16); mul/div/mem stay fixed regardless of
    width, matching those banks' own documented single-issue scope."""
    if alu_ports is None:
        alu_ports = width

    instrs = isa.assemble(asm_text)
    n = len(instrs)
    issue_cycle = [None] * n
    complete_cycle = [None] * n

    # Producer index per instruction (None = no real register dependency,
    # e.g. x0 or a not-yet-written register) -- resolved once, up front,
    # via program-order last-writer tracking. Actual readiness is then
    # `complete_cycle[producer]`, evaluated dynamically during scheduling
    # below, since a producer's real completion cycle depends on
    # resource contention, not just its position in program order.
    producer1 = [None] * n
    producer2 = [None] * n
    last_writer = {}                # reg -> instr idx of its last writer
    for i in instrs:
        if i.rs1 != 0 and i.rs1 in last_writer:
            producer1[i.idx] = last_writer[i.rs1]
        if i.rs2 != 0 and i.rs2 in last_writer:
            producer2[i.idx] = last_writer[i.rs2]
        if i.reg_write and i.rd != 0:
            last_writer[i.rd] = i.idx

    def ready_cycle(idx):
        rc = 0
        p1, p2 = producer1[idx], producer2[idx]
        if p1 is not None:
            if complete_cycle[p1] is None:
                return None  # producer not scheduled yet -- not ready
            rc = max(rc, complete_cycle[p1])
        if p2 is not None:
            if complete_cycle[p2] is None:
                return None
            rc = max(rc, complete_cycle[p2])
        return rc

    remaining = list(range(n))  # program-order indices still unscheduled
    cycle = 1
    ports = {"alu": alu_ports, "mul": mul_ports, "div": div_ports, "mem": mem_ports}
    # A class's port becomes free again at a specific future cycle once
    # occupied by a multi-cycle op (div) -- tracked as a list of
    # "cycle this unit is free again" per class, sized to that class's
    # port count.
    free_at = {cls: [0] * count for cls, count in ports.items()}

    scheduled = 0
    guard = 0
    while scheduled < n:
        guard += 1
        if guard > 200000:
            raise RuntimeError("trace_model: scheduling did not converge")
        admitted_this_cycle = 0
        for idx in list(remaining):
            if admitted_this_cycle >= width:
                break
            instr = instrs[idx]
            rc = ready_cycle(idx)
            if rc is None or rc > cycle:
                continue
            cls = _resource_class(instr)
            free_list = free_at[cls]
            free_port = next((p for p, free_cycle in enumerate(free_list) if free_cycle <= cycle), None)
            if free_port is None:
                continue
            # Admit: oldest-ready-first is already guaranteed by walking
            # `remaining` in ascending program-order index.
            issue_cycle[idx] = cycle
            lat = instr.latency()
            complete_cycle[idx] = cycle + lat
            free_list[free_port] = cycle + lat if cls == "div" else cycle + 1
            remaining.remove(idx)
            admitted_this_cycle += 1
            scheduled += 1
        cycle += 1

    total_cycles = max(complete_cycle) if complete_cycle else 0
    return TraceResult(
        instr_count=n,
        cycles=total_cycles,
        ipc=n / total_cycles if total_cycles else 0.0,
        issue_cycle=issue_cycle,
        complete_cycle=complete_cycle,
    )


if __name__ == "__main__":
    import workloads
    for b in workloads.all_benchmarks():
        r = simulate(b.source())
        print(f"{b.name:<32} cycles={r.cycles:<6} ipc={r.ipc:.3f}")
