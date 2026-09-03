"""
Load/store queue -- simplified mirror of RV64I_OOO/src/lsq.v. 1 issue/
cycle (matches lsq.v/l1_cache.v staying single-issue-per-cycle
regardless of dispatch width -- "lsq.v's load throughput is bounded by
l1_cache.v's single primary port regardless of issue width anyway").

Deliberately does NOT model store-to-load address disambiguation, the
store buffer, or cache hit/miss timing -- none of the shared benchmarks
(simulator/workloads.py) touch memory at all, so this bank sits
structurally present but unexercised, same documented-forward-looking-
scope treatment as branch_rs.py. A future memory-touching workload would
need this extended before its cycle counts could be trusted, not
silently mis-modeled by this stub.
"""
from dataclasses import dataclass


@dataclass
class Entry:
    tag: int
    instr: object
    s1_ready: bool  # base address (rs1) -- the only real dependency modeled
    s1_val: int
    s1_tag: int


class LSQ:
    def __init__(self, depth):
        self.depth = depth
        self.entries = []

    def has_free(self, n=1):
        return (self.depth - len(self.entries)) >= n

    def alloc(self, tag, instr, s1_ready, s1_val, s1_tag):
        self.entries.append(Entry(tag, instr, s1_ready, s1_val, s1_tag))

    def snoop(self, broadcasts):
        for e in self.entries:
            for tag, val in broadcasts:
                if not e.s1_ready and e.s1_tag == tag:
                    e.s1_ready, e.s1_val = True, val

    def ready_entries(self, rob_head_tag):
        """Peek only -- see alu_rs.py's identical method for why."""
        ready = [e for e in self.entries if e.s1_ready]
        if not ready:
            return []
        ready.sort(key=lambda e: (e.tag != rob_head_tag, e.tag))
        return ready[:1]

    def finalize(self, entry):
        self.entries.remove(entry)
        return entry.tag, entry.s1_val  # placeholder "load result" -- see header
