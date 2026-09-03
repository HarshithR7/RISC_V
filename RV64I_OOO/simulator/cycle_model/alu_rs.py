"""
ALU reservation-station bank -- mirrors RV64I_OOO/src/alu_rs.v. Up to
`width` issues per cycle (Phase 16's 3-way issue path), oldest-ready-
first. Also the bank for branch/jal/jalr/ecall-class instructions in
this simplified model (see branch_rs.py for the *prediction* side --
resolution timing itself is 1 cycle either way, so folding it into this
bank's uniform "1-cycle combinational op" treatment is a deliberate
simplification, not a fidelity gap for the straight-line benchmarks this
model currently runs).

Deliberately not a generic, templated reservation-station class shared
across banks -- same "concrete, purpose-built logic over a maximally
generic res_station_bank" instinct alu_rs.v's own header states for the
RTL; mul_rs.py/div_rs.py/lsq.py each stay their own small, explicit
class rather than subclassing a shared abstraction.
"""
from dataclasses import dataclass


@dataclass
class Entry:
    tag: int             # this entry's own dest ROB tag
    instr: object
    s1_ready: bool
    s1_val: int
    s1_tag: int
    s2_ready: bool
    s2_val: int
    s2_tag: int


class ALURS:
    def __init__(self, depth, width):
        self.depth = depth
        self.width = width
        self.entries = []

    def has_free(self, n):
        return (self.depth - len(self.entries)) >= n

    def alloc(self, tag, instr, s1_ready, s1_val, s1_tag, s2_ready, s2_val, s2_tag):
        self.entries.append(Entry(tag, instr, s1_ready, s1_val, s1_tag,
                                   s2_ready, s2_val, s2_tag))

    def snoop(self, broadcasts):
        """broadcasts: list of (tag, value) results won on the CDB this
        cycle -- wakes up any waiting entry whose s1_tag/s2_tag matches."""
        for e in self.entries:
            for tag, val in broadcasts:
                if not e.s1_ready and e.s1_tag == tag:
                    e.s1_ready, e.s1_val = True, val
                if not e.s2_ready and e.s2_tag == tag:
                    e.s2_ready, e.s2_val = True, val

    def ready_entries(self, rob_head_tag):
        """Peek only (see cdb.py's header for why issue is split into
        peek+finalize): up to `width` ready entries this bank could
        present to the shared CDB this cycle, oldest-first -- the real
        arbitration for the shared, width-wide CDB happens centrally in
        cdb.py, since alu_rs.v's own 3 candidates still compete against
        mul/div/lsq/branch for the *same* pool of broadcast slots in the
        real RTL, not 3 dedicated slots of their own."""
        ready = [e for e in self.entries if e.s1_ready and e.s2_ready]
        ready.sort(key=lambda e: (e.tag != rob_head_tag, e.tag))
        return ready[: self.width]

    def finalize(self, entry):
        """Commits one entry cdb.py's arbiter actually granted this
        cycle: removes it from the bank and computes its result (1-cycle
        combinational ALU, matching alu_rs.v's real timing)."""
        self.entries.remove(entry)
        return entry.tag, _alu_result(entry.instr, entry.s1_val, entry.s2_val)


def _alu_result(instr, v1, v2):
    """Timing-only model: the exact ALU op doesn't matter for any of
    this ladder's cycle-count estimates, only that a value exists to
    propagate -- so this is a placeholder arithmetic op, not a faithful
    ALU (unlike the RTL's own alu_exec, which every RTL test suite
    already verifies bit-for-bit)."""
    return (v1 + v2) & 0xFFFFFFFFFFFFFFFF
