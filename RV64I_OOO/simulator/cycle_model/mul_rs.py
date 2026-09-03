"""
Multiply reservation-station bank -- mirrors RV64I_OOO/src/mul_rs.v.
Same shape as alu_rs.py but deliberately capped at 1 issue/cycle
regardless of dispatch width -- mul_rs.v was never widened past 1-wide
issue ("mul is rare enough that a 2nd port isn't worth it").
"""
from dataclasses import dataclass


@dataclass
class Entry:
    tag: int
    instr: object
    s1_ready: bool
    s1_val: int
    s1_tag: int
    s2_ready: bool
    s2_val: int
    s2_tag: int


class MulRS:
    def __init__(self, depth):
        self.depth = depth
        self.entries = []

    def has_free(self, n=1):
        return (self.depth - len(self.entries)) >= n

    def alloc(self, tag, instr, s1_ready, s1_val, s1_tag, s2_ready, s2_val, s2_tag):
        self.entries.append(Entry(tag, instr, s1_ready, s1_val, s1_tag,
                                   s2_ready, s2_val, s2_tag))

    def snoop(self, broadcasts):
        for e in self.entries:
            for tag, val in broadcasts:
                if not e.s1_ready and e.s1_tag == tag:
                    e.s1_ready, e.s1_val = True, val
                if not e.s2_ready and e.s2_tag == tag:
                    e.s2_ready, e.s2_val = True, val

    def ready_entries(self, rob_head_tag):
        """Peek only -- see alu_rs.py's identical method for why (the
        shared CDB arbiter in cdb.py decides who actually wins)."""
        ready = [e for e in self.entries if e.s1_ready and e.s2_ready]
        if not ready:
            return []
        ready.sort(key=lambda e: (e.tag != rob_head_tag, e.tag))
        return ready[:1]  # 1 issue/cycle, unlike alu_rs.py -- see header

    def finalize(self, entry):
        self.entries.remove(entry)
        return entry.tag, (entry.s1_val * entry.s2_val) & 0xFFFFFFFFFFFFFFFF
