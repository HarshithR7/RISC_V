"""
Reorder Buffer -- mirrors RV64I_OOO/src/rob.v. Circular buffer, up to
`width` allocations and `width` commits per cycle, per-entry done bit.
Commit is strictly in-order and cascading (head2 can only commit if head
also commits this cycle, mirroring rob.v's own commit_req/commit_req2/
commit_req3 chain and this session's own ecall-as-commit-barrier fix --
see riscv64_ooo_proc.v's header).
"""
from dataclasses import dataclass, field


@dataclass
class ROBEntry:
    tag: int
    rd: int
    has_dest: bool
    is_store: bool = False
    is_ecall: bool = False
    done: bool = False
    value: int = 0


class ROB:
    def __init__(self, depth):
        self.depth = depth
        self.entries = [None] * depth
        self.head = 0
        self.tail = 0
        self.count = 0

    @property
    def full(self):
        return self.count == self.depth

    def free_count(self):
        return self.depth - self.count

    def alloc(self, rd, has_dest, is_store=False, is_ecall=False):
        """Allocates one entry, returning its tag (== its slot index --
        matches rob.v's own tag==tail_ptr convention). Caller must check
        free_count() first."""
        tag = self.tail
        self.entries[tag] = ROBEntry(tag=tag, rd=rd, has_dest=has_dest,
                                      is_store=is_store, is_ecall=is_ecall)
        self.tail = (self.tail + 1) % self.depth
        self.count += 1
        return tag

    def mark_done(self, tag, value):
        e = self.entries[tag]
        e.done = True
        e.value = value

    def lookup(self, tag):
        """Mirrors rob.v's lookup1..6 ports (rob_rs1_done/rob_rs1_value
        in riscv64_ooo_proc.v): a dispatch-time consumer whose producer
        already broadcast on the CDB *before* this cycle -- but hasn't
        committed yet, so RAT still reports it busy -- must still see it
        as ready, by reading the ROB entry directly, not by waiting on a
        CDB snoop that already happened and won't repeat. Returns
        (done, value); (False, 0) for an unallocated/stale tag."""
        e = self.entries[tag]
        if e is None:
            return False, 0
        return e.done, e.value

    def commit_ready(self, width):
        """Returns up to `width` entries eligible to commit this cycle,
        oldest-first, cascading: an entry only counts if every older
        entry in this batch also committed, and (this session's real
        RTL fix) an ecall is a hard commit barrier -- nothing younger
        commits alongside it."""
        ready = []
        idx = self.head
        for _ in range(min(width, self.count)):
            e = self.entries[idx]
            if e is None or not e.done:
                break
            ready.append(e)
            if e.is_ecall:
                break
            idx = (idx + 1) % self.depth
        return ready

    def commit(self, n):
        """Retires the oldest `n` entries (must equal len(commit_ready())'s
        result from this same cycle)."""
        for _ in range(n):
            self.entries[self.head] = None
            self.head = (self.head + 1) % self.depth
            self.count -= 1

    def squash_younger_than(self, tag):
        """Mirrors rob.v's squash_valid/squash_tag: invalidates every
        entry younger than `tag` (used on a branch misprediction)."""
        idx = (tag + 1) % self.depth
        removed = 0
        # Walk from tag+1 up to (but not including) tail.
        while idx != self.tail:
            self.entries[idx] = None
            idx = (idx + 1) % self.depth
            removed += 1
        self.tail = (tag + 1) % self.depth
        self.count -= removed
