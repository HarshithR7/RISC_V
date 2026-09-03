"""
Branch-class reservation station + a simple direction predictor --
mirrors RV64I_OOO/src/branch_rs.v (exactly 1 entry, correct by
construction since a branch-class instruction blocks dispatch of
anything younger until it resolves) and bht.v (a 2-bit saturating
counter table, here simplified to a plain dict since this model has no
fixed-size-table aliasing behavior worth reproducing).

Not exercised by the current straight-line shared benchmarks (see
simulator/workloads.py -- none contain a branch/JAL/JALR), but
implemented for structural fidelity with the real RTL and to keep this
model ready for a future branchy workload without needing a redesign --
called out explicitly, not left as silent dead code.
"""


class BHT:
    def __init__(self):
        self.counter = {}  # pc -> 0..3 (>=2 predicts taken), default weakly-not-taken

    def predict(self, pc):
        return self.counter.get(pc, 1) >= 2

    def update(self, pc, taken):
        c = self.counter.get(pc, 1)
        if taken:
            c = min(3, c + 1)
        else:
            c = max(0, c - 1)
        self.counter[pc] = c


class BranchRS:
    def __init__(self):
        self.busy = False
        self.tag = None
        self.instr = None
        self.predicted_taken = None

    def has_free(self, n=1):
        return not self.busy

    def alloc(self, tag, instr, predicted_taken):
        self.busy = True
        self.tag, self.instr, self.predicted_taken = tag, instr, predicted_taken

    def resolve(self, actual_taken):
        """Branches resolve the same cycle their operands are ready in
        the real RTL (branch_rs.v) -- this model treats dispatch and
        resolution as coincident for the same reason, since operand
        readiness for a branch's own compare is the only real latency.
        Returns (mispredicted, tag)."""
        mispredicted = actual_taken != self.predicted_taken
        tag = self.tag
        self.busy = False
        return mispredicted, tag
