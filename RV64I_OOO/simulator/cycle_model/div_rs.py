"""
Divide reservation station -- mirrors RV64I_OOO/src/div_rs.v +
div_fu.v. Exactly 1 entry (matches this project's own "correct by
construction... a second slot would be permanently dead capacity"
scope decision, restated for the single-outstanding divider), real
multi-cycle occupancy (isa.LAT_DIV cycles), non-abortable once started
(mirrors div_fu.v's own "no abort input" precedent -- a squash while
mid-division can only disown the eventual result, never stop it early;
this model has no squash path exercised by the current straight-line
benchmarks, so that interaction is structurally absent here, not
silently wrong).
"""
import isa


class DivRS:
    def __init__(self):
        self.busy = False
        self.tag = None
        self.instr = None
        self.s1_ready = False
        self.s1_val = 0
        self.s1_tag = None
        self.s2_ready = False
        self.s2_val = 0
        self.s2_tag = None
        self.remaining = 0
        self.running = False
        self.result_ready = False
        self.result_val = 0

    def has_free(self, n=1):
        return not self.busy

    def alloc(self, tag, instr, s1_ready, s1_val, s1_tag, s2_ready, s2_val, s2_tag):
        self.busy = True
        self.tag, self.instr = tag, instr
        self.s1_ready, self.s1_val, self.s1_tag = s1_ready, s1_val, s1_tag
        self.s2_ready, self.s2_val, self.s2_tag = s2_ready, s2_val, s2_tag
        self.running = False

    def snoop(self, broadcasts):
        if not self.busy:
            return
        for tag, val in broadcasts:
            if not self.s1_ready and self.s1_tag == tag:
                self.s1_ready, self.s1_val = True, val
            if not self.s2_ready and self.s2_tag == tag:
                self.s2_ready, self.s2_val = True, val

    def step(self):
        """Called once per cycle: starts the divide the cycle both
        operands become ready, counts down isa.LAT_DIV cycles, then
        holds the completed result pending broadcast -- matching
        branch_rs.v's own documented precedent that an entry stays
        occupied "until its result is actually broadcast and granted on
        the CDB -- not just resolved," so a divide that loses CDB
        arbitration (see cdb.py) simply re-presents itself as ready
        next cycle instead of silently completing early."""
        if not self.busy or self.result_ready:
            return
        if not self.running:
            if self.s1_ready and self.s2_ready:
                self.running = True
                self.remaining = isa.LAT_DIV
            else:
                return
        self.remaining -= 1
        if self.remaining <= 0:
            divisor = self.s2_val if self.s2_val != 0 else 1
            self.result_val = (self.s1_val // divisor) & 0xFFFFFFFFFFFFFFFF
            self.result_ready = True

    def ready_entries(self, rob_head_tag):
        """Peek only -- see alu_rs.py's identical method for why."""
        return [self] if self.result_ready else []

    def finalize(self, entry):
        tag, val = self.tag, self.result_val
        self.busy = False
        self.running = False
        self.result_ready = False
        return tag, val
