"""
Register Alias Table -- mirrors RV64I_OOO/src/rat.v. busy[r]/tag[r] per
architectural register (x0 excluded, always ready); a single checkpoint
(matching rat.v's own "single checkpoint, matching branch_rs.v's single-
outstanding-branch scoping" convention) for misprediction recovery.
"""


class RAT:
    def __init__(self):
        self.busy = {}   # reg -> bool
        self.tag = {}    # reg -> rob tag
        self._cp_busy = {}
        self._cp_tag = {}

    def is_busy(self, reg):
        return reg != 0 and self.busy.get(reg, False)

    def get_tag(self, reg):
        return self.tag.get(reg)

    def rename(self, rd, new_tag):
        """Mirrors rat.v's write_en: called once per dispatched
        instruction with a real destination register, in program order
        within the cycle so the youngest same-cycle writer to the same
        register wins (last write wins, same as rat.v's write_en-then-
        write2_en-then-write3_en ordering)."""
        if rd == 0:
            return
        self.busy[rd] = True
        self.tag[rd] = new_tag

    def commit_clear(self, rd, tag):
        """Mirrors rat.v's commit_clear_en: only clears busy if no
        younger rename has since remapped this register (tag still
        matches) -- the WAW-safe commit-clear discipline."""
        if rd == 0:
            return
        if self.tag.get(rd) == tag:
            self.busy[rd] = False

    def checkpoint_save(self):
        self._cp_busy = dict(self.busy)
        self._cp_tag = dict(self.tag)

    def checkpoint_restore(self):
        self.busy = dict(self._cp_busy)
        self.tag = dict(self._cp_tag)
