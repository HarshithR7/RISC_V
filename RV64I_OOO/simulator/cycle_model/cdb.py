"""
Common Data Bus arbiter -- mirrors the top-level CDB arbiter in
RV64I_OOO/src/riscv64_ooo_proc.v: an NREQ-sized array of requesters
(here: alu_rs, mul_rs, div_rs, lsq) reduced via "find oldest ready, mask,
repeat" down to `width` winners per cycle.

Split into peek (`ready_entries`) + commit (`finalize`) on each bank
specifically so this arbitration is centralized and real: alu_rs.py's
own up-to-`width` candidates still compete against mul_rs/div_rs/lsq for
the *same* shared pool of `width` broadcast slots, exactly like the real
RTL's 8-requester, `width`-pick arbiter (alu x3, branch x2 threads, mul,
div, lsq in the real design) -- not `width` slots reserved for ALU alone
plus separate slots for everyone else, which would understate real
contention whenever multiple banks complete in the same cycle.
"""


def arbitrate(banks, rob_head_tag, width):
    """banks: list of (bank_obj, entry) pairs is what a naive peek would
    give -- instead each bank exposes ready_entries(head_tag) returning
    its own already-oldest-first candidate list; this function merges
    all of them, applies the same "at-head-first, else oldest overall"
    priority used throughout this bank's own age sort, and finalizes
    only the winners. Returns the list of (tag, value) broadcasts."""
    candidates = []  # (is_head, tag, bank, entry)
    for bank in banks:
        for entry in bank.ready_entries(rob_head_tag):
            tag = entry.tag if hasattr(entry, "tag") else entry
            candidates.append((tag != rob_head_tag, tag, bank, entry))

    candidates.sort(key=lambda c: (c[0], c[1]))
    winners = candidates[:width]

    broadcasts = []
    for _, _, bank, entry in winners:
        broadcasts.append(bank.finalize(entry))
    return broadcasts
