"""
Cycle-level performance model driving loop -- mirrors
RV64I_OOO/src/riscv64_ooo_proc.v's own stage order: commit -> issue/
broadcast(CDB) -> dispatch(rename+alloc), one explicit step per
simulated cycle. Parameterized by the same names riscv64_ooo_proc.v
itself uses (width, rob_depth, alu_rs_depth, mul_rs_depth, lsq_depth),
so "change one parameter, re-run, see IPC move" is a direct analogy to
changing the RTL parameter of the same name.

Per-cycle ordering note: commit is processed before this cycle's own
issue/broadcast and dispatch, using ROB/RAT state as it stood at the
*start* of the cycle -- matching the real RTL's registered-state
semantics (a CDB mark this cycle only becomes commit-eligible starting
*next* cycle, since rob.v's done_arr is a registered, non-blocking
write). Dispatch, however, is allowed to see room commit already freed
*this same* cycle (a same-cycle "commit frees a slot, dispatch uses it"
shortcut this sequential Python model takes for simplicity) -- a
documented, deliberate approximation, not a claim of RTL-bit-exact
timing; that precision is what the actual Verilog simulation
(verify/bench_ooo.py) exists for.

Scope cuts (see this directory's other modules for the specific ones):
single-thread only, no RVV, no branch redirect/squash wiring yet (none
of the shared straight-line benchmarks exercise it -- branch_rs.py is
implemented and ready, just not yet driven by this loop), no real
memory-latency modeling in lsq.py.
"""
from dataclasses import dataclass

import isa
from cycle_model.rat import RAT
from cycle_model.rob import ROB
from cycle_model.alu_rs import ALURS
from cycle_model.mul_rs import MulRS
from cycle_model.div_rs import DivRS
from cycle_model.lsq import LSQ
from cycle_model import cdb


@dataclass
class CycleResult:
    instr_count: int
    cycles: int
    ipc: float


def simulate(asm_text, width=3, rob_depth=8, alu_rs_depth=4, mul_rs_depth=2, lsq_depth=4):
    instrs = isa.assemble(asm_text)
    n = len(instrs)

    rat = RAT()
    rob = ROB(rob_depth)
    alu_rs = ALURS(alu_rs_depth, width)
    mul_rs = MulRS(mul_rs_depth)
    div_rs = DivRS()
    lsq = LSQ(lsq_depth)

    def bank_for(instr):
        if instr.is_mul:
            return mul_rs, "mul"
        if instr.is_div:
            return div_rs, "div"
        if instr.is_load or instr.is_store:
            return lsq, "mem"
        return alu_rs, "alu"  # alu, branch, jal, jalr, ecall (see header)

    pc = 0
    committed = 0
    cycle = 0
    guard_limit = 2_000_000
    while committed < n:
        cycle += 1
        if cycle > guard_limit:
            raise RuntimeError("cycle_model: did not converge (possible deadlock)")

        # ---- 1. Commit (in-order, cascading, using state from before
        # this cycle's own broadcasts -- see header) ----------------------
        ready = rob.commit_ready(width)
        for e in ready:
            rat.commit_clear(e.rd, e.tag)
        rob.commit(len(ready))
        committed += len(ready)

        # ---- 2. Issue/broadcast: div advances its countdown every
        # cycle regardless of arbitration; all 4 banks then compete for
        # the shared, width-wide CDB (see cdb.py's header for why this
        # is centralized, not per-bank dedicated slots) -------------------
        div_rs.step()
        head_tag = rob.entries[rob.head].tag if rob.count > 0 else None
        broadcasts = cdb.arbitrate([alu_rs, mul_rs, lsq, div_rs], head_tag, width)
        for tag, val in broadcasts:
            rob.mark_done(tag, val)
        for bank in (alu_rs, mul_rs, div_rs, lsq):
            bank.snoop(broadcasts)

        # ---- 3. Dispatch: up to `width` instructions, in order,
        # cascading (stop at the first one this cycle can't resource --
        # matches the RTL's own in-order dispatch discipline) -------------
        for _ in range(width):
            if pc >= n or rob.free_count() < 1:
                break
            instr = instrs[pc]
            bank, cls = bank_for(instr)
            if not bank.has_free(1):
                break

            def resolve(reg):
                if reg == 0:
                    return True, 0, None
                if rat.is_busy(reg):
                    producer_tag = rat.get_tag(reg)
                    # Mirrors rob.v's lookup ports (rob_rs1_done/
                    # rob_rs1_value in riscv64_ooo_proc.v): the producer
                    # may have already broadcast on the CDB in an
                    # earlier cycle, before this consumer even existed
                    # to snoop it -- RAT alone can't tell (it only
                    # clears busy at commit, which can trail broadcast
                    # by many cycles), so a stale-snoop-window miss is
                    # avoided by checking the ROB directly here too, not
                    # relying on the bank's snoop() alone (see
                    # cycle_model/rob.py's lookup() for why this is a
                    # real, previously-missed bug, not defensive
                    # boilerplate).
                    done, value = rob.lookup(producer_tag)
                    if done:
                        return True, value, None
                    return False, 0, producer_tag
                return True, 0, None  # value itself doesn't matter -- see alu_rs.py

            if instr.src1_is_dep:
                s1_ready, s1_val, s1_tag = resolve(instr.rs1)
            else:
                s1_ready, s1_val, s1_tag = True, 0, None  # LUI/AUIPC/JAL/ecall
            if instr.src2_is_imm:
                # Carries the real immediate value so dependency chains
                # through it stay numerically sane, even though the
                # exact arithmetic doesn't matter for timing (see
                # alu_rs.py's _alu_result).
                s2_ready, s2_val, s2_tag = True, instr.imm, None
            else:
                s2_ready, s2_val, s2_tag = resolve(instr.rs2)

            tag = rob.alloc(instr.rd, instr.reg_write, is_store=instr.is_store, is_ecall=instr.is_ecall)
            if cls == "mul":
                bank.alloc(tag, instr, s1_ready, s1_val, s1_tag, s2_ready, s2_val, s2_tag)
            elif cls == "div":
                bank.alloc(tag, instr, s1_ready, s1_val, s1_tag, s2_ready, s2_val, s2_tag)
            elif cls == "mem":
                bank.alloc(tag, instr, s1_ready, s1_val, s1_tag)
            else:
                bank.alloc(tag, instr, s1_ready, s1_val, s1_tag, s2_ready, s2_val, s2_tag)

            if instr.reg_write and instr.rd != 0:
                rat.rename(instr.rd, tag)
            pc += 1

    return CycleResult(instr_count=n, cycles=cycle, ipc=n / cycle if cycle else 0.0)


if __name__ == "__main__":
    import workloads
    for b in workloads.all_benchmarks():
        r = simulate(b.source())
        print(f"{b.name:<32} cycles={r.cycles:<6} ipc={r.ipc:.3f}")
