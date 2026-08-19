# RV64I_OOO — Out-of-Order RV64I+M Core

A Tomasulo-algorithm-plus-reorder-buffer out-of-order core, built as a
multi-phase plan to take the sibling
[single-cycle RV64IMACFD core](../RV64I/README.md) (Phase 0) toward
dynamic scheduling, branch prediction/speculation, and 2-issue
superscalar dispatch. Scope is deliberately narrower than the
single-cycle core's: RV64I+M only (no A/C/F/D/V). Everything here is new
RTL in its own directory (`RV64I_OOO/`) — the single-cycle core is
untouched and still the reference implementation for the full
RV64IMACFD+RVV feature set.

## Status

**Phases 1 through 9 are all complete: 33/33 single-core full-pipeline
tests pass, plus a dedicated 2-core coherency test**, plus 21/21 isolated
ROB+RAT unit-test checks, 15/15 standalone divider unit tests, 13/13
isolated L1/L2 MESI coherency checks, 2/2 lockstep DMR checks, and 6/6
isolated ECC checks.

- **Phase 1**: Tomasulo + ROB, out-of-order execution, strictly in-order
  commit, no speculation. Every instruction class in scope (ALU,
  conditional branches, JAL/JALR, MUL/MULH/MULHSU/MULHU (+ W forms),
  DIV/DIVU/REM/REMU (+ W forms), loads/stores with real load/store-queue
  disambiguation) dispatches out of order, executes out of order, and
  commits in program order.
- **Phase 2**: conditional branches are dynamically predicted (a BHT) and
  executed speculatively, with real misprediction recovery: reservation-
  station and LSQ squash, a RAT checkpoint/restore, and correct redirect
  to the actual resolved target.
- **Phase 3**: dispatch and rename are 2-wide (superscalar) — two
  instructions per cycle are renamed and put in flight together, with a
  real intra-group RAW/WAW forwarding path for when the younger of the
  pair depends on (or overwrites) the older's own destination register.
- **Phase 4**: a real, measured benchmark suite comparing dual- vs
  single-issue dispatch on identical RTL — see "Phase 4: benchmarking
  dispatch width, honestly" below for what it actually found (not the
  naive expectation).
- **Phase 5**: widens the CDB and commit path to 2-wide, closing the gap
  Phase 4 found — see "Phase 5: widening the CDB and commit path" below.
- **Phase 6**: real, scoped RVV vector execution (data-level parallelism)
  — see "Phase 6: data-level parallelism (vector)" below.
- **Phase 7**: 2-thread simultaneous multithreading (SMT) — see "Phase 7:
  simultaneous multithreading" below.
- **Phase 8**: a real 2-core system with private L1 data caches, a shared
  L2, and genuine MESI coherency, wired all the way into the OoO
  pipeline's actual load/store execution — see "Phase 8: multi-level
  cache + MESI coherency" below.
- **Phase 9**: redundancy, in two complementary parts. First, lockstep
  dual-modular redundancy (DMR) — two fully independent cores run the
  identical program and are continuously compared, with a sticky fault
  flag on any divergence — see "Phase 9: lockstep redundancy (DMR)"
  below. Second, real SECDED ECC (single-error-correct, double-error-
  detect) on the register file, L1, L2, and ROB payload storage — see
  "Phase 9 continued: ECC on memory structures" below.

Tests cover RAW/WAR/WAW hazards (including intra-group ones specific to
2-wide dispatch), a real backward-branch loop (which, as a side effect of
dynamic prediction, exercises *both* a cold-start misprediction on its
first iteration *and* a loop-exit misprediction on its last — see "Bugs
found" in the Phase 2 section), reservation-station exhaustion
(structural hazard), a long-latency divide overlapping with independent
short ops, memory aliasing/non-aliasing through the LSQ, dedicated
forward-branch misprediction tests including one that specifically proves
a squashed store never reaches memory, and dedicated dual-issue tests for
intra-group forwarding and intra-group WAW.

## Architecture

| File (`src/`) | Module | Role |
|---|---|---|
| `riscv64_ooo_proc.v` | `riscv64_ooo_proc` | Top level: fetch/decode/rename/dispatch, CDB arbiter, commit |
| `decode_ooo.v` | `decode_ooo` | Combinational RV64I+M decode, classified by destination RS bank |
| `rat.v` | `rat` | Register Alias Table (single, no speculation checkpointing yet) |
| `rob.v` | `rob` | Reorder buffer: in-order commit, precise state |
| `alu_rs.v` | `alu_rs` | ALU reservation stations (4 entries) + combinational 1-cycle ALU |
| `branch_rs.v` | `branch_rs` | Branch/JAL/JALR reservation station (exactly 1 entry) |
| `mul_rs.v` | `mul_rs` | Multiply reservation stations (2 entries) + combinational 1-cycle multiplier |
| `div_rs.v` | `div_rs` | Divide reservation station (1 entry), wraps `div_fu` |
| `div_fu.v` | `div_fu` | Iterative (multi-cycle) restoring-division functional unit |
| `lsq.v` | `lsq` | Load-store queue (4 entries), age-ordered alias disambiguation |
| `bht.v` | `bht` | Branch History Table: 2-bit saturating-counter direction prediction (Phase 2) |

Reused unchanged from the single-cycle RV64I core: `program_counter.v`,
`instruction_fetch.v` (RVC halfword-awareness simply unused — PC always
advances by exactly 4), `register_file.v` (written only at commit),
`data_memory.v` (scalar port only; the RVV vector port is tied off).

### The pipeline shape

Fetch → Decode → Dispatch (rename via RAT, allocate a ROB entry, allocate
into the destination RS bank) → Execute (each RS bank runs and requests
the CDB independently) → CDB arbitration (oldest-ROB-tag-first across all
five requesters: ALU, branch, mul, div, LSQ-loads) → ROB mark-done →
Commit (strictly in-order, retiring exactly the ROB head each cycle it's
ready).

Dispatch is single-issue: at most one instruction is renamed and
allocated per cycle. Execution is fully out-of-order: any RS bank with a
ready entry can broadcast on the CDB the moment its result exists,
regardless of how many older instructions are still in flight elsewhere.
Commit is what makes this look, from the architectural register file's
point of view, exactly like an in-order machine — the register file and
data memory are only ever written at commit, never speculatively.

### Register renaming discipline

- Dispatch reads rs1/rs2's *current* RAT status before writing rd's new
  tag, avoiding a self-dependency deadlock for `add x1, x1, x1`-shaped
  instructions.
- Commit clears `RAT[rd]` only if it still points at the committing tag —
  this is what resolves WAW for the RAT itself (a younger producer's
  remap must never be clobbered by an older instruction's late commit).
- The architectural register file is written *only* at commit, never at
  CDB-broadcast time — this is the actual mechanism that resolves WAW for
  final architectural state, independent of the RAT.
- `x0` bypasses renaming entirely, mirroring `register_file.v`'s
  hardwiring.
- One RAT is sufficient for Phase 1 (no speculation, so no
  checkpoint/rollback is needed yet) — Phase 2 (branch prediction) is
  exactly where that stops being true.

### CDB arbitration

Oldest-ROB-tag-first, using circular distance from the *current* ROB head
(`age(tag) = tag - rob_head_tag`, wrapping correctly via unsigned
subtraction on a power-of-2-depth ROB) — not raw numeric tag comparison,
which is wrong across a wraparound (a just-allocated, youngest entry can
have a numerically smaller tag than an older one still in flight). This
policy has the useful property that the ROB head, when it's itself a
requester, always has age 0 and therefore always wins immediately, so
arbitration never delays in-order commit. Widened incrementally from 2-way
(ALU/branch) to the full 5-way (ALU/branch/mul/div/LSQ-loads) as each
functional unit was added, via a pairwise reduction (`ab` = oldest of
{alu,branch}, `abm` = oldest of {ab,mul}, `abmd` = oldest of {abm,div},
final = oldest of {abmd,lsq}).

### Branch prediction + speculation (Phase 2)

Phase 1 blocked dispatch of anything younger than an outstanding
branch-class instruction until it resolved. Phase 2 lifts that for
conditional branches specifically:

- **JAL** never needed prediction to begin with — its target is exact at
  decode (pc+imm, no register dependency) — so dispatch now redirects
  fetch to it immediately and unconditionally. There's no "prediction"
  here, so nothing can ever be mispredicted, and no rollback machinery is
  needed for it at all.
- **Conditional branches** get real dynamic prediction: a 64-entry BHT
  (`bht.v`, 2-bit saturating counters, PC-indexed, initialized "weakly
  not-taken") predicts the direction at dispatch time, and fetch
  continues down the predicted path speculatively. Exactly one
  speculative branch may be outstanding at a time — matching
  `branch_rs`'s own single-entry sizing, so a single RAT checkpoint (see
  below) is always enough; no checkpoint stack is needed. When the branch
  resolves, its actual outcome is compared against the prediction that
  was made; a correct prediction needs no further action (dispatch
  already went the right way), while a misprediction triggers full
  rollback (see below) and a redirect to the branch's real target.
- **JALR** keeps Phase 1's conservative stall-until-resolved behavior —
  its target depends on a register value not known until execute, and
  predicting it would need a separate mechanism (a branch-target buffer
  or return-address stack) that's out of scope here.

#### Misprediction recovery

A misprediction squashes every instruction younger than the mispredicted
branch and rolls fetch back to its real target:

- **Every reservation-station bank and the LSQ** (`alu_rs`, `mul_rs`,
  `div_rs`, `lsq`) gets a `squash_valid`/`squash_tag` port: any resident
  entry strictly younger than the branch (via the same
  age()-relative-to-`rob_head_tag` comparison the CDB arbiter and the
  LSQ's own disambiguation already use) is cleared. `branch_rs` itself
  needs no squash logic — since only one branch-class instruction can
  ever be outstanding, nothing younger could have dispatched into it
  while the mispredicted branch was still unresolved.
- **`rob.v`** gets a `squash_valid`/`squash_tag` port that rolls
  `tail_ptr` back to just past the branch's own tag, discarding every
  younger, speculatively-allocated entry — computed via the same
  age()-relative-to-head-pointer arithmetic the normal `count` bookkeeping
  uses, so it composes safely with an unrelated, completely independent
  older entry committing the very same cycle.
- **`rat.v`** gets a `checkpoint_save`/`checkpoint_restore` pair backed
  by a shadow copy of the whole table. `checkpoint_save` snapshots live
  state the cycle a conditional branch dispatches; from then on, the
  shadow mirrors every `commit_clear_en` event the live table also sees
  (always safe — any commit during a speculative window is guaranteed
  older than the outstanding branch, since commit is strictly in-order).
  `checkpoint_restore` overwrites live state from the (kept-current)
  shadow on a misprediction, discarding exactly the renames caused by the
  now-squashed speculative instructions while preserving every
  legitimate commit that happened in the meantime.
- **Dispatch itself** is suppressed the exact cycle a misprediction is
  discovered (`!mispredict` gates `dispatch_fire`) — the instruction
  currently being decoded was fetched along the just-discovered-wrong
  path and must never be allowed to rename or allocate.
- **`div_fu`** has no abort input, so a squash mid-division can't stop
  it — `div_rs.v` instead keeps its own `full` asserted (blocking
  reallocation) until the disowned division's eventual `done` pulse has
  nowhere new to corrupt, then goes idle normally.

### The Load-Store Queue

Loads and stores share one 4-entry queue, age-ordered by each entry's
captured ROB tag. A load may execute (read memory and broadcast) once its
address is known, *unless* some older, still-resident store either (a)
doesn't know its own address yet (can't rule out aliasing), or (b) has a
known address that overlaps the load's, at doubleword granularity. Phase
1 does no store-to-load forwarding, so the aliasing case simply blocks
the load — and since a store only leaves the queue at commit, "block
until the aliasing store commits" falls out for free from "block while
that store is still resident," with no extra state needed.

Stores never broadcast a value (no destination register); once both their
address and data operands are known, they become commit-eligible via a
dedicated ROB port (see "the mark2/extra_mark story" below), and the
actual memory write happens only at commit — never speculatively, exactly
mirroring the register file. The data-memory module has one shared
read/write address port, so a store's commit-time write and a load's
speculative read can't both use it the same cycle; `lsq.v` simply stalls
load issue for that one cycle when a commit-write is happening (a real,
deliberately simple resolution of that structural hazard).

### The mark2/extra_mark story

Most instructions get their ROB entry marked "done" by winning CDB
arbitration. Two classes of instruction never touch the CDB at all but
still need their ROB entry to become commit-eligible:

- **A resolved conditional branch** has no destination register, so it
  never broadcasts. `rob.v` has a second, unarbitrated `mark2` port for
  this — safe without arbitration because `branch_rs` has exactly one
  entry, so at most one branch can ever resolve in a given cycle.
- **A store whose address and data are both known** similarly has
  nothing to broadcast. Since the LSQ has 4 entries, *multiple* stores
  can legitimately become ready the same cycle — a single-tag port like
  mark2 isn't enough. `rob.v`'s `extra_mark` port generalizes the same
  idea to `EXTRA_MARK_N` (= `LSQ_DEPTH`) parallel lanes, each carrying an
  independent valid+tag pair. No arbitration is needed here either: ROB
  tags are always distinct by the allocator's own invariant, so `N`
  simultaneous marks targeting `N` different tags never conflict.

### 2-wide dispatch (Phase 3)

Fetch, decode, and dispatch/rename all widen to two lanes (lane 0 = older,
lane 1 = younger, fetched from `pc` and `pc+4`) — but the CDB, execution
completion, and commit all stay exactly 1-wide. This is a deliberate
design point, not a shortcut: a 2-wide front end feeding a 1-wide back
end still delivers the real ILP win Phase 3 is about (reservation
stations get fed twice as fast, so they're less often empty/stalled)
without the much larger complexity of a fully doubled execute/commit path
(a multi-port register file, dual ROB retire, doubled CDB arbitration).
`program_counter`/`instruction_fetch`/`register_file` are each
instantiated twice (once per lane) rather than widened internally — since
each is a deterministic, read-mostly structure with a single shared write
path, two instances fed identical clock/reset/write signals always hold
identical state, so this is really just extra read ports on what is
logically one array, the same way a real dual-read-port memory would be
built.

Three scoping decisions keep the two-lane interaction tractable:

- **Lane 1 can never itself be a branch-class instruction** (conditional
  branch, JAL, or JALR) — only lane 0 can. This avoids ever needing to
  reason about two branches (and two possible redirects) landing in the
  same fetch group; `branch_rs.v` still has just one, lane-0-only
  allocation port.
- **Both lanes are fetched every cycle from their straight-line
  addresses**, regardless of what lane 0 turns out to be. If lane 0
  dispatches as something that breaks that assumption (JAL, JALR, or a
  conditional branch predicted taken), lane 1's speculatively-fetched
  instruction is simply never dispatched that cycle — discarded, not
  squashed, since it never allocated anything — and gets correctly
  re-fetched from the real target the following cycle.
- **Intra-group RAW/WAW forwarding**: if lane 1 depends on lane 0's own
  destination register, both instructions are renaming in the very same
  cycle, so the RAT's raw read for lane 1 doesn't yet reflect lane 0's
  not-yet-registered rename. `rat.v` gained a second, independent read
  port (`rs1b`/`rs2b`) giving that *raw* (pre-lane-0-rename) view;
  `riscv64_ooo_proc.v`'s dispatch logic layers the actual intra-group
  bypass on top, overriding lane 1's operand source to point directly at
  lane 0's own newly-allocated ROB tag whenever `d0_reg_write &&
  d1_rs1==d0_rd` (or `rs2`). rat.v itself stays lane-agnostic — it just
  provides two read ports and two write ports (`write_en` then
  `write2_en`, applied in that program order so an intra-group WAW
  correctly resolves to lane 1's rename, exactly the same
  last-non-blocking-assignment-wins reasoning as the existing
  commit-vs-rename race).

Every multi-entry reservation-station bank (`alu_rs`, `mul_rs`, `lsq`)
gained a second allocation port (`alloc2_*`/`has_2_free`), generalizing
"pick the lowest free slot" to "pick the lowest free slot for lane 0,
then the next-lowest *distinct* free slot for lane 1." `rob.v` similarly
gained dual allocation (`alloc2_*`, plus a `free_count` output so the top
level can gate lane 1 on "at least 2 free" rather than just "at least 1
free"). The single-entry banks (`branch_rs`, `div_rs`) can't accept two
allocations in one cycle by construction — `div_rs` gets a mux at the top
level so *either* lane can feed its one port (whichever actually wants a
divide; if both do, lane 1 simply doesn't fire that cycle), while
`branch_rs` only ever needs lane 0, per the scoping decision above.

### Sizing

ROB = 8 entries, ALU RS = 4, Branch RS = exactly 1 (correct by
construction — Phase 1 stalls dispatch while any branch is outstanding,
so a second slot would be permanently dead capacity), Mul RS = 2, Div RS
= exactly 1 (this scoped core never needs two divides in flight at once),
LSQ = 4.

## Build history

Built incrementally, each stage tested in real simulation before the next
was added:

1. **`div_fu.v` standalone** — the iterative (not combinational) divider,
   unit-tested in isolation (15/15). Deliberately multi-cycle: a direct
   response to the single-cycle RV64 core's finding that a combinational
   64-bit divider synthesizes to ~149,000 gates and dominates that core's
   area (see `../RV64I/README.md`'s "Area and frequency" section). An
   out-of-order core is exactly the kind of machine built to hide that
   latency behind independent work, rather than pay for it in silicon.
2. **`rob.v` + `rat.v` isolated** — unit-tested directly against each
   other with no fetch/decode/RS/FU involved (21/21 checks: RAW, WAW
   across two different commit cycles, x0 exclusion, ROB-full stall and
   drain, and a same-cycle commit-vs-rename race).
3. **Minimal ALU-only pipeline** — first real end-to-end dispatch →
   execute → CDB → commit loop, no branches yet.
4. **Branch/JAL/JALR support** — found and fixed a real bug: conditional
   branches have no destination register and thus never win (or even
   request) the CDB, so nothing was marking their ROB entry done, and
   commit stalled forever the instant one reached the head. Fixed by
   adding `rob.v`'s `mark2` port (see above). A second, more subtle bug
   surfaced only in a real backward-branch loop test — see "Bugs found"
   below; it took several rounds of hand-tracing internal RS/ROB state to
   isolate.
5. **Multiply** — `mul_rs.v` (2 entries) + widened the CDB arbiter to
   3-way. Includes an RS-exhaustion stress test (three independent
   multiplies dispatched back-to-back, forcing the third to genuinely
   stall on a full reservation-station bank).
6. **Divide** — wired the already-unit-verified `div_fu.v` into `div_rs.v`
   (1 entry) + widened the CDB arbiter to 4-way. Includes a long-latency
   demonstration test: one ~64-cycle divide followed by several
   independent short adds, checking both final values and that total
   cycle count stays close to one divide's latency rather than scaling
   with the number of trailing instructions.
7. **Load/store** — `lsq.v` (4 entries) + `data_memory.v` wired in +
   widened the CDB arbiter to 5-way. Built with real age-ordered
   disambiguation from the start (not retrofitted): dedicated aliasing
   (RAW-through-memory, same address) and non-aliasing (independent
   addresses) tests, plus a sub-doubleword width test (byte/halfword,
   sign vs zero extension).
8. **Full regression (Phase 1 "done" milestone)** — every hazard class
   (RAW/WAR/WAW, structural, control, long-latency, memory aliasing) plus
   the RV64I+M functional subset, through the fully integrated core:
   19/19.
9. **Branch prediction + speculation (Phase 2)** — `bht.v` (a 2-bit
   saturating-counter table), a RAT checkpoint/restore mechanism, and a
   `squash_valid`/`squash_tag` port threaded through every reservation-
   station bank, the LSQ, and the ROB. Includes dedicated tests for a
   guaranteed cold-start misprediction (BHT defaults to not-taken, so a
   branch that's actually taken on its first occurrence always
   mispredicts), a variant proving a squashed store never reaches memory,
   and a correctly-predicted control case. The existing backward-branch
   loop test from step 4 turned out to be a bonus regression check here
   too: with real prediction now active, it exercises a cold-start
   misprediction on its first iteration *and* a loop-exit misprediction
   on its last (predicted taken from warmup, actually not-taken) without
   any changes to the test itself — both confirmed correct via a direct
   internal trace before trusting the passing result alone. All 8/8
   step-4-era tests plus everything from steps 5-8 continue to pass
   unmodified, with most actually running in fewer cycles than before
   (JAL and correctly-predicted branches no longer stall dispatch at
   all).
10. **2-wide dispatch (Phase 3)** — widened fetch/decode/dispatch/rename
    to two lanes across `rob.v`, `rat.v`, `alu_rs.v`, `mul_rs.v`,
    `lsq.v`, and `div_rs.v`'s top-level mux, plus the intra-group
    forwarding logic in `riscv64_ooo_proc.v`. Hit one real, genuinely
    tricky bug on the first test run (see "Bugs found" below); after
    fixing it, all pre-existing tests continued to pass unmodified, plus
    new dedicated tests for intra-group RAW forwarding and intra-group
    WAW.
11. **Benchmarking (Phase 4)** — an `ENABLE_DUAL_ISSUE` parameter plus
    `bench_ooo.py`/`tb_bench_ooo.v` for direct dual- vs single-issue
    comparison on identical RTL. Found, and fixed, a second real bug
    along the way: `alu_rs`/`mul_rs`'s issue arbitration was fixed
    lowest-slot-index priority (a Phase 1 simplification, documented at
    the time as "never actually starves anything in practice"); Phase
    3's burstier 2-wide allocation pattern was enough to actually trigger
    that starvation, measurably (dual-issue running *slower* than
    single-issue on fully independent work, which should never happen).
    Fixed by switching issue arbitration to oldest-ROB-tag-first, the
    same policy the CDB arbiter and LSQ already used. See "Phase 4:
    benchmarking dispatch width, honestly" below for what the benchmarks
    found even after that fix.

## Bugs found

The two most significant bugs were both real architectural gaps, not
typos — both found by hand-tracing internal reservation-station/ROB state
in a temporary hierarchical-reference debug testbench (`$display` reading
`uut.<module>.<signal>` paths directly) rather than by inspection.

**1. Conditional branches never marked their own ROB entry done.**
Diagnosed via a trace showing `rob_full=0`, `branch_rs_full=1`,
`branch_outstanding=1` stuck forever with no CDB broadcast ever
targeting the branch's tag. Root cause: branches have no destination
register, so they never request the CDB at all, and nothing else was
marking them done either. Fixed by `rob.v`'s `mark2` port (see
Architecture above).

**2. A one-cycle gap between a producer's CDB broadcast and its RAT
clear could permanently strand a consumer.** This one took much longer
to find: a real backward-branch loop (`sum 1..5`) timed out, with
`addi x6, x6, 1` inside the loop body never receiving its operand. Tracing
`alu_rs`'s internal per-slot state showed the entry sitting with
`s1_ready=0, s1_tag=<x6's producer>` forever, even though that producer
had already broadcast — the trace showed the broadcast happening in the
cycle *before* the consumer dispatched, not the same cycle.

The root cause: dispatch's operand-readiness check only had two cases —
"not renamed, read the register file" or "renamed, and this exact
cycle's CDB broadcast happens to match" (a same-cycle bypass, needed
because the CDB is a one-shot pulse, not a latched bus). It was missing a
third case: a producer's tag can broadcast, then *some cycle later* — but
still before that producer's ROB entry actually commits and clears the
RAT — a consumer dispatches needing that same register. In that gap, the
RAT still (correctly) shows the register busy, but the CDB no longer
carries the tag (it already broadcast, once, and won't again), so the
consumer parks itself waiting for an event that has already happened and
will never recur. This isn't a narrow corner case — any producer/consumer
pair separated by enough intervening instructions hits it, which is
exactly why it took a real loop (not the earlier, shorter hazard tests)
to expose it.

Fixed by giving `rob.v` two combinational value-lookup ports
(`lookup1`/`lookup2`, keyed by `rs1_tag`/`rs2_tag`): once an entry is
marked done, its value stays validly readable there — unlike the CDB's
one-shot pulse — right up until commit. Dispatch's readiness check now
has all three cases: not renamed, same-cycle CDB bypass, or
already-done-in-the-ROB. This is safe for any tag dispatch presents,
since the RAT's busy/tag mapping is kept in sync with an actual
outstanding (allocated, not-yet-committed) ROB entry by construction.

Phase 2 itself went in cleanly on the first real test run (22/22 passing
immediately after wiring), largely because the squash/checkpoint design
was worked out on paper first — in particular, reasoning through the
"does the RAT checkpoint correctly track older instructions that commit
*during* the speculative window" question (it does, via the mirrored
`commit_clear_en` in the shadow copy — see rat.v's header) before writing
any RTL, rather than discovering it as a bug afterward the way the
Phase 1 ROB-lookup gap was found.

**Phase 3 found one real bug, in the dual-allocation "second free slot"
logic shared by `alu_rs.v`/`mul_rs.v`/`lsq.v`.** Two of the pre-existing
tests (`ooo_waw`, `ooo_jal_jalr`) started timing out. Tracing the ROB
showed it permanently full, stuck on a head tag whose `done_arr` bit
never set — and no reservation-station bank anywhere held an entry for
that tag at all. The instruction had a ROB entry and a RAT rename (the
top level believed dispatch had succeeded), but no functional unit had
ever actually received it: a genuine "ghost" instruction, permanently
undispatchable to anything that could ever mark it done.

Root cause: each dual-alloc bank's "find lane 1 a free slot distinct from
lane 0's" search unconditionally excluded lane 0's own picked index
(`free_idx`) from consideration for lane 1 — *even when lane 0 wasn't
actually targeting that bank at all* (e.g. lane 0 was a branch, using
`branch_rs`, while lane 1 alone wanted `alu_rs`). When that excluded slot
happened to be the *only* free slot in the bank, lane 1's search correctly
reported "no second slot available" and silently declined to allocate —
while the top level, whose own resource check only asked "does lane 0 use
this bank too, or does lane 1 alone just need one free slot" (correctly
answering "one is enough" here), had already gone ahead and committed
lane 1 to dispatching. Fixed by gating the exclusion on `alloc_req` (is
lane 0 *actually* going to consume `free_idx` this cycle), not just
`have_free` (is `free_idx` valid at all) — the same three-line fix in all
three banks. Found the same way the two Phase 1/2 bugs were: hand-tracing
ROB and reservation-station internal state from a stuck-full symptom back
to a missing entry, rather than guessing at the RTL from the black-box
timeout alone.

## Verification methodology

Same discipline as the single-cycle RV64I core: `asm64.py` (RV64I+M
mnemonics, reused unmodified, no fork needed), a `TestBuilder` /
`check_eq` / `x31=0xFFFF0000`+`ecall` pass convention, standalone
unit-testbenches for the riskiest new logic before wiring it into the
full pipeline (the divider, then ROB+RAT together), and — critically for
this core specifically — temporary hierarchical-reference debug
testbenches that trace exact cycle-by-cycle internal RS/ROB state rather
than guessing at RTL bugs from black-box symptoms alone. Both real bugs
in the "Bugs found" section above were isolated this way, not by
inspection.

## Phase 4: benchmarking dispatch width, honestly

All four phases from the original plan are now complete. Phase 4's brief
was a benchmark suite demonstrating real ILP gains from 2-wide dispatch.
The actual result, after real measurement rather than assumption, is more
interesting than a simple "here's the speedup" table: **on this core, as
scoped, 2-wide dispatch alone produces essentially no measurable
improvement in total program completion time** — and the reason why is a
genuine, worthwhile architectural lesson, not a bug.

### Setup

`riscv64_ooo_proc.v` gained an `ENABLE_DUAL_ISSUE` parameter (default 1)
that forces lane 1 to never fire when set to 0 — the *same* RTL becomes a
single-issue machine, with everything else (OoO execution, branch
prediction/speculation, ROB/RS sizing) held identical. This is a direct,
apples-to-apples comparison, not two diverging implementations that could
drift apart. `bench_ooo.py` builds several hand-assembled RV64I+M
programs and runs each one twice, once per setting.

### Results

| Benchmark | Dual-issue | Single-issue | Speedup |
|---|---|---|---|
| `bench_serial_chain` (16-deep dependency chain) | 52 | 52 | 1.00x |
| `bench_independent_ops` (16 fully independent adds) | 52 | 52 | 1.00x |
| `bench_reduction_unrolled4` (4-way unrolled reduction) | 80 | 80 | 1.00x |
| `bench_divide_overlap` (independent ops *after* a divide) | 128 | 128 | 1.00x |
| `bench_divide_after_independent` (independent ops *before* a divide) | 122 | 122 | 1.00x |
| `bench_mixed_bank_pairs` (alternating ALU/multiply) | 45 | 43 | 0.96x |

Every case lands at essentially 1.00x — including the ones specifically
designed to be favorable to dual-issue (fully independent work, a
loop-unrolled reduction with 4 separate accumulator chains, independent
work placed *before* a long-latency divide so faster dispatch could start
the divide sooner). Widening `alu_rs` from 4 to 16 entries (an
experiment, not part of the permanent suite) changed nothing either:
51 cycles either way.

### Why: the single CDB, not dispatch width, is the real bottleneck

Phase 3's design deliberately kept the CDB (one broadcast per cycle) and
commit (one retirement per cycle) exactly as they were in Phase 1/2 — see
"2-wide dispatch" above for why that scoping choice was made. That choice
has a direct consequence: for a stream of independent, single-cycle-
latency instructions, the *sustained* throughput of the whole pipeline —
from result broadcast through to commit — is capped at one instruction
per cycle no matter how many can be dispatched per cycle. Getting
instructions into the reservation stations faster doesn't help once
they're all waiting on the same one-at-a-time broadcast bus and the same
one-at-a-time retirement port. Widening the front end without widening
the back end just moves the queueing earlier in the pipeline; it doesn't
remove it.

The long-latency-divide benchmarks illustrate the same conclusion from
the other direction: single-issue dispatch was *already* fast enough to
get independent short instructions into flight well within a ~64-cycle
divide's latency window (this was the actual finding of Phase 1's
original `div_overlap` test, back when only single-issue existed) — so
there was no dispatch-side slack left for 2-wide dispatch to close.

This is a standard, well-known result in real superscalar design (a
wider front end needs a correspondingly wider execute/result/retire path
to pay off — see e.g. any real N-issue processor's per-cycle multiple
CDBs and multi-port register files) — but it's worth having actually
*measured* it here rather than assumed it, since the naive expectation
("wider dispatch → faster programs") is exactly wrong for this specific,
honestly-scoped design. Realizing genuine wall-clock ILP gains from
2-wide dispatch on this core would require widening the CDB and commit
path to match — a substantially larger undertaking than Phase 3's
front-end-only widening, and a natural next phase beyond the four
originally planned. See "Phase 5" below.

## Phase 5: widening the CDB and commit path

Phase 4 found that dispatch width alone doesn't help without a matching
execute/commit path. Phase 5 closes that gap: the CDB, ROB mark ports,
and commit are all now 2-wide, using the same two-instances-fed-
identical-inputs trick as the register-file duplication in Phase 3
(`register_file.v` gained a second write port, backward-compatible with
the single-cycle RV64I core's existing single-write usage — leaving the
new port unconnected is a safe no-op, the same extension pattern already
used for `rob.v`'s `extra_mark` ports).

- **CDB arbiter**: picks the two oldest-ready requesters per cycle via
  two passes of the same pairwise-oldest-first reduction — pass 1 finds
  the single oldest (structurally identical to the original 5-way
  arbiter), pass 2 re-runs the identical reduction with pass 1's winning
  bank masked out of contention.
- **Every reservation-station bank** (`alu_rs`, `mul_rs`, `div_rs`, `lsq`,
  `branch_rs`) now snoops two independent CDB buses instead of one.
- **`rob.v`** gained a second CDB-sourced mark port and a full second
  commit port (`head2`), retiring up to two entries per cycle. At most
  one of the two may be a store per cycle (`data_memory.v` still has a
  single write port) — head always has priority for that port; a second
  same-cycle committing store simply waits one more cycle.
- **`rat.v`** gained a second commit-clear port to match.

All 25/25 pre-existing tests continued to pass, with every cycle count
improving. Re-running the Phase 4 benchmark suite (unchanged) shows the
qualitative fix working: dispatch width now shows a genuine, if modest,
positive contribution (1.01x–1.10x, versus 0.90x–1.00x before). The
magnitude is smaller than a naive "now it's 2x" expectation because
`ENABLE_DUAL_ISSUE` only toggles dispatch width — the CDB/commit widening
applies unconditionally to *both* configurations being compared, so even
the "single-issue" baseline now benefits from a wider execute/commit
path. That's the correct comparison for isolating dispatch width's own
marginal contribution once it's no longer competing with a starved
execute/commit path for credit.

## Phase 6: data-level parallelism (vector)

Adds real RVV vector execution to the out-of-order core — the DLP item
from the original request, previously only present on the sibling
single-cycle core. Scoped deliberately narrower than "port the whole
RVV unit": `.vv`-form elementwise arithmetic/logic/min-max/shift/divide
only, single-issue vector dispatch (always lane 0, like branches), no
masking, no reductions, no `.vx`/`.vi` forms, and no vector load/store.

- **Reuses `vector_alu.v` and `vector_register_file.v` unchanged** —
  same "reuse the real execution logic" instinct as `alu_rs.v` reusing
  `execute.v`'s scalar ALU case statements.
- **`vec_rat.v`**: a vector RAT, structurally simpler than the scalar
  `rat.v` (single read/write port, no intra-group forwarding needed) but
  with the same checkpoint/restore machinery, since a vector instruction
  dispatched during a speculative window must roll back on misprediction
  exactly like any scalar one.
- **`vec_rs.v`**: a vector reservation station wrapping `vector_alu.v`,
  broadcasting on a *dedicated* `vec_mark` port on `rob.v` rather than the
  scalar CDB — vector operands in this scope only ever come from other
  vector instructions, so there's no cross-domain arbitration to unify.
- **`rob.v`** gained a parallel `vec_value_arr` (VLEN-wide) alongside the
  existing 64-bit `value_arr`, an `is_vec_dest` flag per entry, and a
  vector commit path that shares the *same* age-ordering, squash, and
  dual-commit machinery the scalar side already has (vector instructions
  still get an ordinary ROB tag from the unified tag space — this is what
  lets misprediction squash treat them exactly like any other in-flight
  instruction, with no extra cross-domain age comparison needed).

**A real gap, found immediately when writing the first test**: with only
`.vv`-form ops in scope, and no vector load/store, there was no way to
get non-zero data into a vector register at all (`0 op 0 = 0`, always) —
nor any way to read a vector result back out for `check_eq` to verify.
Fixed with two minimal, honest additions rather than silently expanding
scope:
- **`vmv.v.x`** (broadcast a scalar register into all vector lanes) — a
  real RVV instruction, not a shortcut invented for this project. It's
  the one place a vector-dest instruction reads a scalar operand; rather
  than teaching `vec_rs.v` to snoop two unrelated CDBs for this single
  bring-up instruction, dispatch simply stalls until the scalar source is
  already resolved (reusing the ordinary scalar `rs1`/RAT/CDB path
  unchanged), then feeds `vec_rs.v` an already-ready entry through its
  ordinary add-with-zero path — no new opcode needed in `vector_alu.v` at
  all. `asm64.py` doesn't have this mnemonic; tests emit it via `.word`
  with a small helper built on `asm64.py`'s own `v_type()`.
- **Vector register verification** via `tb_core_ooo.v`'s existing
  hierarchical-reference debug convention, made permanent: an
  unconditional `[VREGS]` dump (the same technique used for internal
  debugging throughout every phase of this project), since there's no
  extract/store instruction in scope to get a result into anything
  `check_eq` could otherwise see.

4/4 dedicated vector tests pass, including a dependency-chain test
(`vadd.vv` feeding a later `vmul.vv`) that specifically exercises
`vec_rs.v`'s own CDB snoop — an entry waiting on *another* vec_rs entry's
not-yet-broadcast result, not just operands already resolved at dispatch.
All 25 pre-existing tests continue to pass unmodified, since vector
instructions are entirely orthogonal to any program that never uses the
`OP_V` opcode.

## Phase 7: simultaneous multithreading

Adds 2-thread SMT — the TLP item from the original request. Originally
asked for as "2/4 threads," settled at exactly **2 threads, built with
the same rigor and test coverage as every earlier phase**, in exchange
for dropping 4: a fuller writeup of that scope tradeoff (and the cache/
MESI, redundancy items still queued behind it) belongs in project notes,
not here, but the short version is that 4 threads' issue-priority and
commit-arbitration surface didn't look tractable to get genuinely right
at this project's usual depth in the same pass.

- **Fully duplicated per-thread front end**: PC, 2-wide fetch, 2-wide
  decode, `rat.v`, dual-ported `register_file.v`, `rob.v`, `branch_rs.v`
  — each a complete, independent instance (`t0_*`/`t1_*` prefixes in
  `riscv64_ooo_proc.v`), not a `generate` loop, matching this project's
  existing preference for concrete, purpose-built instantiation over
  maximally generic abstraction (see `alu_rs.v`'s own header).
- **Shared, thread-tagged execution backend**: `alu_rs`, `mul_rs`,
  `div_rs`, `lsq` stay single shared instances — every entry now carries
  a 1-bit `tid` alongside its ROB tag. Issue priority moved from Phase
  1-6's plain oldest-ROB-tag-first (no longer meaningful once there are
  two independent ROBs' tag spaces) to **"is-own-thread-ROB-head-first,
  else fixed lowest-index"** — a thread's own head, once ready, always
  wins immediately, which is what actually matters for not stalling that
  thread's commit; ties otherwise break by a fixed, bounded-starvation
  index order, same risk profile Phase 3/4 already accepted for
  intra-bank ties.
- **Coarse-grained round-robin dispatch**: a single `active_thread`
  register toggles unconditionally every cycle; one thread's full 2-wide
  slate dispatches per cycle, never mixed-lane. This is a deliberate
  simplicity-over-utilization tradeoff, not an oversight — a thread
  stalled on, say, an outstanding JALR still only gets skipped on its own
  turn rather than yielding its turn to the other thread, which would
  claw back some throughput at the cost of a real arbiter. Almost the
  entire Phase 3/5/6 dispatch/operand-readiness block is reused
  *verbatim*, fed by "muxed" views of whichever thread is
  `active_thread` this cycle (both threads' front ends are always
  computed combinationally regardless of whose turn it is) and "demuxed"
  back out to the correct per-thread RAT/ROB/branch\_rs write ports.
- **CDB arbiter widened from 5-way to 6-way** (alu, thread 0's branch,
  thread 1's branch, mul, div, lsq — `branch_rs` is two independent
  per-thread instances now, not one shared requester) and switched from
  the hand-unrolled pairwise-reduction age() comparison to a procedural
  for-loop reduction using the same is-own-thread-head-first priority as
  the RS banks, for the identical reason.
- **Two independent per-thread squash ports** on every shared bank
  (`squash0_*`/`squash1_*`), not one port muxed by a `tid` selector —
  branch resolution is asynchronous to which thread is currently
  dispatching, so both threads' outstanding branches can resolve, and
  both mispredict, in the exact same cycle. A single muxed port could
  only ever squash one of them that cycle, leaving the other thread's
  wrong-path entries stale in a shared bank.
- **Cross-thread store-commit arbitration**: `data_memory.v` still has
  one write port. Each thread computes its own store-commit candidate
  exactly as Phase 5 already did (untouched per-thread logic); a
  fixed-priority arbiter (thread 0 always wins) then decides which
  thread's store, if any, actually uses the port. If thread 1 loses and
  its own *head* (not just head2) was the contended store, thread 1's
  entire commit is suppressed that cycle — head2 can never retire past a
  blocked head.
- **Vector stays thread-0-only**, an explicit and *structurally*
  enforced scope cut: thread 1's decode outputs are never read for
  `is_vec` at all (hard-tied to 0 in the active-thread mux), so a vector
  opcode in thread 1's own instruction stream simply never dispatches —
  the same safe-failure convention this project already uses for any
  unrecognized opcode, not a silent correctness gap.
- **BHT stays a single shared instance** (real SMT processors commonly
  share predictor state, and it needs no per-thread ROB-ordering
  guarantee the way RS-bank squash does): its one update port is fed by
  whichever thread resolves a conditional branch this cycle, thread 0
  winning if both resolve the same cycle — a second, documented,
  rare-but-real simplification in the same spirit as the dispatch
  round-robin's.
- **Cross-thread memory consistency is explicitly out of scope**:
  `lsq.v`'s store-vs-load disambiguation only reasons about same-thread
  entries — two threads deliberately sharing addresses is not guaranteed
  safe, the same category of scope cut as this project having no MMU/
  virtual memory at all.
- **Two independent instruction memories** (`IMEM_FILE0`/`IMEM_FILE1`,
  one per thread) so two genuinely independent programs can run
  concurrently, with a single shared data memory (`DMEM_FILE`) — the
  only arrangement consistent with the cross-thread-memory scope note
  above, which presumes a shared address space.

### Bugs found

Phase 7 found three real bugs — all three only became *visible* once two
threads' timing actually interleaved, though the third turned out to
predate Phase 7 entirely.

**1. Cross-thread ROB-tag collision on the shared CDB snoop path.**
ROB tags are only unique *within* a thread — two independent ROBs both
number their in-flight entries `0..DEPTH-1`. Every shared bank's
"is this broadcast the operand I'm waiting on" check
(`alu_rs.v`/`mul_rs.v`/`div_rs.v`/`lsq.v`'s CDB snoop, and the identical
same-cycle bypass check in `riscv64_ooo_proc.v`'s own dispatch logic,
and — once shared CDB access was accounted for — `branch_rs.v`'s
operand snoop too, even though it isn't itself a shared bank) originally
compared only the numeric tag, not `(tid, tag)` together. With
`ROB_DEPTH=8`, a thread-0 entry waiting on tag 3 and an unrelated
thread-1 producer also broadcasting tag 3 the same cycle is not a rare
corner case — it happens routinely, and the thread-0 entry would
silently capture the wrong thread's value as its own operand. Found via
a hand-written regression test built specifically to trigger it
(`t_smt_tag_collision_stress` — two threads dispatching the *same shape*
of independent-ALU-op program in lockstep, with different immediates so
a leaked cross-thread value produces a detectably wrong result), after
noticing thread 0 alone produced a wrong final register value in an
otherwise-correct short program. Fixed by adding `cdbA_tid`/`cdbB_tid`
ports everywhere a CDB is snooped, comparing the full `(tid, tag)` pair;
`branch_rs.v` additionally gained a compile-time `MY_TID` parameter
(0/1, one per instantiation) since it has no per-entry `tid` storage of
its own.

**2. `commit_req2`'s downstream effects weren't actually gated on head
also retiring — a bug present since Phase 5, only *exposed* by Phase
7's timing.** `rob.v`'s own internal retirement logic already correctly
requires `do_commit1` (head retiring) before it will act on
`commit_req2` (`do_commit2 = commit_req2 && head2_ready && do_commit1`)
— but every downstream top-level *use* of the `rob_commit_req2` wire
(register-file write, RAT commit-clear, and, after Phase 7, store/vector
commit) read it directly, without the same gate. `head2_ready` can
legitimately go true — marked done via its own CDB broadcast — cycles
before `head_ready` does; when it does, the top level was writing
head2's value into architectural state *every such cycle*, repeatedly,
regardless of whether head had actually retired yet. In Phase 1-6's
single-thread testing this was silently harmless (head2's own value
doesn't change while parked, and it does eventually retire for real once
head catches up, so an outside observer checking final values only ever
saw the correct end state). It stopped being harmless the instant a
program-order-*later* instruction's premature, repeated write could
land *before* an exact-retirement-time observation — precisely what
`tb_core_ooo.v`'s `ecall_halt`-triggered register read is. Found by hand-
tracing `commit_rf_write_en2`/`t0_rob_head2_tag` cycle-by-cycle against
`ecall_halt0`'s own firing cycle in a temporary debug testbench, the same
technique this project has used to find every non-trivial bug so far.
Fixed once, at the source, in `commit_req2`'s own definition
(`t0_rob_commit_req2`/`t1_rob_commit_req2` now `&&`-in `t0_rob_commit_req`/
`t1_rob_commit_req`) rather than patching every downstream use site.

**3. A genuine testbench race, not an RTL bug, in `tb_core_ooo.v`
itself.** `ecall_halt0`/`ecall_halt1` are combinational wires derived
from the *same clock edge's* NBA-updated ROB state. The original
(Phase 1-6) convention evaluated `if (ecall_halt) ... #1; <read
registers>` — settling with `#1` before reading registers, but *not*
before evaluating the `ecall_halt` condition itself, which is exactly as
racy for a combinational signal derived from another module's
just-updated registers. This never manifested with a single always
block driving everything; adding the second thread's parallel
`ecall_halt1` check changed Icarus's scheduling enough to expose it
(occasionally observing the wrong edge's value for `ecall_halt0`,
displaced by one cycle from the RTL's actual retirement). Fixed by
settling both `ecall_halt0` and `ecall_halt1` into `reg`s immediately
after a single `#1`, *before* either condition is ever tested.

### Tests

3 dedicated SMT tests, plus all 29 pre-existing tests continuing to pass
unmodified (thread 1 defaults to a trivial always-passing idle program —
see `build_tests_ooo.py`'s `_ensure_idle_thread_mem` — so every existing
single-thread-focused test still only needs to check thread 0's own
result):

- **`t_smt_independent_programs`**: two structurally unrelated programs
  (a multiply, and a summing loop) running concurrently, each verified
  against its own expected result — the basic Phase 7 claim.
- **`t_smt_tag_collision_stress`**: the direct regression test for bug
  #1 above — two threads dispatching the same-shaped independent-ALU-op
  program in lockstep, with different immediates, so ROB tags collide
  numerically many times over the run and a cross-thread leak would be
  immediately visible in the final values.
- **`t_smt_slow_thread_no_starvation`**: thread 0 runs a genuine
  ~64-cycle divide (a long-outstanding, ROB-head-blocking instruction);
  thread 1 runs a handful of independent short adds concurrently, sharing
  the same `alu_rs`/`mul_rs`/`div_rs`/`lsq`/CDB. Both must complete
  correctly well within a modest cycle budget — checks that
  is-own-thread-head-first issue priority never lets one thread's
  unrelated long-latency work starve the other's ready-and-waiting head.

32/32 total.

## Phase 8: multi-level cache + MESI coherency

Adds a real 2-core system: private per-core L1 data caches, a shared L2,
and genuine MESI coherency (M/E/S/I, snooping, dirty forwarding) wired
all the way into the OoO pipeline's actual load/store execution — not
just a standalone protocol simulation. Originally scoped as "2-core
L1+L2 with real MESI," the user explicitly chose the larger of two
integration options: full pipeline integration (reworking the LSQ's
load/store execution into a real, variable-latency, outstanding-request
protocol) over a standalone-but-not-wired-in subsystem.

This phase was built and verified in two stages, matching this project's
established discipline of testing new, bug-prone logic in isolation
before wiring it into the pipeline:

### Stage 1: the coherency subsystem itself

- **`l1_cache.v`**: a private per-core, direct-mapped, write-back data
  cache with a real 4-state MESI machine. Blocking — one outstanding
  request at a time, the same "real, multi-cycle, but not pipelined"
  scope `div_fu.v` already established. An eviction's old occupant (if
  dirty) is written back to L2 *before* the new line is ever requested,
  so the affected set is genuinely Invalid for the whole time a request
  is outstanding — this is what keeps a same-index snoop arriving during
  that window trivially correct without needing to reason about a new,
  not-yet-installed line being snoop-visible early.
- **`l2_cache.v`**: shared, inclusive, write-through to the backing
  memory, and the coherence *director* for exactly 2 L1s (fixed-priority
  arbitration, one transaction in flight at a time — a deliberately
  simple, appropriate choice for a 2-requester coherence point). Doesn't
  need to track M/E/S for its own copy, only a "valid" bit plus two
  presence bits (a directory, not a third MESI level) — see its own
  header for why the directory is a safe over-approximation, not exact
  bookkeeping, and the one case (an L2-internal replacement) where it
  must stay precise. Reuses `data_memory.v`'s existing RVV vector port,
  unmodified, as a natural whole-cache-line memory interface — Phase 6
  never puts real traffic on that port (vector load/store was explicitly
  out of scope there), so it was sitting unused.
- **`tb_cache_mesi.v`**: drives two `l1_cache.v` instances and one
  `l2_cache.v` directly (no OoO core involved), proving real MESI state
  transitions before any pipeline wiring: write-miss → M, read-miss with
  dirty-snoop-forwarding (M → S downgrade plus data forward through L2),
  a Shared-line write upgrading to M while invalidating the other core,
  re-forwarding a *newer* dirty value after invalidation, and both clean
  and dirty (writeback) eviction. 13/13 checks.

Two real bugs were found and fixed during this stage, both classic
same-cycle Verilog timing mistakes rather than protocol-design errors:
asserting a snoop request via a non-blocking assignment and then trying
to read its resulting *combinational* response in the same clocked
block (an NBA update isn't visible until the next cycle — fixed by
driving the snoop request outputs combinationally instead, off `l2_cache.v`'s
own FSM state directly); and a one-cycle race in the L1↔L2 handshake
where L2 could return to idle and re-sample a requester's request line
one edge before that requester's own clear of it became visible,
spuriously replaying the just-completed transaction (fixed with a
one-cycle cooldown state before L2 re-arms).

### Stage 2: full pipeline integration

- **`lsq.v`**: loads become a genuine outstanding-request operation —
  structurally the same category of thing `div_rs.v` already does for a
  multi-cycle divide (issue, wait for `l1_read_valid`, then broadcast on
  the CDB) — but with one wrinkle a divide never has: a load's ROB entry
  can be squashed *while the request is still outstanding at the cache*.
  If that happens, the eventual response is silently discarded
  (`outstanding_squashed`), never broadcast onto a tag that may since
  have been reused by an unrelated instruction.
- **A small store buffer**, also new in `lsq.v`: store commit can no
  longer be instant (acquiring a cache line can take many cycles), but
  ROB retirement itself must stay exactly as fast as Phase 1-7 — nothing
  about *architectural* commit should slow down just because memory got
  more realistic. A committing store is handed off into a 4-entry buffer
  (address already resolved by commit time) and the LSQ slot vacates
  immediately, exactly as before; the buffer drains into `l1_cache.v`
  asynchronously. A buffered entry, having already committed, can never
  be squashed. Younger loads additionally check the buffer (not just
  LSQ-resident stores) for an address match — every buffered entry is
  unconditionally older than any still-resident LSQ instruction, so this
  needs a plain address compare, no age logic. If the buffer is ever
  full, `store_buffer_full` withholds that commit for a cycle — the same
  kind of space-available backpressure Phase 5's dual-commit already
  uses for the single-store-port conflict.
- **`riscv64_ooo_proc.v`** gained a private `l1_cache_i` (shared across
  both SMT threads, distinguished by the `tid` each LSQ/store-buffer
  entry already carries) and lost its direct `data_memory.v` instance —
  a single core no longer owns, or can even reach, memory directly; new
  `l2_*`/`snoop_*` ports connect to an external `l2_cache.v`.
  `DMEM_FILE`/`DMEM_WORDS` are gone from this module's own parameter
  list accordingly.
- **`riscv64_ooo_proc_solo.v`** (new): pairs one core with a private
  `l2_cache.v` (core 1's side simply tied off) so every single-core
  testbench keeps the same simple, self-contained "just instantiate this
  one module" surface Phase 1-7 had.
- **`dual_core_riscv64_ooo.v`** (new): two real cores sharing one
  `l2_cache.v` — the genuine 2-core system.

### A real, latent bug found here — not introduced by Phase 8, only exposed by it

The dual-core producer/consumer test (below) initially hung: core 0
correctly wrote a data word and a flag word; core 1's polling loop
never observed the flag. Hand-tracing (the same technique that found
every non-trivial bug in this project) eventually showed the *cached
data itself* was correct — the bug was in `rat.v`, present since Phase 2,
and had simply never been exercised by any prior test.

`rat.v`'s checkpoint/restore mechanism (see its own header) mirrors
every commit-clear into a shadow copy during a speculative window, so
that a misprediction's restore doesn't resurrect an already-committed
register as "still busy." But when a register's own commit-clear lands
on the *exact same cycle* as a `checkpoint_save` (a real timing
coincidence, not a corner case invented to test the fix — it happened
naturally here because a stable, once-set comparison register's commit
happened to coincide with the next loop iteration's branch dispatching
its own checkpoint), the checkpoint's bulk copy — which reads the
*pre-edge*, not-yet-cleared live `busy[]` — overwrote the shadow's
mirrored clear, since the bulk copy comes later in program order.  The
checkpoint then permanently "remembered" that register as busy on a tag
whose producer had already committed; once a later misprediction
restored from that checkpoint, the register was wedged waiting on a tag
that real, unrelated instructions kept recycling and rebroadcasting —
so every read of it silently returned whatever *that* unrelated
instruction had just computed, forever. This is exactly why the
resolved branch value looked like a fragment of a totally unrelated
`li` sequence elsewhere in the program: it was one, read through a
stale tag.

No pre-Phase-8 test ever triggered this: Phase 1-6's memory access was
single-cycle, so the specific relative timing between a stable
register's commit and a later branch's own dispatch never lined up this
way, and no existing test has a *stable* (set-once, read-repeatedly
across many loop iterations) comparison operand in the first place —
every prior loop test re-produces both branch operands every iteration.
Fixed once, in `checkpoint_save`'s own copy loop: for any register a
same-cycle commit-clear targets, the checkpoint now captures "not busy"
directly, instead of trusting the (racing) mirror step alone.
`t_stable_operand_loop` (a minimal single-core reproduction of this
exact shape) was added to `build_tests_ooo.py` as a regression test,
though the underlying race is a genuine timing coincidence, not
something a fixed test program can force on every run with certainty.

### Tests

- **`tb_cache_mesi.v`**: 13/13, described above.
- **`t_stable_operand_loop`**: the RAT checkpoint-race regression test,
  described above. All 33 single-core tests (29 pre-existing + this one
  + `t_smt_*`... — see Phase 7's count) pass unmodified otherwise, since
  the cache/store-buffer timing change is invisible to correctness
  checks that only look at final values, not cycle counts.
- **`build_dual_core_tests.py`**'s **`t_producer_consumer`**: the real
  payoff test. Core 0 writes a data word, then a flag word, both through
  its own private L1 and the new store buffer. Core 1 busy-polls the
  flag through its own, entirely separate L1 until it observes the
  write — which can only happen if core 1's load-miss on the flag
  address genuinely triggers a coherency snoop of core 0's L1 (or a
  correctly-updated L2) and keeps re-checking (a fresh L1 miss/re-snoop
  each loop iteration, since a cached copy would otherwise never observe
  a later write) until the real MESI transaction actually delivers the
  new value — then checks it reads the *data* value core 0 actually
  wrote, not stale or zero memory. This is the genuine end-to-end proof
  that a value committed by one core becomes visible on the other only
  via a real cross-core coherency transaction, running through the full
  OoO pipeline, not a scripted protocol test.

## Phase 9: lockstep redundancy (DMR)

Cache/MESI (Phase 8) was about two cores *cooperating* through shared
memory. Redundancy is the opposite goal: two copies of the *same* core
running the *same* program from the *same* initial state, coupled to
each other in no way at all, so that one copy's fault can never
propagate into the other and corrupt the comparison itself. Scoped (via
an explicit tradeoff conversation) as "lockstep first" -- dual-modular
redundancy (DMR) with a continuous fault-detection comparator -- with
ECC on memory structures queued as a follow-on, not built yet.

- **`lockstep_dual_core.v`** (new): two independent
  `riscv64_ooo_proc_solo` instances -- core A (primary, whose outputs
  this module exposes) and core B (a pure checker) -- each with its own
  private L1+L2+memory, deliberately *not* sharing an L2 the way
  `dual_core_riscv64_ooo.v`'s cooperating cores do. A continuous
  comparator watches both threads' PC and `ecall_halt` every cycle;
  `lockstep_fault` is sticky (latches on first divergence, stays set
  until reset), matching how a real safety system's fault flag needs to
  persist for downstream handling rather than self-clear the instant a
  transient mismatch passes.
- **Fault-detection scope, deliberately narrower than "catch every
  possible fault"**: comparing PC/halt only, not the full architectural
  register file every cycle. Comparing everything would catch a fault
  the instant it's *computed*, including a "dead" wrong value that never
  again affects control flow -- but at the cost of a much wider per-cycle
  comparator (every register, both write ports, both threads). PC/halt
  comparison is far cheaper and still catches the overwhelming majority
  of realistic faults, since almost any corrupted value eventually
  reaches *some* branch condition or the final halt outcome -- the same
  detection-latency-vs-comparator-cost tradeoff real lockstep systems
  have to make too, not something unique to this scope.
- **`tb_lockstep.v` / `build_lockstep_tests.py`** (new): a
  `FAULT_INJECT_CYCLE` parameter drives a one-time, testbench-only
  hierarchical corruption of one bit of core B's own architectural x7 at
  a chosen cycle -- a simulated soft-error single-event upset in one of
  the two redundant copies, not an RTL "please corrupt yourself" port (a
  synthesizable version of that would be a strange thing for a real
  redundant core to have). Two tests: `lockstep_baseline` (no fault
  injected, `lockstep_fault` must stay 0 -- the "redundancy doesn't cry
  wolf" check) and `lockstep_fault_detected` (`lockstep_fault` must latch
  to 1).

### Getting fault injection right took three iterations

The end goal -- corrupt one bit in one redundant copy, prove the other
copy's comparator catches it -- turned out to have more failure modes
than expected, each only visible by actually running it and tracing
cycle-by-cycle when it silently didn't work:

1. **Forcing a `wire`, not a `reg`**: the first attempt used Verilog
   `force`/`release` on `core_b`'s PC *output port* (a
   continuously-driven wire). `lockstep_fault` never fired. `force` on a
   wire only overrides what that wire displays; the moment `release`
   executes, it snaps back to whatever its actual driver (the untouched
   internal PC register) was already producing -- the "corruption" never
   touched real state, so there was nothing left to observe by the next
   clock edge.
2. **`force`/`release` on an indexed array word isn't supported by
   Icarus**: switching the target to `registers[5]` (an actual `reg`,
   inside `register_file.v`'s architectural register array) seemed like
   the fix, but `iverilog`'s code generator rejected it outright:
   `cannot %force/vec4 to the word of a variable array`. More
   fundamentally, `force` was the wrong tool for the job anyway -- a
   soft-error bit-flip is a one-time event, not a sustained drive, and
   `force`-then-`release` a mere `#1` later has the same
   collapses-before-the-next-edge problem as case 1, just for a
   different reason (NBA-scheduled real writes to the same array word
   would still be racing a held force). Switched to a single plain
   hierarchical procedural assignment instead -- a real one-time write,
   which is both legal Icarus syntax and the semantically correct model
   of a single-event upset.
3. **Which register, and when**: the test program's `li x7, N` (its loop
   bound) was initially left *inside* the loop body, redefined every
   iteration -- so corrupting its committed copy did nothing, since
   in-flight consumers were overwhelmingly reading it via CDB/tag
   forwarding from the RS, not via a fresh read of the (corrupted)
   architectural regfile, and any residual corruption got overwritten by
   the very next iteration's own `li` anyway. Moving `li x7, N` outside
   the loop (set once) fixed the forwarding-bypass problem, but exposed a
   second one: the fixed injection cycle originally chosen landed
   *before* `x7`'s own defining instruction had even committed, so its
   real, correct write simply overwrote the injected corruption moments
   later. Cycle-by-cycle tracing (`$display` of both cores' x7 every
   cycle around the injection point) pinned down a cycle safely after
   that commit, which finally produced a real, sustained divergence.

The general lesson, now documented in `tb_lockstep.v`'s own header:
target a register that (a) is a genuine `reg`, not a derived wire, (b) is
written once and then stable -- not something every loop iteration
redefines, since tight RAW chains route around the committed regfile via
forwarding -- and (c) is corrupted *after* its own real defining write
has already committed, not before.

### Tests

- **`build_lockstep_tests.py`**: 2/2 -- `lockstep_baseline` (no injected
  fault, `lockstep_fault` stays 0 through a full correct run) and
  `lockstep_fault_detected` (a mid-run bit-flip to core B's x7 at cycle
  60, `lockstep_fault` latches to 1). Both run through the full OoO
  pipeline (real dispatch/rename/execute/commit in each copy), not a
  simplified comparator-only model.

## Phase 9 continued: ECC on memory structures

Lockstep DMR (above) catches *any* divergence between two whole redundant
cores, at the cost of a second full core. ECC is the cheaper, narrower
complement: real SECDED (single-error-correct, double-error-detect)
protection on the individual storage arrays most exposed to a soft-error
bit-flip, catching and silently fixing the common single-bit case without
needing the other core's help at all. Scoped, per the user's original
"both, lockstep first" direction, as the widest of the offered options:
the architectural register file, the L1 data cache, the L2 data cache,
and the ROB's payload storage.

- **`ecc64.v`** (new): the one shared primitive everything else builds
  on -- a classic Hamming SECDED(72,64) code (7 real Hamming parity bits
  covering a 71-bit virtual codeword, plus 1 overall parity bit across
  that codeword, giving 8 check bits total for 64 data bits). Always
  computes both directions combinationally (`wr_data -> wr_check` and
  `rd_data,rd_check -> rd_data_corrected,rd_sbe,rd_dbe`); a caller wires
  up whichever half a given instantiation needs. The bit-position math
  (which data bits fall under which parity group) is computed once, via
  a `function` walking the 71-position codeword and skipping every
  power-of-two (parity) position -- not a hand-written lookup table.
- **`ecc_line.v`** (new): a thin generate-loop bundle of `ecc64.v`
  instances, one per 64-bit word in a wider cache line (4 words for this
  project's 32-byte lines) -- independently-corrected 64-bit words, not
  one wider code across the whole line, since that would only ever
  correct one bit *line-wide* instead of one bit *per word*.
- **`ecc_register_file.v`** (new): drop-in replacement for the sibling
  RV64I core's `register_file.v` (identical port list/semantics), used
  only within `RV64I_OOO` -- the single-cycle core's own copy is never
  touched, the same standing rule this project has followed for every
  prior phase that reuses RV64I/src unchanged. All 4 architectural
  register-file instances in `riscv64_ooo_proc.v` (2 per thread, the
  existing "extra read port via a second full instance" pattern) were
  switched over.
- **`l1_cache.v`**: `line[]` (the cache data array) gained a parallel
  `line_check[]`; every access point (CPU read-hit, CPU write-hit merge,
  the store-buffer-drain UPGR merge, and the coherence snoop response)
  now decodes-and-corrects through its own `ecc_line.v` instance before
  the corrected value is used, and re-encodes fresh check bits on every
  write. `tag[]`/`state[]` are deliberately *not* protected -- see the
  scope note below.
- **`l2_cache.v`**: same idea for `l2_data[]`, simpler in one respect
  (only ever one transaction in flight, so only one `req_idx`-keyed
  `ecc_line.v` instance is needed) since L2 never merges a partial write
  the way L1's CPU-facing side does -- every L2 write (a requester's own
  dirty writeback, a memory fetch, or an absorbed dirty snoop-forward)
  replaces the whole line outright.
- **`rob.v`**: `value_arr[]` (the scalar payload that becomes committed
  architectural register state) gained a parallel `value_check[]`.
  Correction is applied on *every* read port, including the 4 dispatch-
  time operand lookups (a consumer capturing a bad value straight out of
  the ROB would be a real, if quieter, failure mode than a corrupted
  commit) -- but fault *counting* toward `ecc_rob_sbe_fault`/
  `dbe_fault` is scoped to just the commit-time (`head`/`head2`) reads,
  gated on `head_ready`/`head2_ready`, since those are the only reads
  with an equally clean "is this meaningful right now" signal to gate
  on (see the module's own comment for why the lookup ports don't have
  one). `vec_value_arr[]` is out of scope, matching vector's
  already-narrower, thread-0-only footprint everywhere else in this
  project.

### Scope: data arrays, not control metadata

Every structure above protects only its bulk *data* payload -- L1/L2's
line contents, the register file's/ROB's stored values -- not the small
per-entry control fields alongside them (`tag[]`/`state[]` in the
caches, `valid_arr`/`done_arr`/`rd_arr`/etc. in the ROB). A corrupted
control bit is a different failure mode (misdirecting a hit/miss
decision or a commit's own bookkeeping, rather than silently handing
back wrong data) and was deliberately left out of this pass, the same
kind of explicit scope line this project has drawn before (e.g. Phase 9
lockstep comparing only PC/halt, not full architectural state, every
cycle).

### Two real timing bugs found integrating the fault-flag outputs

Both structures' *data correction* worked on the first pass (verified by
`tb_ecc_line.v` and friends in isolation); it was specifically the
`sbe_fault`/`dbe_fault` *output* signals -- observability, not
correctness of the corrected data itself -- that needed a second pass
once wired into `l1_cache.v` and `l2_cache.v`'s real multi-cycle
request/response protocols:

- **`l1_cache.v`**: gating the fault flags on the live request signal
  (`cpu_read_req && rd_hit && ...`) meant the flag was only true during
  the exact cycle the *request* was asserted -- but `cpu_read_req` is
  already back to 0 well before the *response* (`cpu_read_valid`)
  actually pulses several cycles later, which is the natural point any
  real caller (including `tb_ecc_l1.v` itself, on its first attempt)
  would check it. Found immediately by the test reporting `sbe=0` on a
  real, injected corruption. Fixed by latching `access_sbe`/`access_dbe`
  into registers at the same cycle the FSM actually decides to consume a
  corrected value, so they're already stable by the next cycle's DONE
  state.
- **`l2_cache.v`**: a subtler version of the same class of bug --
  `c0_resp_valid`/`c1_resp_valid` are themselves `<=`-assigned *inside*
  the `ST_RESPOND` state's own body, so they only become visible the
  cycle *after* `fsm` actually holds `ST_RESPOND` (once it's already
  moved on to `ST_COOLDOWN`). A combinational `fsm == ST_RESPOND` gate is
  therefore one cycle early relative to when `resp_valid` is actually
  observed. Same fix: latch `access_sbe`/`access_dbe` in the same
  `ST_RESPOND` body, alongside `c0_resp_valid`/`c1_resp_valid`
  themselves, so both become visible on the same next cycle.

The ROB's read ports needed no such fix: `head_value`/`lookup*_value`
are plain continuous combinational reads (like `ecc_register_file.v`'s
own read ports), not a pulsed request/response protocol, so gating
directly on `head_ready`/`head2_ready` was correct the first time.

### A real interaction with Phase 9's other half (lockstep)

Once the register file was ECC-protected, `tb_lockstep.v`'s existing
`lockstep_fault_detected` test (a single-bit flip of core B's x7)
started failing -- `lockstep_fault` stopped firing. Not a regression:
`ecc_register_file.v` now transparently corrects exactly that kind of
single-bit corruption before it can ever reach a branch and diverge
control flow, which is the *correct*, intended layering of the two
mechanisms -- ECC silently heals the common single-bit case; lockstep is
the backstop for whatever ECC structurally cannot fix. Fixed by changing
the injected fault to flip *two* bits instead of one: an uncorrectable
(dbe) error that `ecc_register_file.v` passes through unmodified (with a
fault flag raised, but no attempted correction), so it still reaches the
pipeline wrong and still diverges control flow the way the original
single-bit test intended. Documented inline in both `tb_lockstep.v` and
`build_lockstep_tests.py`.

### Tests

- **`tb_ecc64.v`**: 5 representative 64-bit data patterns x (1 clean +
  all 72 single-bit-flip positions + 20 sampled double-bit-flip
  combinations) -- exhaustive per-position coverage of the core SECDED
  primitive, not spot checks, since off-by-one bit-position math is
  exactly where "looks right" and "is right" diverge.
- **`tb_ecc_line.v`**: clean round-trip, one single-bit flip in each of
  the 4 words of a 256-bit line in turn (proving *per-word* correction),
  and a double-bit flip confined to one word (proving the aggregate
  `rd_dbe` is correctly driven by that one word, not a bundling bug that
  always reports it).
- **`tb_ecc_register_file.v`**: clean write/read through both ports, x0
  always reads zero and never faults, single-bit corruption of a
  register's data *and*, separately, its check bits (both must resolve
  identically), and a double-bit corruption reporting `dbe_fault`.
- **`tb_ecc_l1.v`** / **`tb_ecc_l2.v`**: a real write-miss/BusRd-miss
  fills a line through the genuine MESI/memory-fetch FSM path (not a
  hand-poked initial value), then a hierarchical procedural corruption
  (same one-time-write technique as `tb_lockstep.v`, and for the same
  reason -- Icarus can't `force` an indexed word of a variable array
  either) proves transparent correction plus `sbe_fault`, and a second,
  double-bit corruption proves `dbe_fault` with the data left
  uncorrected.
- **`tb_ecc_rob.v`**: a real `mark_valid` broadcast produces real check
  bits (not hand-poked ones), then the same single-bit/double-bit
  corruption pattern against `value_arr[0]`, plus confirming
  `head_ready` (and therefore the fault flags) correctly goes quiet once
  the entry actually commits.
- **Zero regressions**: the full pre-existing suite -- 33/33 single-core,
  the dual-core coherency test, 13/13 isolated MESI checks, 21/21
  isolated ROB+RAT checks, both lockstep DMR checks, and all 6
  benchmarks (identical cycle counts to before this phase) -- passes
  unchanged with every one of these structures now ECC-wrapped, since
  none of this phase's work is externally observable behavior change
  absent an actual injected fault.

33/33 single-core, plus the dual-core coherency test, plus 13/13
isolated MESI checks, plus 21/21 isolated ROB+RAT checks, plus 2/2
lockstep DMR checks, plus 6/6 isolated ECC checks (`ecc64`, `ecc_line`,
register file, L1, L2, ROB).
