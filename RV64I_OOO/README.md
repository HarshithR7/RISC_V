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

**Phases 1 through 4 are all complete: 25/25 full-pipeline tests pass**,
plus 21/21 isolated ROB+RAT unit-test checks and 15/15 standalone divider
unit tests.

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
front-end-only widening, and a natural (if not yet started) next phase
beyond the four originally planned.
