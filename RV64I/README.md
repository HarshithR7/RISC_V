# RV64IMACFD — 64-bit RISC-V Core

A single-cycle RV64IMACFD core, built as Phase 0 of a longer-term plan
(RV64I → M → A → C → F → D → RVV 1.0), sharing the same architecture and
verification approach as the sibling [RV32I core](../README.md) one
directory up — same asynchronous-fetch single-cycle datapath, same
self-checking testbench pattern, same "assemble it with a small Python
assembler, actually run it in a real simulator, fix what the simulator
finds" discipline. Read `../README.md` first for the datapath rationale and
the bug catalog that motivated some of this core's design choices (e.g. why
instruction fetch is combinational, not registered).

## Status

**RV64IMACFD + a scoped RVV 1.0, fully implemented and verified: 58/58
tests pass** (11 RV64I + 5 M + 6 A + 6 C + 6 F + 8 D + 6 V Tier 1 + 5 V
Tier 2 + 4 V divide/remainder/shift + 1 configurable-vector-width proof),
plus 45/45 compressed-encoder round-trip checks and a standalone 28/28
FPU unit test (both precisions). Every planned extension from the
original sequencing (I → M → A → C → F → D → RVV) is now implemented;
RVV covers elementwise arithmetic, masking (`v0.t`), compares, min/max,
reductions, divide/remainder, and shifts, with the full spec still
intentionally scoped down (no permutation instructions, widening/
narrowing, or vector floating point) — see the RVV section below and
[What's not covered](#whats-not-covered) for exactly what that means.

Beyond hand-written assembly: a real, unmodified GCC (xPack 15.2.0)
autovectorizing a plain C loop produces machine code that runs correctly
on this simulated hardware, verified by splicing GCC's actual compiled
output into the simulator (not just RVV assembly that resembles compiler
output) — see [Real compiler integration and performance
benchmarks](#real-compiler-integration-and-performance-benchmarks). A
13-benchmark scalar-vs-vector suite (Level 1 elementwise ops, Level 2
dot product/AXPY/reduction, Level 3 matvec/matmul/convolution/image
filter) measures real speedups of 1.09–2.12x and real memory-bandwidth
ratios up to 2.93x, all frequency-independent (bytes/cycle) since no
synthesized Fmax exists yet to convert that to bytes/second honestly.
The vector datapath's width (`LANES`, default 4 → `VLEN=128`) is a real
synthesis-time parameter, not a hardcoded constant — proven by
instantiating and running the core with `LANES=8` end-to-end — see
[Configurable vector width](#configurable-vector-width). Real Yosys
synthesis (not estimated) puts the synthesizable core (F/D excluded — see
below for why) at **52,126 LUTs, 19,108 carry cells, 6,212 flip-flops**
on Lattice iCE40 — larger than any real iCE40 device, driven mostly by
five independent combinational hardware dividers — see [Area and
frequency](#area-and-frequency) for the full breakdown and what a real
deployment would do differently.

## Architecture

Same single-cycle shape as the RV32I core, widened to 64 bits throughout
(registers, ALU, PC, addresses), with instructions still 32 bits wide
(RV64I doesn't change instruction encoding size):

| File (`src/`) | Module | Role |
|---|---|---|
| `program_counter.v` | `program_counter` | 64-bit synchronous PC |
| `instruction_fetch.v` | `instruction_fetch` | Async halfword-addressable instruction ROM (RV64C: 2-byte, not 4-byte, alignment) |
| `compressed_decoder.v` | `compressed_decoder` | Expands RV64C 16-bit instructions into their exact 32-bit equivalent |
| `decode_control_unit.v` | `decode_control_unit` | Combinational decode + control, RV64I + M + A + F + D |
| `register_file.v` | `register_file` | 32×64-bit register file, x0 hardwired to zero |
| `execute.v` | `execute` | ALU (64-bit and word-truncated), M-extension mul/div, branch/jump target calc |
| `data_memory.v` | `data_memory` | 64-bit-wide byte/half/word/doubleword memory |
| `fp_register_file.v` | `fp_register_file` | 32×64-bit FP register file (f0-f31), NaN-boxed, no hardwired-zero register |
| `fpu.v` | `fpu` | Single- and double-precision arithmetic/compare/convert/classify/sign-injection |
| `vector_register_file.v` | `vector_register_file` | 32×128-bit vector register file (v0-v31) |
| `vector_alu.v` | `vector_alu` | Elementwise vector arithmetic, 4×32-bit lanes |
| `riscv64_proc.v` | `riscv64_processor` | Top-level wiring; A-extension reservation register, AMO ALU, F-extension register-file routing, and the RVV `vl` state register all live here |

### RV64I additions over the RV32I core

- **Widened everything**: 64-bit registers, ALU, PC, addresses. Immediates
  sign-extend to 64 bits (was 32).
- **`LD`/`SD`/`LWU`**: doubleword load/store, and zero-extending word load
  (as distinct from `LW`'s sign-extending word load — this is the load
  width RV64I actually needs that RV32I has no equivalent of).
- **6-bit shift amounts**: `SLLI`/`SRLI`/`SRAI` shift 0–63 positions
  (RV32I's 5-bit shamt only reaches 31), decoded from `instruction[25:20]`
  with `instruction[31:26]` as the SLL-vs-SRA discriminator (one bit
  narrower than RV32I's 7-bit `funct7`, since the shamt field grew by one
  bit).
- **`OP-IMM-32`/`OP-32` opcodes** (`ADDIW`/`SLLIW`/`SRLIW`/`SRAIW` and
  `ADDW`/`SUBW`/`SLLW`/`SRLW`/`SRAW`): compute on the low 32 bits of the
  operands and sign-extend the 32-bit result to 64. Implemented in
  `execute.v` via a `word_op` control flag: the execute stage always
  computes both a 64-bit and a 32-bit-truncated ALU result in parallel, and
  a mux picks the (sign-extended) 32-bit one when `word_op` is set. This is
  the single most important semantic difference from a plain 64-bit op —
  see the `word_ops` test's `addw`/`sllw` cases for why: a 32-bit overflow
  must wrap *within the word* and then sign-extend, which is a different
  result than doing the same arithmetic at full 64-bit width and just
  reading the low 32 bits back out.
- **`ecall_halt`** and `IMEM_FILE`/`DMEM_FILE` parameterization carried over
  unchanged from the RV32I core's design.

### M extension (multiply/divide)

`MUL`/`MULH`/`MULHSU`/`MULHU`/`DIV`/`DIVU`/`REM`/`REMU` (opcode `R_TYPE`,
`funct7 = 0000001`) plus `MULW`/`DIVW`/`DIVUW`/`REMW`/`REMUW` (opcode
`OP_32`, same `funct7`, no `MULHW` family — the spec doesn't define narrow
multiply-high variants). `func3` already matches the M-extension's own
`MUL`..`REMU` ordering 1:1, so decode just passes it through as `muldiv_op`
rather than needing a translation table.

Implementation notes (`execute.v`):
- **Multiply**: sign/zero-extend both 64-bit operands to 128 bits per the
  needed interpretation (signed, unsigned, or mixed for `MULHSU`), then a
  single 128×128 signed multiply gives the correct bit pattern for all
  three cases — no separate signed/unsigned multiplier hardware needed,
  just three different extensions feeding one operation.
- **Divide/remainder**: implements the RISC-V-mandated non-trapping special
  cases explicitly, since these are exactly the corner cases a naive `/`/`%`
  would get wrong: division by zero (`DIV`/`DIVU` → all-ones, `REM`/`REMU`
  → the dividend, *not* a trap), and signed overflow (`MIN_INT / -1` →
  `MIN_INT`, its remainder → 0). `REM`'s sign-of-dividend truncating
  semantics fall out for free from Verilog's `%` operator on signed
  operands, which already matches C-style truncating remainder.
- **`MULW`'s low 32 bits are free**: the low N bits of a two's-complement
  product depend only on the low N bits of the operands, so `MULW` reuses
  the already-computed 64-bit signed product's low 32 bits rather than
  running a second 32×32 multiply.

### A extension (atomics)

`LR.W`/`LR.D`, `SC.W`/`SC.D`, and `AMOSWAP`/`AMOADD`/`AMOXOR`/`AMOAND`/
`AMOOR`/`AMOMIN`/`AMOMAX`/`AMOMINU`/`AMOMAXU` in word and doubleword form
(opcode `AMO = 0101111`, `funct5` in `instruction[31:27]` selects the
operation, `func3` selects word vs. doubleword using the *same* encoding
`LW`/`LD` already use — so `data_memory`'s existing width/sign-extension
logic handles AMO's memory access with zero changes to `data_memory.v`).

This core is single-cycle and single-hart — only one instruction is ever
"in flight" — so an ordinary AMO read-modify-write is atomic by
construction: no locking is needed, just "read the old value, compute the
new one combinationally, write it back on the same clock edge the old value
is latched into `rd`." That combinational new-value computation
(`amo_write_data` in `riscv64_proc.v`) has to live at the top level rather
than inside `execute.v`, since it needs `output_mem_read` (data memory's
read result), which isn't available until *after* `execute` has already
produced the address that fed that read — a genuine sequencing dependency,
not just a convenient wiring choice.

`LR`/`SC` need real state, though, since `SC`'s success depends on whether
anything wrote to the reserved address between the `LR` and this `SC` —
information that doesn't exist within a single instruction's combinational
logic. A `reservation_valid` + `reservation_addr` register pair
(`riscv64_proc.v`) is set by `LR`, invalidated by `SC` (regardless of
outcome, per spec) or by *any* store instruction (deliberately more
aggressive than the spec strictly requires, which is explicitly permitted
and much simpler than tracking exact address overlap). `SC` succeeds only
if the reservation is still valid *and* its address matches; `rd` is
synthesized as 0 (success) or 1 (failure) — this is the one case in the
whole core where a register's write-back value isn't the ALU result or a
loaded memory value, so it gets its own override in the top-level
write-back mux.

Min/max needed one more piece of care: for the `.W` forms, RV64A doesn't
require `rs2` to already be validly sign-extended, so the comparison must
explicitly reinterpret the low 32 bits of both operands (signed for
`MIN`/`MAX`, zero-extended for `MINU`/`MAXU`) rather than comparing the
full 64-bit register contents directly — the earlier "low bits are
invariant regardless of extension" argument that simplified `ADD`/`XOR`
etc. does *not* apply to comparisons, since sign vs. zero extension changes
which operand compares smaller.

### C extension (compressed instructions)

The full mandatory RV64C set: `C.ADDI4SPN`, `C.LW`/`C.LD`/`C.SW`/`C.SD`,
`C.NOP`/`C.ADDI`/`C.ADDIW`/`C.LI`/`C.ADDI16SP`/`C.LUI`, `C.SRLI`/`C.SRAI`/
`C.ANDI`/`C.SUB`/`C.XOR`/`C.OR`/`C.AND`/`C.SUBW`/`C.ADDW`, `C.J`/`C.BEQZ`/
`C.BNEZ`, `C.SLLI`/`C.LWSP`/`C.LDSP`/`C.JR`/`C.MV`/`C.EBREAK`/`C.JALR`/
`C.ADD`/`C.SWSP`/`C.SDSP` — 33 instructions (compressed floating-point
loads/stores are out of scope until F/D exist).

This is the one extension so far that couldn't be bolted on by adding
opcodes to the existing decode/execute stages, because it breaks an
assumption baked into every other part of the core: **instructions are no
longer 4 bytes, or even a fixed size.** A compressed instruction is 2
bytes, so the instruction stream is only guaranteed to be 2-byte aligned —
a 32-bit instruction can start at a non-4-byte-aligned address. The fix
touches three things:

1. **`instruction_fetch.v` is now halfword-addressable.** Memory is a
   `reg [15:0]` array instead of `reg [31:0]`, and every fetch reads *two*
   consecutive halfwords — the one at the current PC and the next one —
   regardless of whether the current instruction turns out to be
   compressed. This is what makes an unaligned 32-bit instruction "just
   work" in one cycle: the two halfwords it's split across are exactly the
   ones fetched, with no separate re-fetch or stall needed.
2. **`compressed_decoder.v` sits between fetch and everything else.** It
   looks at the low 2 bits of the fetched halfword (`11` = a real 32-bit
   instruction, anything else = compressed) and either passes the 32-bit
   fetch through unchanged or expands the compressed instruction into its
   *exact* 32-bit RV64I/M equivalent. This is the key design choice: by
   producing a bit-accurate real 32-bit instruction word,
   `decode_control_unit`/`execute`/`data_memory` need zero awareness that
   compression exists — they only ever see instructions they already knew
   how to handle. Each of the 33 expansions is implemented by literally
   inverting the encoding tables in the RISC-V spec's C-extension chapter,
   using small `mk_r`/`mk_i`/`mk_s`/`mk_b`/`mk_u`/`mk_j` packing functions
   that mirror the RV32I assembler's own encoders.
3. **`execute.v` needs to know whether the current instruction was
   compressed**, purely to get PC arithmetic right: the default
   fall-through increment is `pc+2` instead of `pc+4`, and a compressed
   `JAL`/`JALR` (i.e. `C.JALR`/`C.JR` after expansion) must write back
   `pc+2` as its return address, not `pc+4`.

Everything else — branch/jump targets, the write-back mux, `ecall_halt`
detection — operates on the *expanded* instruction and needed no changes,
which is the payoff of expanding early rather than teaching every stage
about two instruction widths.

### F extension (single-precision floating point)

The full RV64F set (30 instructions): `FLW`/`FSW`; `FADD.S`/`FSUB.S`/
`FMUL.S`/`FDIV.S`/`FSQRT.S`; `FMADD.S`/`FMSUB.S`/`FNMSUB.S`/`FNMADD.S`
(fused multiply-add — a *new* instruction format, R4-type, with four
register fields instead of the usual three); `FSGNJ.S`/`FSGNJN.S`/
`FSGNJX.S`; `FMIN.S`/`FMAX.S`; `FEQ.S`/`FLT.S`/`FLE.S`; `FCLASS.S`;
`FCVT.W.S`/`FCVT.WU.S`/`FCVT.L.S`/`FCVT.LU.S` (float→int) and their
inverses `FCVT.S.W`/`FCVT.S.WU`/`FCVT.S.L`/`FCVT.S.LU`; `FMV.X.W`/
`FMV.W.X` (raw bit moves, no conversion).

**Scope decision, stated up front: only round-to-nearest-even (RNE) is
implemented.** RV64F's `rm` field (and the `fcsr`/`frm`/`fflags` CSRs it
can dynamically reference) is decoded but ignored; every operation always
rounds RNE, and no exception flags are recorded (there's no CSR file at
all yet — that's its own substantial feature this core doesn't have).
This is a real, deliberate scope cut: implementing all five IEEE-754
rounding modes correctly, plus flag accumulation, would have roughly
doubled the size of this extension for something the overwhelming
majority of compiled code never touches (RNE is the near-universal
default). It's the same category of tradeoff as A's `aq`/`rl` bits —
documented rather than silently dropped.

**How the arithmetic actually works.** Verilog gives simulation-only
access to a native double-precision `real` type (`$bitstoreal`/
`$realtobits`), but this Icarus build doesn't support `shortreal`/
`$bitstoshortreal` (checked empirically before committing to an
approach). So `fpu.v` does the arithmetic by exactly widening a float32
bit pattern to a float64 one (`f32_to_f64`, lossless — every float32 value
has an exact float64 representation), computing with Verilog's native
`+`/`-`/`*`/`/`/`**0.5` operators on `real`, and rounding the float64
result back down to float32 (`f64_to_f32`, the only place where actual
rounding happens). This is mathematically sound for RNE: float64's
52-bit mantissa has 29 more bits than float32's 23-bit mantissa, which is
enough headroom that computing in the wider format and rounding *once*
gives the correctly-rounded float32 result for `+`,`-`,`*`,`/`,`sqrt` —
the same principle real hardware FPUs rely on internally. Comparison,
classification, sign-injection, and raw bit moves involve no rounding at
all and are implemented as exact bit manipulation.

Both conversion functions handle normals, subnormals, zero, infinity, and
NaN explicitly. Normal-range rounding (including the mantissa-overflow
carry into the exponent) is implemented and tested; subnormal rounding is
best-effort and has *not* been exhaustively verified across the whole
subnormal domain the way the normal-range path has — flagged here rather
than implied to be bit-exact everywhere.

**Integer↔float conversions don't go through `real` at all.**
`FCVT.L.S`/`FCVT.LU.S`/`FCVT.S.L`/`FCVT.S.LU` need to move values in and
out of 64-bit integers, and `$itor`/`$rtoi` turned out to be a trap here:
checked empirically (see "bugs worth knowing about" below), they silently
truncate to 32 bits in this Icarus build. `f32_to_int`/`int_to_f32`
instead work directly on the bit-level float32 representation (extract
sign/exponent/mantissa, shift into position, round or truncate), with the
RISC-V-mandated saturating behavior for out-of-range values (NaN and
overflow → the target type's max value; `-∞`/very-negative → min value or
0 for an unsigned target — not a trap). `f32_to_int` truncates toward
zero rather than rounding to nearest for the fractional bits being
discarded; this is a second, smaller documented simplification in the
same spirit as the RNE-only decision above.

**Register file and routing.** `fp_register_file.v` is 32×64-bit, already
wide enough for D (unlike the integer file, F has no hardwired-zero
register — f0 is ordinary). Single-precision values are NaN-boxed (upper
32 bits all 1s) per spec, which is also what makes this register file
already D-ready without changes if D gets added later. Since `rs1`/`rs2`
can refer to either the integer or FP register file depending on the
instruction (`FLW`'s address register is integer, its destination is FP;
`FCVT.S.W`'s source is integer, destination is FP; `FEQ.S`'s operands are
FP, destination is integer, etc.), decode emits explicit `rs1_is_fp`/
`rs2_is_fp`/`rd_is_fp` signals; both register files are always read in
parallel using the same address bits, and the top level just muxes which
result actually feeds downstream and which file's write-enable fires —
no separate "which file does rs1 mean here" wiring needed anywhere else.
`FLW`/`FSW` reuse `data_memory`'s existing word-access path entirely
unchanged, with one override: `FLW` must zero-extend its 32-bit read (a
bit pattern, not a signed integer) where a real `LW` would sign-extend,
so the top level substitutes `func3=110` (`LWU`'s zero-extending code)
specifically for that case.

### D extension (double-precision floating point)

`FLD`/`FSD`; `FADD.D`/`FSUB.D`/`FMUL.D`/`FDIV.D`/`FSQRT.D`; `FMADD.D`/
`FMSUB.D`/`FNMSUB.D`/`FNMADD.D`; `FSGNJ.D`/`FSGNJN.D`/`FSGNJX.D`;
`FMIN.D`/`FMAX.D`; `FEQ.D`/`FLT.D`/`FLE.D`; `FCLASS.D`; the to/from-int
conversions `FCVT.W.D`/`FCVT.WU.D`/`FCVT.L.D`/`FCVT.LU.D` and `FCVT.D.W`/
`FCVT.D.WU`/`FCVT.D.L`/`FCVT.D.LU`; `FMV.X.D`/`FMV.D.X`; and the two
cross-format conversions `FCVT.S.D`/`FCVT.D.S` — 30 instructions, the same
count as F, for the same reasons (it's the same operation set at a
different width, plus the two conversions between the two widths).

**D reuses essentially all of F's infrastructure — this was the payoff of
building `fp_register_file.v` 64-bit-wide and NaN-boxed from the start.**
Every D opcode is *exactly* F's encoding with `funct7`'s low bit flipped
(`FADD.S = 0000000` → `FADD.D = 0000001`, and so on through the whole set)
— RISC-V's `fmt` field is deliberately laid out this way — so
`decode_control_unit.v` handles the entire D instruction set by matching
`funct7[6:1]` the same way it already did for F and using `funct7[0]`
directly as a new `is_double` signal, rather than duplicating every case.
`FLD`/`FSD` are detected the same way `LD`/`SD` already are: `func3=011`
instead of `010`, reusing `data_memory`'s existing width path outright —
unlike `FLW`, `FLD` needs no zero-extension override at all, since a
64-bit read has no sign/zero-extension question to begin with.

**The arithmetic is *simpler* than F's, not harder.** Verilog's native
`real` already *is* IEEE double precision, so double-precision operations
skip the widen-compute-round-down dance entirely: `$bitstoreal`/
`$realtobits` convert directly, with no `f32_to_f64`/`f64_to_f32` step
and — critically — no rounding at all, since the result is already
exactly double precision. `fpu.v` picks between the two paths per-call
based on `is_double`, and the two cross-format conversions
(`FCVT.S.D`/`FCVT.D.S`) are exactly the widen/narrow functions the F
implementation already needed, just now reachable directly as
instructions instead of being internal-only.

New pieces D needed that don't reduce to "the same as F, wider": the
64-bit-int↔float64 conversion functions (`int_to_f64`/`d64_to_int`),
mirroring `int_to_f32`/`f32_to_int`'s bit-level approach (same reasoning
as before: `$itor`/`$rtoi` are 32-bit-only in this Icarus build) but with
an 11-bit exponent and 52-bit mantissa instead of float32's 8/23; and
`FMV.X.D`/`FMV.D.X`, which — unlike `FMV.X.W` — need no sign-extension or
NaN-boxing at all, since a double already fills the entire 64-bit
register exactly.

### RVV 1.0 (scoped)

This is architecturally the biggest jump of all seven extensions —
everything through D reused the existing single-cycle datapath, adding
opcodes and (for F/D) one new register file; RVV needed a genuinely new
kind of state (`vl`, the vector length — the first CSR-like state this
core has anywhere) and a new execution model (one instruction operating on
several data elements at once). Implementing the *full* RVV 1.0 spec —
arbitrary `VLEN`, all four `SEW`s, `LMUL` grouping and its fractional
forms, masking, strided/indexed memory access, reductions, permutations,
vector floating point — would be a substantially larger undertaking than
everything else in this repository combined. So this implementation is
**explicitly scoped down to a real but bounded subset**, in the same
spirit as the RNE-only decision for F/D:

- **`VLEN = 128` bits, `SEW = 32` bits (elements) only, `LMUL = 1` only.**
  `vl` therefore only ever ranges 0–4. No fractional or grouped `LMUL`, no
  8/16/64-bit elements.
- **`vsetvli`/`vsetvl`**, but they only ever configure this one fixed
  `vtype` — the requested `SEW`/`LMUL` encoding in the immediate is
  ignored; only the requested `vl` (via `rs1`, or `VLMAX` when `rs1=x0`,
  per spec) is honored.
- **Arithmetic**: `vadd`/`vsub`/`vand`/`vor`/`vxor`/`vmul`/`vmin`/`vminu`/
  `vmax`/`vmaxu`, each in `.vv` (vector-vector), `.vx` (vector-scalar), and
  `.vi` (vector-immediate, where the real ISA has one — `vmul`/`vmin*`/
  `vmax*` have no `.vi` form, matching spec).
- **Divide/remainder**: `vdivu`/`vdiv`/`vremu`/`vrem`, `.vv`/`.vx` only (no
  `.vi`, matching spec). Divide-by-zero and the one signed-overflow case
  (`MIN_INT / -1`) are defined results per the RVV spec, not traps — see
  "Divide, remainder, and shift" below, including a real Icarus bug this
  surfaced.
- **Shifts**: `vsll`/`vsrl`/`vsra`, all three forms. Shift amount is the
  low 5 bits of the operand (`SEW=32` → shift amounts are mod 32, same as
  scalar RISC-V's own shift instructions).
- **Compares**: `vmseq`/`vmsne`/`vmslt`/`vmsltu`/`vmsle`/`vmsleu`, `.vv`
  and `.vi` forms, producing a packed 1-bit-per-lane mask (see "Compares
  and masks" below).
- **Reductions**: `vredsum`/`vredand`/`vredor`/`vredxor`/`vredmin`/
  `vredminu`/`vredmax`/`vredmaxu`, `.vs` form only — fold all active lanes
  of `vs2` plus a scalar seed (`vs1`'s element 0) down to a single result
  in `vd`'s element 0.
- **Masking**: `v0.t` is implemented for arithmetic, compare, and
  reduction instructions — see "Compares and masks" below for the exact
  policy (mask-agnostic, not mask-undisturbed).
- **Memory**: `vle32.v`/`vse32.v`, unit-stride only — no strided or
  indexed addressing, and vector accesses require 8-byte alignment.
- **Not implemented**: widening/narrowing arithmetic (doesn't fit the
  fixed `VLEN=128`/`SEW=32` model cleanly — see below), permutation
  instructions (`vslide*`/`vrgather`/`vcompress`), vector floating point.

**`vl` is real per-cycle state, not just a decode signal** — the first
time this core has needed that. A `reg [2:0] vl_reg` in `riscv64_proc.v`
holds it, updated only by `vsetvli`/`vsetvl` and read by every subsequent
vector instruction until the next one. Tail handling (elements at or
beyond `vl`) is **tail-agnostic** for both directions: `vle32.v` zero-fills
lanes ≥ `vl` in the destination register (spec-legal — tail-agnostic
behavior is explicitly permitted, not just "the lazy option"), and
`vse32.v` does a read-modify-write so elements ≥ `vl` are left exactly as
they were in memory rather than overwritten with whatever garbage sits in
the unused source-register lanes.

**Reusing `LOAD_FP`/`STORE_FP` for vector memory ops is real RISC-V
design, not a shortcut this core invented**: `vle32.v`/`vse32.v` share
those opcodes with `FLW`/`FLD`/`FSW`/`FSD` because the vector `width`
field's encodings (used here: `110` for 32-bit elements) simply don't
overlap the scalar `width` values `FLW`/`FLD` use (`010`/`011`) — real
hardware disambiguates the same way. This reuse is also exactly what
surfaced the first bug below, since it meant a fix aimed at F/D could
silently break V without any F/D test noticing.

**Compares and masks.** Compare instructions don't produce a normal
per-lane 32-bit result — per spec, they produce one mask *bit* per
element. `vector_alu.v` computes both: a `vd_result` (used by arithmetic
ops) and a separate `cmp_bits[3:0]` (one boolean per lane, used by compare
ops). At the top level, `vcmp_result = {124'b0, cmp_bits & full_lane_mask}`
packs the four booleans into the low bits of `vd`'s *element 0* — the
other three elements of a compare result are always zero. This matters
for anything reading a compare result back out: `vse32.v` + `lw` on
element 0 gets the packed mask; elements 1–3 are not "the mask bit for
that lane" repeated, they're just zero.

Masking reuses that exact packed-bit format: `v0.t` reads are always
`v0_data[3:0]` — the low 4 bits of `v0`'s element 0 — so the natural way
to build a mask by hand (as opposed to generating one from a compare) is
a single-element `vle32.v` (`vl=1`) loading the packed value, not a
4-element load with one 0/1 per lane (that puts each bit in its own
32-bit lane instead of packing them, and silently reads back as
all-zero-or-nonsense when used as `v0.t` — a mistake this session's own
first draft of the masking test made; see "Bugs worth knowing about").

**Masking policy: mask-agnostic (zero-fill), not mask-undisturbed** — the
same tradeoff already made for tail handling on `vle32.v`. A real
mask-undisturbed policy would need `vd`'s *previous* value merged in for
inactive lanes, which means reading `vd` as a third vector-register source
in addition to `vs2`/`vs1` — a third read port `vector_register_file.v`
doesn't have (and adding one only for this would cost real area for a
policy the spec explicitly allows skipping). So inactive lanes — whether
masked by `v0.t` or beyond `vl` — always read back as zero. `full_lane_mask
= v_vm ? tail_mask : (tail_mask & v0_data[3:0])` combines both kinds of
inactivity into one 4-bit mask, applied uniformly to arithmetic
(`varith_result = valu_result & lane_mask_128`), compares, and reductions
(the fold loop simply skips lanes where `full_lane_mask[i]` is 0).

**Reductions are a fold, not a lane operation**, so they reuse
`vector_alu.v`'s op encoding but not its per-lane datapath: a combinational
`for` loop in `riscv64_proc.v` walks the 4 lanes of `vs2`, seeded with
`vs1`'s element 0, applying the reduction op to each active
(`full_lane_mask`-gated) lane in turn, and writes the final scalar to
`vd`'s element 0 (elements 1–3 are zero, same convention as compares).
Reductions are decoded as a *separate* case from arithmetic even though
both share the `OP_V` opcode and (mostly) the same `funct6` encoding
space, because RVV reuses `OPMVV`'s `funct3=010` for both — the real
disambiguator is that reduction funct6 values are always `< 0b001000`
while the regular arithmetic/compare funct6 space starts at `0b001001`
and up (see `decode_control_unit.v`'s `OP_V` case): `is_vreduce = (func3
== 3'b010 && instruction[31:26] < 6'b001000)`. Getting this range wrong
would have silently aliased `vredsum` onto `vmul`'s encoding (both would
otherwise look like generic OPMVV/funct6=0) — worth calling out because
it's the kind of decode collision that only becomes obvious by cross-
checking the *complete* funct6 table, not by testing either instruction
in isolation.

**Divide, remainder, and shift decode in two disjoint `funct6` tables, not
one** — this is real RVV, not an artifact of this implementation. Real
RVV *reuses* `funct6` values across the `OPIVV`/`OPIVX`/`OPIVI` group
(`func3` 000/100/011, used by `vadd`/`vsub`/.../shifts) and the
`OPMVV`/`OPMVX` group (`func3` 010/110, used by `vmul`/divide/remainder/
reductions): `vsll`'s `funct6` (`0b100101`) is *numerically identical* to
`vmul`'s. Only `func3` tells them apart. The original Tier 1/2 decoder
didn't need to care about this — it only ever populated the integer group
— but adding `vdivu`/`vdiv`/`vremu`/`vrem`/`vsll`/`vsrl`/`vsra` in the
same pass would have silently aliased `vsll.vv` onto `vmul.vv` if the
`funct6` case stayed keyed on `funct6` alone. Fixed by splitting
`decode_control_unit.v`'s `OP_V` non-reduction branch into two `case`
blocks gated on which `func3` group the instruction belongs to.

**A second Icarus `$signed()`-in-a-ternary bug, this time in the vector
divide path** — found by the exact same "isolate the sub-expression"
discipline as the FPU's `~int_a[31:0]+1'b1` bug. `vdiv.vv`'s first draft
wrote the divide-by-zero/overflow check as one expression:
`(b==0) ? ... : ($signed(a)==MIN_INT && $signed(b)==-1) ? a :
($signed(a) / $signed(b))`. For `a=0xFFFFFFEC` (-20), `b=4`, this
produced `0x3FFFFFFB` (`0xFFFFFFEC / 4` computed as *plain unsigned*
division) instead of the correct `0xFFFFFFFB` (-5) — confirmed with a
4-line standalone probe showing `$signed(a)/$signed(b)` gives the right
answer *outside* a ternary but the wrong one nested inside one, on this
same Icarus build. Fixed with the same pattern used for the FPU bug:
compute the signed divide/remainder into an explicit `reg signed [31:0]`
*before* the ternary (`sdiv`/`srem` in `vector_alu.v`), and use plain
bit-pattern equality (`a == 32'h80000000 && b == 32'hFFFFFFFF`) instead
of `$signed()` comparisons for the overflow check, to avoid the same risk
there too. Worth noting this wasn't caught by a "does division work at
all" smoke test — `t_v_divrem`'s first two checks (`vdivu.vv`, `vremu.vv`
on all-positive operands) passed fine, since unsigned division was never
wrong; only the signed-divide-with-a-negative-operand case exposed it.

**Widening/narrowing arithmetic was scoped out, not attempted.** RVV's
widening ops (`vwadd`, etc.) produce a result twice the width of the
source elements — e.g. 32-bit source lanes into a 64-bit-element
destination. That doesn't fit this implementation's fixed `SEW=32`/four-
32-bit-lane model at all: the destination would need a different element
count *and* width from the source register in the same 128-bit register
file, which this scoped design has no mechanism for (every register here
is uniformly "4×32-bit lanes"). Supporting it properly would mean
generalizing the vector register file and datapath to variable `SEW`,
which is exactly the larger undertaking the top of this section already
scoped out. Slide/gather/compress (permutation instructions) were also
not attempted this round — lower value than masking/compares/reductions
for the workloads this core is meant to demonstrate (conditional and
aggregate vector code), and not worth the added datapath complexity
(cross-lane muxing) without a concrete use case driving it.

`data_memory.v` gained a second, independent 128-bit port
(`vmem_read`/`vmem_write`/`vmem_addr`/`vmem_write_data`/`vmem_read_data`)
alongside its original 64-bit scalar port, reading/writing two adjacent
64-bit words as one aligned 128-bit access — control-level mutually
exclusive with the scalar port, so there's no arbitration needed between
them.

```
verify/
  asm64.py             RV64IMACFD+V assembler (extends ../../verify/asm.py:
                        LD/SD/LWU, OP-IMM-32/OP-32 word-ops, 6-bit shifts,
                        M/A/C/F/D/V mnemonics, and a general 8-instruction
                        li sequence that builds arbitrary 64-bit constants
                        without relying on LUI's sign-extension, which the
                        RV32I assembler's 2-instruction li can't do safely
                        for RV64 -- see "Bugs worth knowing about" below).
                        Also has a `.word <hex>` directive for splicing
                        pre-encoded machine code (e.g. real compiler
                        output) into otherwise hand-written programs --
                        see the compiler-integration section below.
  tb_core64.v           Generic self-checking testbench (same convention as
                        the RV32I suite: x31 = 0xFFFF0000 on pass).
  build_tests64.py      11 RV64I tests + runner.
  build_tests_m.py      5 M-extension tests + runner (imports
                        build_tests64.py's TestBuilder/build_and_run).
  build_tests_a.py      6 A-extension tests + runner (same reuse pattern).
  build_tests_c.py      6 C-extension tests + runner (same reuse pattern);
                        also extends asm64.py with all 33 compressed
                        mnemonics and a halfword-aware assembler pass.
  c_roundtrip_check.py   A Python port of compressed_decoder.v's expansion
                        logic, used to round-trip every compressed encoder
                        (encode -> decode -> compare) before spending RTL
                        simulation cycles chasing a bit-layout mistake.
                        45/45 checks pass.
  build_tests_f.py      6 F-extension tests + runner (same reuse pattern);
                        also extends asm64.py with all 30 RV64F mnemonics
                        and the R4-type (4-register) encoding FMADD/FMSUB/
                        FNMSUB/FNMADD need.
  build_tests_d.py      8 D-extension tests + runner (same reuse pattern);
                        extends asm64.py's F tables with D's mnemonics,
                        which differ only in a funct7 bit and, for
                        FMADD.D/etc., the R4-type fmt field.
  tb_fpu_unit.v          Unit-level testbench for fpu.v alone (28 checks
                        covering both precisions, expected values computed
                        independently in Python) -- much faster than a
                        full-core run for iterating on the arithmetic, and
                        what actually caught the two real bugs described
                        below.
  build_tests_v.py      6 RVV Tier 1 tests + runner (same reuse pattern);
                        extends asm64.py with vsetvli/vle32.v/vse32.v and
                        the six arithmetic ops in .vv/.vx/.vi form.
  build_tests_v2.py     5 RVV Tier 2 tests + runner: masking (v0.t),
                        compares, min/max, reductions, and a small masked-
                        filter application; extends asm64.py with those
                        mnemonics plus optional v0.t on any arithmetic op.
  build_tests_v3.py     4 RVV Phase-2-completion tests + runner:
                        vdivu/vdiv/vremu/vrem (including divide-by-zero
                        and signed-overflow corner cases) and vsll/vsrl/
                        vsra (including shift-amount masking); extends
                        asm64.py with those seven mnemonics.
  tb_bench64.v           Instrumented variant of tb_core64.v for the
                        benchmark suite: same pass/fail contract, plus
                        dynamic vector-instruction/memory-op counters and
                        a real per-access bytes_moved counter (func3-aware
                        for scalar, VLEN-wide for vector) for the memory
                        bandwidth numbers below.
  bench_common.py        Shared benchmark infra: pack_i32_array/
                        preload_mem (packs int32 arrays into
                        data_memory.v's real 64-bit-doubleword layout for
                        zero-setup-cost preloading) and
                        build_and_run_core/build_and_run_bench (like
                        build_tests64.build_and_run, but write real
                        64-bit values instead of asm.write_mem's
                        documented 32-bit-truncating format).
  bench_rvv.py            Performance benchmark suite: Level 1
                        (add/sub/mul/and/or/xor) + Level 2 (dot product,
                        AXPY, reduction), each as a scalar-vs-vector pair
                        with cycles/instructions/vector instructions/
                        memory ops/CPI/speedup reported.
  bench_compiler_vadd.py  Splices real GCC-compiled machine code (a
                        `c[i]=a[i]+b[i]` loop, `-march=rv64gv... -O3
                        -ftree-vectorize`) into a hand-written harness via
                        asm64.py's `.word` directive and runs it -- proves
                        the C -> compiler -> RVV -> hardware pipeline,
                        not just hand-written RVV assembly.
  bench_level3.py         Level 3 benchmarks (matvec, matmul, conv1d,
                        separable image filter), same scalar-vs-vector
                        methodology, restructured around this core's
                        unit-stride/alignment constraints -- see the
                        Level 3 section above for why each one needed to
                        be.
  tb_core64_lanes8.v      Instantiates riscv64_processor with LANES=8
                        (VLEN=256) instead of the default LANES=4 --
                        proves "configurable vector width" is a real,
                        load-bearing parameter, not a cosmetic one.
  build_tests_lanes8.py   Runs an 8-element vadd.vv through the LANES=8
                        instantiation end-to-end: 1/1 passed.
  generated/            Build output (gitignored).
```

Run:
```sh
export PATH="/c/iverilog/bin:$PATH"
cd RV64I/verify
python3 build_tests64.py       # RV64I base: 11/11
python3 build_tests_m.py       # M extension: 5/5
python3 build_tests_a.py       # A extension: 6/6
python3 build_tests_c.py       # C extension: 6/6
python3 c_roundtrip_check.py   # compressed encoder self-check: 45/45
python3 build_tests_f.py       # F extension: 6/6
python3 build_tests_d.py       # D extension: 8/8
python3 build_tests_v.py       # RVV Tier 1: 6/6
python3 build_tests_v2.py      # RVV Tier 2: 5/5
python3 build_tests_v3.py      # RVV divide/remainder/shift: 4/4
python3 build_tests_lanes8.py  # configurable vector width (LANES=8): 1/1
iverilog -g2012 -o /tmp/fpu.vvp tb_fpu_unit.v ../src/fpu.v && vvp /tmp/fpu.vvp  # FPU unit test: 28/28
```

### RV64I base results (11/11 pass)

| Test | Covers | Key corner case |
|---|---|---|
| `alu_rtype64` | R-type ALU | 64-bit signed overflow (`0x7FFF...FFFF + 1`), shift by 63, signed-vs-unsigned `SLT` on the all-ones pattern |
| `alu_itype64` | I-type ALU | 6-bit shift amounts (shift by 40 — impossible to encode in RV32I's 5-bit field), max/min 12-bit immediates |
| `word_ops` | `ADDW`/`SUBW`/`SLLW`/`SRLW`/`SRAW`/`ADDIW`/`SLLIW` | 32-bit overflow wraps *then* sign-extends (the whole point of `*W`); garbage in a source register's upper 32 bits is correctly ignored |
| `branch64` | All 6 branches | Signed-vs-unsigned divergence on the 64-bit all-ones pattern |
| `mem64` | `LB`/`LH`/`LW`/`LWU`/`LD`, `SB`/`SH`/`SW`/`SD` | `LW` sign-extends vs `LWU` zero-extends (the key RV64-specific distinction); byte/halfword sign vs zero extension into a 64-bit register |
| `jump64` | `JAL`/`JALR` | Return address + target in 64-bit space |
| `upper64` | `LUI`/`AUIPC` | 64-bit sign-extension when bit 31 of the U-immediate is set |
| `app_gcd64` | Euclidean GCD | Data-dependent branching |
| `app_factorial64` | 15! via shift-and-add multiply loop | **1,307,674,368,000 — overflows 32 bits**, specifically exercises 64-bit-width arithmetic |
| `app_fibonacci64` | Fibonacci(50) | **12,586,269,025 — overflows 32 bits**, same rationale |
| `app_sum_array64` | Sum a 10-element doubleword array | 8-byte-stride addressing |

```
[PASS] alu_rtype64 | 135 cycles
[PASS] alu_itype64 | 83 cycles
[PASS] word_ops | 187 cycles
[PASS] branch64 | 79 cycles
[PASS] mem64 | 120 cycles
[PASS] jump64 | 28 cycles
[PASS] upper64 | 29 cycles
[PASS] app_gcd64 | 79 cycles
[PASS] app_factorial64 | 571 cycles
[PASS] app_fibonacci64 | 351 cycles
[PASS] app_sum_array64 | 121 cycles
11/11 passed
```

### M-extension results (5/5 pass)

| Test | Covers | Key corner case |
|---|---|---|
| `m_mul` | `MUL` | negative×positive, negative×negative, 64-bit truncation on overflow |
| `m_mulh` | `MULH`/`MULHSU`/`MULHU` | The three signedness interpretations genuinely diverge on the same bit pattern (`-1` interpreted as signed vs. `2^64-1` unsigned) |
| `m_div_rem` | `DIV`/`DIVU`/`REM`/`REMU` | **Division by zero** (defined result, not a trap), **signed overflow** (`MIN_INT64 / -1`), truncating (not flooring) signed division and its sign-of-dividend remainder |
| `m_word_ops` | `MULW`/`DIVW`/`DIVUW`/`REMW`/`REMUW` | Same division-by-zero/overflow special cases, re-derived at 32-bit width independently of the 64-bit versions |
| `m_app_factorial` | 15! using real `MUL` | Same answer as `app_factorial64`'s shift-and-add version, in ~6x fewer cycles (95 vs. 571) — direct before/after evidence of what the M extension buys |

```
[PASS] m_mul | 106 cycles
[PASS] m_mulh | 100 cycles
[PASS] m_div_rem | 210 cycles
[PASS] m_word_ops | 133 cycles
[PASS] m_app_factorial | 95 cycles
5/5 passed
```

### A-extension results (6/6 pass)

| Test | Covers | Key corner case |
|---|---|---|
| `a_amo_arith` | `AMOADD`/`AMOSWAP`/`AMOXOR`/`AMOAND`/`AMOOR`, doubleword | `rd` gets the *old* value while memory gets the new one — verified both sides of every op, not just the final memory state |
| `a_amo_minmax` | `AMOMIN`/`AMOMAX`/`AMOMINU`/`AMOMAXU` | Signed vs. unsigned comparison genuinely diverges: `min(-1, 1)` is `-1` signed but `1` unsigned |
| `a_amo_word` | `AMOADD.W` | 32-bit old value sign-extends into `rd`; the write only touches the low 32 bits of the doubleword, leaving the upper 32 bits (a different live value) untouched |
| `a_lr_sc_success` | `LR.D`/`SC.D` | `SC` immediately following its own matching `LR` succeeds (`rd=0`) and the memory write actually commits |
| `a_lr_sc_fail_store` | `LR.D`/`SC.D` | An unrelated store *between* the `LR` and `SC` invalidates the reservation — `SC` fails (`rd=1`) and does **not** write memory |
| `a_lr_sc_fail_addr` | `LR.D`/`SC.D` | `SC` to a different address than the `LR`'s reservation fails, and neither memory location is touched |

```
[PASS] a_amo_arith | 217 cycles
[PASS] a_amo_minmax | 153 cycles
[PASS] a_amo_word | 64 cycles
[PASS] a_lr_sc_success | 64 cycles
[PASS] a_lr_sc_fail_store | 72 cycles
[PASS] a_lr_sc_fail_addr | 82 cycles
6/6 passed
```

### C-extension results (6/6 pass)

| Test | Covers | Key corner case |
|---|---|---|
| `c_arith` | `C.LI`/`C.ADDI`/`C.MV`/`C.ADD` (full 5-bit regs), `C.SUB`/`C.XOR`/`C.OR`/`C.AND` (compressed 3-bit regs) | Same arithmetic correctness as the base ISA tests, now via 16-bit encodings mixed with regular 32-bit `li`/`mv` |
| `c_word_ops` | `C.ADDIW`/`C.ADDW`/`C.SUBW` | The same 32-bit-overflow-then-sign-extend corner case as `word_ops`, reached through the compressed encoding |
| `c_shift_upper` | `C.SLLI`/`C.SRLI`/`C.SRAI`/`C.ANDI`/`C.LUI`/`C.ADDI16SP` | Arithmetic vs. logical right shift preserving sign; stack-pointer adjustment by a signed multiple of 16 |
| `c_mem` | `C.ADDI4SPN`/`C.LW`/`C.LD`/`C.LWSP`/`C.LDSP`/`C.SWSP`/`C.SDSP` | Compressed loads/stores round-trip correctly against both compressed-register and stack-relative addressing |
| `c_branch_jump` | `C.BEQZ`/`C.BNEZ`/`C.J`/`C.JR` | Not-taken branches don't skip the fall-through; `C.JR`'s target is computed correctly across a mix of 2-byte and 4-byte instructions in between |
| `c_mixed_gcd` | A real application (Euclidean GCD) freely mixing `c.*` and regular 32-bit instructions | This is the point of the extension — proves the halfword fetch + expansion works over a genuine multi-instruction control-flow program, not just isolated single instructions |

```
[PASS] c_arith | 119 cycles
[PASS] c_word_ops | 88 cycles
[PASS] c_shift_upper | 100 cycles
[PASS] c_mem | 89 cycles
[PASS] c_branch_jump | 51 cycles
[PASS] c_mixed_gcd | 79 cycles
6/6 passed
```

### F-extension results (6/6 pass, plus 28/28 in the standalone FPU unit test)

| Test | Covers | Key corner case |
|---|---|---|
| `f_arith` | `FADD.S`/`FSUB.S`/`FMUL.S`/`FDIV.S`/`FSQRT.S`/`FMADD.S`/`FMSUB.S` | Basic arithmetic correctness, cross-checked against Python's own float32 rounding |
| `f_sign_minmax` | `FSGNJ.S`/`FSGNJN.S`/`FMIN.S`/`FMAX.S` | Sign injection is exact bit manipulation, verified against a value whose sign literally changes the numeric result |
| `f_compare_class` | `FEQ.S`/`FLT.S`/`FLE.S`/`FCLASS.S` | Both directions of an unequal comparison; classification of +0 vs. -0 (bit-identical in value, different `FCLASS` codes) |
| `f_convert` | `FCVT.W.S`/`FCVT.WU.S`/`FCVT.S.W`, plus the 64-bit `FCVT.S.L`/`FCVT.L.S` pair | A negative float converting to a sign-extended negative int; a value (10 billion) chosen to round-trip *exactly* through float32 despite exceeding 32-bit range, specifically exercising the L/LU conversions the 32-bit-only `$itor` bug (below) would have silently broken |
| `f_move_mem` | `FMV.X.W`/`FMV.W.X`, `FLW`/`FSW` | Raw bit-pattern moves round-trip exactly; a negative float (bit 31 set) loaded via `FLW` must **not** get sign-extended like a normal `LW` would |
| `f_app_average` | A small application: average of 4 floats | Multiple chained `FADD.S` + one `FDIV.S`, the kind of sequence real compiled code produces |

```
[PASS] f_arith | 221 cycles
[PASS] f_sign_minmax | 125 cycles
[PASS] f_compare_class | 178 cycles
[PASS] f_convert | 142 cycles
[PASS] f_move_mem | 86 cycles
[PASS] f_app_average | 68 cycles
6/6 passed
```

### D-extension results (8/8 pass)

| Test | Covers | Key corner case |
|---|---|---|
| `d_arith` | `FADD.D`/`FSUB.D`/`FMUL.D`/`FDIV.D`/`FSQRT.D`/`FMADD.D` | Same arithmetic correctness as F, at double width |
| `d_precision` | `FADD.D` on a value (1×10¹⁸) with no exact float32 representation | Specifically proves D isn't just F zero-padded — this value would lose precision if it ever round-tripped through a 32-bit path |
| `d_sign_minmax` | `FSGNJ.D`/`FMIN.D`/`FMAX.D` | Sign injection and min/max at double width |
| `d_compare_class` | `FEQ.D`/`FLT.D`/`FCLASS.D` | Compare/classify at double width |
| `d_convert` | `FCVT.W.D`/`FCVT.D.W`, plus the 64-bit `FCVT.L.D`/`FCVT.D.L` pair | The `int_to_f64`/`d64_to_int` functions doing the same job `int_to_f32`/`f32_to_int` do for F, at double's wider exponent/mantissa |
| `d_f_conversion` | `FCVT.D.S` (widen, exact) and `FCVT.S.D` (narrow, rounds) | Round-trips a value through both conversions and checks the bit pattern survives exactly |
| `d_mem` | `FLD`/`FSD` | Round-trips a negative double (sign bit set) through memory with no unwanted extension, mirroring `f_move_mem`'s `FLW` check |
| `d_app_average` | A small application: average of 4 doubles | Same shape as `f_app_average`, at double width |

```
[PASS] d_arith | 183 cycles
[PASS] d_precision | 38 cycles
[PASS] d_sign_minmax | 96 cycles
[PASS] d_compare_class | 84 cycles
[PASS] d_convert | 104 cycles
[PASS] d_f_conversion | 49 cycles
[PASS] d_mem | 67 cycles
[PASS] d_app_average | 68 cycles
8/8 passed
```

### RVV results (6/6 pass)

There's no way to inspect a vector register directly through the
`check_eq` machinery (same issue F/D registers have), so every test reads
results back out through memory: `vse32.v` the vector to a scratch
address, then plain `lw` + `check_eq` on each of the 4 lanes.

| Test | Covers | Key corner case |
|---|---|---|
| `v_arith` | `vadd.vv` | Elementwise add across all 4 lanes at once |
| `v_sub_logic` | `vsub.vv`, `vand.vv` | Elementwise subtract and bitwise AND |
| `v_scalar_imm` | `vadd.vx` (scalar broadcast), `vadd.vi` (immediate broadcast) | The same operand broadcast into all 4 lanes, from two different sources |
| `v_mul` | `vmul.vv` | Elementwise multiply |
| `v_partial_vl` | `vsetvli` configuring `vl=2` (less than `VLMAX=4`), then `vle32.v`/`vse32.v` | Tail-agnostic behavior in both directions: the load zero-fills lanes ≥ `vl`, and the store leaves memory at those positions untouched rather than overwriting it with the zero-filled tail |
| `v_app_dotprod` | A small application: dot product via `vmul.vv` + a scalar reduction loop | No reduction instruction in this scoped implementation, so the sum-of-products step is an ordinary scalar loop over the vector result — a realistic shape for what "vector op feeding scalar code" looks like here |

```
[PASS] v_arith | 159 cycles
[PASS] v_sub_logic | 309 cycles
[PASS] v_scalar_imm | 225 cycles
[PASS] v_mul | 159 cycles
[PASS] v_partial_vl | 102 cycles
[PASS] v_app_dotprod | 144 cycles
6/6 passed
```

### RVV Tier 2 results (5/5 pass)

| Test | Covers | Key corner case |
|---|---|---|
| `v_masking` | `vadd.vv ..., v0.t` | Masked-out lanes read back as zero (mask-agnostic policy) while masked-in lanes compute normally; mask built as a single packed value in `v0`'s element 0, not one 0/1 per lane |
| `v_compare` | `vmslt.vv`, `vmseq.vv`, `vmsltu.vi` | Compare results pack all 4 lane booleans into element 0 of `vd`, not one scalar per lane — checked with a dedicated `check_vmask` helper (see below) |
| `v_minmax` | `vmin.vv`, `vmax.vv` | Signed min/max across all 4 lanes |
| `v_reduce` | `vredsum.vs`, `vredmax.vs`, `vredmin.vs` | Fold-to-scalar respects the scalar seed (`vs1` element 0), and the result lands only in `vd` element 0 |
| `v_app_masked_filter` | A small application: `vmslt.vi` to build a mask, then a masked `vadd.vi` (add 0, i.e. conditional copy) reusing that mask | Demonstrates the compare→mask→masked-op pipeline end to end — mask generated by one instruction, consumed by a completely different one |

```
[PASS] v_masking | 186 cycles
[PASS] v_compare | 323 cycles
[PASS] v_minmax | 209 cycles
[PASS] v_reduce | 378 cycles
[PASS] v_app_masked_filter | 122 cycles
5/5 passed
```

### RVV divide/remainder/shift results (4/4 pass)

| Test | Covers | Key corner case |
|---|---|---|
| `v_divrem` | `vdivu.vv`, `vremu.vv`, `vdiv.vv` | Signed division/remainder with negative operands and results, truncating toward zero (not floor) |
| `v_div_zero_and_overflow` | `vdivu`/`vdiv`/`vremu`/`vrem` by zero; `vdiv`/`vrem` at `MIN_INT / -1` | RVV-defined (not trapped) results: divide-by-zero gives all-ones/`-1`, remainder-by-zero gives the dividend back, and the one signed-overflow case gives `MIN_INT`/`0` |
| `v_shift` | `vsll.vi`, `vsrl.vi`, `vsra.vi`, `vsll.vx` | Logical vs. arithmetic right-shift (zero-fill vs. sign-extend from the top), and shift-amount masking (a `.vx` shift amount of 33 behaves as 1, not "shift out everything") |
| `v_app_avg_filter` | A small application: `vadd.vv` + `vsrl.vi` composing into `(a+b)>>1` | Exercises an add feeding a shift in one instruction sequence — a realistic vectorized idiom |

```
[PASS] v_divrem | 359 cycles
[PASS] v_div_zero_and_overflow | 609 cycles
[PASS] v_shift | 433 cycles
[PASS] v_app_avg_filter | 160 cycles
4/4 passed
```

## Real compiler integration and performance benchmarks

Every test above proves this core executes *hand-written* RVV assembly
correctly. That leaves a real question unanswered: does an actual
compiler's autovectorizer produce code this scoped implementation can
run, or does it need arbitrary `SEW`/`LMUL`/masking idioms this core
doesn't have? The honest way to answer that is to compile real C with a
real, unmodified compiler and run its actual output through the
simulator — not to write RVV assembly that merely looks like what a
compiler might produce.

### The pipeline actually works

`verify/bench_compiler_vadd.py` compiles

```c
void vadd(int *c, const int *a, const int *b, int n) {
    for (int i = 0; i < n; i++) c[i] = a[i] + b[i];
}
```

with a real xPack GCC 15.2.0 (`riscv-none-elf-gcc
-march=rv64gv_zicsr_zifencei -mabi=lp64d -O3 -ftree-vectorize`, C
extension deliberately excluded for this first integration so every
instruction is a plain 4-byte word). GCC's autovectorizer, with no
prompting toward this core's specific limits, emits exactly this
implementation's supported subset:

```
blez  a3, .L5
.L3: vsetvli a5, a3, e32, m1, ta, ma
     vle32.v v2, (a1)
     vle32.v v1, (a2)
     slli  a4, a5, 2
     sub   a3, a3, a5
     add   a1, a1, a4
     add   a2, a2, a4
     vadd.vv v1, v1, v2
     vse32.v v1, (a0)
     add   a0, a0, a4
     bnez  a3, .L3
.L5: ret
```

This is not a coincidence engineered by the test — it's what a real
compiler naturally generates for a simple `int[]` loop at `-O3`: the
standard "`vl = min(remaining, VLMAX)`" stripmining pattern, `e32`/`m1`
(this core's only supported configuration), and no masking, since a
plain non-predicated loop doesn't need any. The generated `vsetvli`'s
`ta,ma` (tail-agnostic, mask-agnostic) policy request also matches this
core's only implemented policy, for both tail and mask handling.

The compiled function's literal machine code — extracted with `objdump
-d`, copied verbatim, not re-derived or hand-corrected — is spliced into
an otherwise hand-written test program via a new `.word <hex>` directive
added to `asm64.py` for exactly this purpose (emits a raw pre-encoded
32-bit value; labels and `jal`/branch targets around it resolve normally
through the existing two-pass assembler, so no manual address arithmetic
is needed to call into or return from the spliced-in code). The harness
calls it via `jal ra` following the standard RISC-V calling convention
(`a0`=dest, `a1`=a, `a2`=b, `a3`=n) and checks the result against a
Python-computed expected array:

```
[PASS] compiler_vadd | 291 cycles
```

GCC's actual compiled output runs correctly on this simulated hardware.
The `C → real compiler → RVV → this hardware` pipeline is real, not
aspirational — for the class of program it was tested on (a
non-predicated, single-precision-integer, unit-stride elementwise loop).
It has not been tested against loop shapes a compiler might vectorize
differently (reductions, strided access, mixed-width data, or anything
`-O3` decides needs `LMUL>1` for register pressure reasons) — those
would need their own instances of this same splice-and-run process to
confirm, not an assumption that this one success generalizes.

### Benchmark suite (`verify/bench_rvv.py`)

For each benchmark, a scalar (plain RV64I loop) and a vectorized (RVV)
version of the *same* computation are both run through the simulator,
each self-checked against a Python-computed expected result. `N=16`
elements throughout (4 full `VLMAX=4` stripmine groups, no partial tail,
chosen to keep the numbers simple to reason about by hand).

**Methodology note on what's actually measured.** Correctness and
performance are checked by two *separate* programs sharing the identical
loop body, not one: a "measure" program is just the preloaded input
arrays, the loop, and an unconditional pass — its cycle count is the
loop's real cost, nothing else. A "verify" program is the same loop
followed by real `lw`+`check_eq` comparisons — its cycle count is
discarded; its only job is proving the *exact code that got measured* is
correct. A single combined program would have reported cycle counts
polluted by `O(n)` verification overhead having nothing to do with the
loop itself. Input arrays are preloaded directly into data memory rather
than built with `li`+`sw` at runtime, for the same reason: `li` alone is
8 instructions per call in this assembler's fixed-width 64-bit-constant
encoding, so an `O(n)` runtime setup loop would swamp the loop cost for
any `n` small enough to be a readable benchmark.

This core is single-cycle with no stalls, so **CPI is always exactly
1.0** for every program, scalar or vector — that's reported below for
completeness, not because it varies; the number that actually reflects
the vector unit's value is the speedup column.

| Benchmark | Scalar cycles | Scalar mem ops | Vector cycles | Vector instrs (vec) | Vector mem ops | Speedup |
|---|---|---|---|---|---|---|
| add | 169 | 48 | 85 | 20 | 12 | 1.99x |
| sub | 169 | 48 | 85 | 20 | 12 | 1.99x |
| mul | 169 | 48 | 85 | 20 | 12 | 1.99x |
| and | 169 | 48 | 85 | 20 | 12 | 1.99x |
| or | 169 | 48 | 85 | 20 | 12 | 1.99x |
| xor | 169 | 48 | 85 | 20 | 12 | 1.99x |
| dot_product | 153 | 32 | 82 | 21 | 9 | 1.87x |
| axpy (`y = a·x + y`) | 169 | 48 | 85 | 24 | 12 | 1.99x |
| reduction (`sum(A)`) | 97 | 16 | 62 | 13 | 5 | 1.56x |

### Memory bandwidth

`tb_bench64.v` also counts real per-access byte widths every cycle (scalar
accesses vary by `func3` — LB/LH/LW/LD/AMO all decode exactly the way
`data_memory.v` itself does; vector accesses are always the full 128-bit
port). Reported in **bytes/cycle**, which is frequency-independent and
therefore always a fair comparison regardless of what clock this core is
ever actually run at — a bytes/second number would only be as credible as
the clock frequency assumed, so it's deliberately not fabricated here
without a real, synthesized Fmax to back it (see "Area and frequency"
below for what is and isn't available on that front yet).

| Benchmark | Scalar B/cycle | Vector B/cycle | Bandwidth ratio |
|---|---|---|---|
| add/sub/mul/and/or/xor | 1.14 | 2.26 | 1.99x |
| dot_product | 0.84 | 1.76 | 2.10x |
| axpy | 1.14 | 2.26 | 1.99x |
| reduction | 0.66 | 1.29 | 1.96x |

The vector unit's bytes/cycle is consistently roughly double the scalar
version's, tracking the cycle-count speedup closely (as it should: same
total bytes moved, fewer cycles to move them in) — this is a genuine,
derived measurement, not a restatement of the speedup column under a
different name, since it comes from actually counting each retiring
instruction's real access width, not assuming a constant.

Every row passed its "verify" program before its "measure" numbers were
trusted. Speedups cluster around 1.6–2x, not a naive 4x (`VLMAX`) —
that's the honest, expected result for this microarchitecture, not a
shortfall: each stripmine group carries fixed per-group bookkeeping
overhead (`vsetvli`, pointer/counter updates, the branch), so a 4-lane
unit processing only 4 groups of 4 elements each (`N=16`) never gets
enough amortization to approach its theoretical ceiling. Reduction's
lower speedup (1.56x) specifically reflects that the scalar reduction
loop has no memory-write step per iteration (register accumulator only),
while the vector version still pays for `vredsum`'s scalar-seed chaining
across groups — a real architectural tradeoff of this implementation's
"no `vmv.x.s`" scoping decision (see below), not a measurement artifact.

**Reduction and dot-product accumulation chain across stripmine groups
without a vector-to-scalar move instruction** (`vmv.x.s` is not
implemented in this scoped core — see "What's not covered"). Each
`vredsum.vs` takes its *seed* from `vs1`'s element 0, so successive
groups' partial sums are chained by feeding one iteration's result
vector register back in as the next iteration's seed operand
(`vredsum.vs v3, v1, v3`), never needing to move a value out to a scalar
register mid-loop. This works because it stays entirely within
already-implemented instructions, but it does mean the accumulator has
to live in a vector register for the loop's whole duration, which is
part of why the reduction benchmark's speedup is lower than the pure
elementwise ops above.

Run:
```sh
export PATH="/c/iverilog/bin:$PATH"
cd RV64I/verify
python3 bench_compiler_vadd.py   # real-GCC-output integration proof
python3 bench_rvv.py             # Level 1 + Level 2 benchmark suite
python3 bench_level3.py          # Level 3: matvec, matmul, conv1d, image filter
```

### Level 3 (`verify/bench_level3.py`): matvec, matmul, convolution, image filter

Same scalar-vs-vector, measure-vs-verify methodology as Level 1/2. All
four algorithms are deliberately restructured around this core's real
constraints — unit-stride-only vector memory access, 8-byte/doubleword
alignment — rather than assuming an idealized vector unit that doesn't
have them; each restructuring is a legitimate, real vectorization
technique, not a workaround invented for this specific implementation.

| Benchmark | Scalar cycles | Vector cycles | Speedup | Scalar B/cycle | Vector B/cycle | BW ratio |
|---|---|---|---|---|---|---|
| matvec (4x16 · 16) | 517 | 277 | 1.87x | 1.02 | 2.43 | 2.38x |
| matmul (4x4 · 4x8) | 1225 | 665 | 1.84x | 1.31 | 2.41 | 1.84x |
| conv1d (16 outputs, 3-tap) | 249 | 229 | 1.09x | 1.03 | 2.24 | 2.17x |
| imgfilter (10x18→8x16, 3x3 box) | 5149 | 2433 | 2.12x | 1.29 | 3.79 | 2.93x |

- **Matrix-vector multiply** reuses Level 2's dot-product pattern
  (`vmul.vv` + chained `vredsum.vs`) per output row — no new
  architectural issue, since every access (one row of `A`, all of `x`) is
  already unit-stride.
- **Matrix multiplication** cannot use the naive row-times-column
  formulation: that needs strided *column* access on `B`, which this
  core's unit-stride-only vector loads don't support. Restructured as
  row-broadcast accumulation instead — for each row `i`,
  `C[i,:] += A[i,k] * B[k,:]` for each `k` — a real, standard vectorized-
  GEMM technique (not invented for this limitation), where every access
  is unit-stride and each `(i,k)` step is structurally an AXPY with a
  runtime-loaded scalar multiplier.
- **1D convolution** hits the alignment constraint directly: the natural
  vectorization (loop over taps, each tap reading `x` shifted by that
  tap's element offset) needs an *odd*-element shift for tap 1, which
  moves the read address by 4 bytes — not doubleword-aligned, and this
  core has no unaligned/strided vector load to fall back on. Fixed the
  same way real vectorized convolution implementations handle exactly
  this (im2col-style data layout, not a workaround): `K` separately
  8-byte-aligned copies of the shifted input windows are prepared in
  memory before the vectorized loop runs (standing in for what a real
  compiler's data-layout pass would do). The scalar version needs none of
  this — byte-addressed scalar loads have no alignment restriction — and
  is the honest baseline: the vector version's extra data-layout step is
  a real cost of this architecture, not hidden from the comparison.
- **Image filter** (separable 3x3 box sum) is built from the same K-tap
  weighted-accumulate primitive as conv1d, applied twice: a horizontal
  pass (row-aware, needs the same pre-shifted-copy treatment as conv1d)
  then a vertical pass. The vertical pass needs *no* pre-shifted copies
  at all — a shift by one whole image row is `row_stride_bytes = N*4`,
  always a multiple of 8 for an even row width, so every vertical tap is
  naturally aligned. That asymmetry (horizontal passes pay a real
  alignment tax; vertical passes don't) is a genuine, direct consequence
  of this core's alignment rule, not an implementation quirk — and part
  of why image filter shows the largest bandwidth ratio (2.93x) of any
  Level 3 benchmark: the vertical pass vectorizes with zero data-layout
  overhead at all.

## Bugs worth knowing about (found during this build, not the RV32I one)

**`LOAD_FP`/`STORE_FP` were missing from immediate generation entirely**
— a real RTL bug, not a test-harness one, and a good example of why test
*coverage shape* matters as much as test count. `FLW`/`FLD`/`FSW`/`FSD`
decode their offset the same way `LW`/`SW` do (`instruction[31:20]` for
loads, the split `instruction[31:25]`/`instruction[11:7]` for stores), but
`LOAD_FP`/`STORE_FP` were never added to `decode_control_unit.v`'s
immediate-generation `case` — so `imm` silently defaulted to 0 for every
floating-point memory access, regardless of what offset the instruction
actually encoded. Every F and D memory test up to that point happened to
use offset 0 (`flw f3, 0(x2)`, not `flw f3, 16(x2)`), so the bug was
invisible to a fully-passing test suite. Caught only by noticing the gap
while reading the code for an unrelated reason (checking how `LOAD_FP`
should interact with vector loads sharing the same opcode) and going back
to add a nonzero-offset case specifically. Fixed, and both `f_move_mem`
and `d_mem` now include a nonzero-offset check so this can't silently
regress. The general lesson: a green test suite only proves what it
actually exercises — a passing set of tests that all happen to share one
unstated assumption (here, "offset is always 0") can hide a real bug
indefinitely.

**RVV's first real run was 0/6, from four separate mistakes — two in the
RTL, two in the test helpers — found by tracing a minimal 3-instruction
program (`li`+`sw`, `vle32.v`, `vse32.v`) cycle-by-cycle rather than
guessing from the failure alone:**

1. **The `LOAD_FP`/`STORE_FP` immediate fix above, applied too broadly.**
   Adding those opcodes to immediate generation was correct for scalar
   `FLW`/`FLD`/`FSW`/`FSD` — but `vle32.v`/`vse32.v` share the *same*
   opcodes with a completely different bit layout at those same positions
   (`nf`/`mew`/`mop`/`vm`/`lumop` structural fields, not an offset). The
   fix decoded those structural bits as if they were a signed immediate,
   producing a garbage nonzero "offset" that corrupted every vector
   memory address (observed: an intended address of `64` came out as
   `97`). Fixed by gating the immediate decode on `func3` too — 010/011
   get a real immediate, 110 (vector) always gets 0.
2. **`vle32.v` never asserted `vreg_write`** in the `is_vle=1` branch of
   the `LOAD_FP` opcode. The vector register file's write enable is a
   *different* signal from the one vector arithmetic instructions set
   (`vreg_write`, decoded in the separate `OP_V` opcode case), and it was
   simply never wired up for the load case — so `vle32.v` correctly
   computed the right address and read the right memory content
   (confirmed by tracing `vmem_read_data` directly), but the destination
   vector register silently stayed all zeros forever, because nothing
   ever told the register file to actually latch it.
3. **Test helper bug: `vsetvli`'s `rs1` was accidentally the *address*
   register, not an AVL (requested vector length) register.** `x6` held
   the vector's backing memory address in `build_v_regs`, and got reused
   as `vsetvli x7, x6, ...` — so whenever a test vector happened to live
   at address 0 (the very first one, always), `vsetvli` read `rs1=0` as
   "set `vl` to 0", not the intended "use the full vector". The `rs1=x0`
   special case (`vl=VLMAX`) didn't save it, because `x6` isn't `x0` — it
   just *contained* 0 for other reasons. Fixed by using a dedicated
   register (`x11=4`) for the AVL value.
4. **Test helper bug: `lw x8, {addr}(x6)` double-counted the base
   address**, since `x6` already held `addr` and the same value was also
   passed as the immediate offset — reading from `2×addr` instead of
   `addr`. Should have been `lw x8, 0(x6)` (or an `i*4` offset for later
   elements).

Bugs 1–2 are why "coverage shape" (see the point above) matters even
*within* one core: F and D's own test suites gave no signal here at all,
because they never touch the vector opcode-sharing path — it took RVV's
own tests, run against RVV's own new RTL, to surface a defect in code that
technically shipped as part of the F/D work.

**`li`'s sign-extension trap.** The RV32I assembler's `li` pseudo-op builds
a 32-bit constant with `lui`+`addi`. Reused unchanged for RV64, this is
*architecturally correct* but silently dangerous: **RV64I's `LUI`
sign-extends its result to 64 bits whenever the loaded value's bit 31 is
set** — so `li x1, 0xFFFF0000` doesn't give `0x00000000FFFF0000`, it gives
`0xFFFFFFFFFFFF0000`. That's correct hardware behavior, not a bug — but it
meant every test that compared a register against a hex literal with bit 31
set (which was most of them, by construction) was comparing against the
wrong constant. This produced 0/11 failures on the first real run, all
traceable to the test harness, not the RTL. Fixed by giving `asm64.py`'s
`li` a proper 8-instruction sequence (`_li64_words`) that builds the high
and low 32-bit halves independently and combines them without relying on
sign extension behaving any particular way.

**The compressed-instruction encoders had real bit-layout bugs — caught
before touching RTL simulation.** Hand-deriving 16 bit-scatter/gather
functions (one per RVC instruction format) from the spec is exactly the
kind of work that invites silent transcription errors: three of them
(`c_addi16sp`, `c_j`, `c_branch`) had wrong bit positions on the first
pass. These were caught by `c_roundtrip_check.py` — a from-scratch Python
port of `compressed_decoder.v`'s expansion logic — round-tripping every
encoder (encode → decode → compare against the intended fields) *before*
running anything through the simulator. That caught the encoder bugs in
seconds instead of hours of RTL waveform debugging. It's worth calling out
because the natural first instinct when a compressed-instruction test fails
is to suspect the new hardware (`compressed_decoder.v`, the halfword-aware
`instruction_fetch.v`) — and in this case the RTL was actually correct on
the first pass; every remaining failure after the round-trip check turned
out to be in the *test programs* themselves: `c.li`/`c.andi`'s 6-bit signed
immediate silently wrapped out-of-range values like `0xF0` (240) down to 48
instead of erroring (fixed by making `c_field` range-check and raise), and
one jump-target test's address arithmetic assumed every nearby instruction
was 2 bytes when two of them (`auipc`/`addi`) were 4. Both mistakes are
easy to make once a program mixes fixed-size and variable-size
instructions, which is exactly what makes the C extension harder to get
right than the others.

**Two Icarus-specific traps in the FPU, both caught by the standalone
`tb_fpu_unit.v` before they could hide inside a full-core test failure:**

1. **`$itor`/`$rtoi` are documented as 32-bit-only, and it's easy to
   assume otherwise.** Passing a 64-bit `reg` to `$itor` doesn't error or
   warn — it silently truncates to the value mod 2³², so
   `$itor(10_000_000_000)` quietly returns `$itor(1_410_065_408)`.
   Confirmed empirically (see the two-line probe in the commit history)
   *before* building `FCVT.L.S`/`FCVT.S.L` around it, which is why those
   conversions are implemented via direct bit manipulation
   (`f32_to_int`/`int_to_f32`) instead. Worth remembering for any future
   Verilog work involving `real` conversions on this toolchain.
2. **`~int_a[31:0] + 1'b1`, written inline inside a ternary feeding a
   64-bit context, corrupted its upper 32 bits** — `~` is supposed to be
   self-determined at its part-select operand's true width (32 bits) per
   the Verilog LRM, but empirically this Icarus build appears to widen
   `int_a[31:0]` to the surrounding 64-bit context *before* applying `~`,
   giving `0xFFFFFFFF00000001` instead of `1` for `int_a = -1`. Root-caused
   by isolating the exact sub-expression in a two-line testbench rather
   than staring at the full FPU. Fixed by computing the negation into its
   own explicitly-64-bit-declared `reg` first, which sidesteps the
   ambiguity entirely — the general lesson being: when an inline
   expression mixing part-selects, unary operators, and a wide assignment
   context produces a surprising value, don't trust the mental model of
   "should be self-determined" without checking; isolate it.

**RVV Tier 2's first run was 3/5, both failures self-inflicted test bugs
(not RTL) — same "trace the exact semantics by hand before touching RTL"
discipline as the Tier 1 bugs above:**

1. **`v_masking`'s first draft built the mask wrong.** It loaded a 4-element
   vector (one `0`/`1` per lane) via `vle32.v` and used that as `v0`,
   expecting each lane's own value to gate that lane. But `v0.t` masking
   reads only `v0_data[3:0]` — the low 4 bits of *element 0* — because
   that's the packed format compare instructions actually produce
   (`cmp_bits` → `vcmp_result`). A 4-element per-lane load puts each 0/1 in
   its own 32-bit lane instead of packing them into element 0's low bits,
   so the "mask" the hardware actually read was just element 0's low 4
   bits (`0b0001`, from the first loaded value being `1`), not the
   intended `0b0101`. Fixed by loading a single packed value instead — the
   correct way to hand-construct a mask matches exactly how a compare
   instruction would produce one.
2. **`v_compare` checked results with the wrong helper.** `check_vreg`
   (built for arithmetic results, one real scalar per lane) was reused for
   compare results, which pack all 4 lane booleans into element 0 only —
   elements 1–3 are always zero. For `vmslt.vv` this happened to pass by
   coincidence (only bit 0 was set, so the packed value and the "per-lane"
   check lined up by accident); `vmseq.vv` (bit 1 set → packed value `2`)
   immediately exposed the mismatch, since `check_vreg` compared element
   0 against the *expected bit for lane 0* (`0`) instead of the packed
   value (`2`). Fixed by adding a dedicated `check_vmask` helper that packs
   the expected bits itself and compares against element 0 only. Worth
   noting because the first test that happened to pass (`vmslt.vv`) could
   easily have been mistaken for proof the checking approach was correct.

**Two more test-harness mistakes when adding divide/remainder/shift, both
instances of gotchas already documented above rather than new ones —
worth calling out anyway, since it shows the same class of mistake
recurring once negative vector results entered the picture for the first
time:**

1. **`check_vreg`'s non-negative assumption, tripped for real.** Every
   Tier 1/2 test happened to use small positive results, so `check_vreg`'s
   `assert 0 <= val < 0x80000000` never fired. The divide-by-zero test
   (`vdivu` by zero → `0xFFFFFFFF`) and the signed-overflow test
   (`MIN_INT / -1` → `0x80000000`) both have the sign bit set, tripping
   the assertion immediately at test-construction time (a clear Python
   `AssertionError`, not a silent wrong answer) — a deliberate safety net
   in `check_vreg` doing its job, not a bug. Fixed by adding a
   `check_vreg_signed` helper for exactly this case.
2. **The `li`-sign-extension gotcha (see "`li`'s sign-extension trap"
   above) reappeared, this time on the *checking* side rather than the
   *building* side.** Comparing a negative division result (e.g. `-5` as
   a 32-bit pattern) with plain `check_eq(reg, 0xFFFFFFFB)` builds the
   scratch constant as `li`'s literal 64-bit value (`0x00000000FFFFFFFB`,
   zero-extended), but `lw` always sign-extends its 32-bit result to 64
   bits (`0xFFFFFFFFFFFFFFFB`) — so the two never match even when the
   division is correct. `s32_to_s64hex`/`check_vreg_signed` in
   `build_tests_v3.py` fix this by sign-extending the expected value
   before handing it to `check_eq`, matching what `lw` actually produces.

**Two more test-harness bugs building the benchmark suite, both found by
tracing the exact memory-write log rather than guessing — the same
discipline as every other bug in this catalog:**

1. **`asm.write_mem` silently truncates every value to 32 bits — an
   established, documented contract other tests correctly rely on (see
   `sum_array64`'s comment in `build_tests64.py`), not a bug in
   `write_mem` itself, but one the benchmark suite's first draft violated
   by reusing it for something it was never designed for.** The
   benchmark suite needs to preload int32 arrays packed two-per-doubleword
   (`pack_i32_array`, matching `data_memory.v`'s real 64-bit-wide, 8-byte-
   stride layout) — but piping that through `write_mem` discarded the
   upper 32 bits of every doubleword, so every *odd-indexed* array
   element (living in a doubleword's upper half) silently read back as
   zero. Every even-indexed element was fine, which is exactly what made
   the failure pattern (`check #2` fails, `check #1` doesn't) legible
   once the raw `MEM WRITE`/`MEM READ` log was inspected directly instead
   of just re-reading the assembly for a logic error. Fixed by adding a
   separate `write_dmem64` helper in `bench_common.py` (writes real
   16-hex-digit 64-bit lines) instead of changing `write_mem`'s existing,
   depended-upon behavior.
2. **A vector-store scratch address wasn't 8-byte-aligned.** The dot-
   product and reduction benchmarks' verification code stored their
   `vredsum` result via `vse32.v` to address `3100` — not a multiple of
   8, violating this scoped implementation's documented alignment
   requirement. The hardware doesn't trap on this (consistent with how
   misalignment is handled everywhere else in this core); it just used
   `vmem_addr[...:3]`, silently rounding down to doubleword `3096`, so
   the write landed in the *low* half of that doubleword while the
   immediately following `lw x8, 0(x22)` (also using the unaligned
   `3100`) read the *high* half — reading back zero even though the
   vector computation itself had already produced the correct answer
   (visible directly in the `VMEM WRITE` log entry, which is what made
   this a five-minute fix once compared against the failing `lw`'s
   address rather than re-deriving the reduction logic from scratch).
   Fixed by using an aligned scratch address (`3200`).

**Three more test-harness bugs building the Level 3 benchmarks — all
found the same way as every bug in this catalog: trace the actual
memory-transaction log or isolate the failing stage, don't guess from the
assembly:**

1. **`li`'s x28-scratch-clobber trap, this time in the matmul vector
   routine.** `li`'s pseudo-op uses `x28` internally as scratch (see the
   `li`-sign-extension entry above for the same register in a different
   role) — the matmul vector loop loaded `a_ik` into `x28` via `lw`, then
   called `li x9, {n}` *before* using `x28` in `vmul.vx`, silently
   corrupting `a_ik` in between. Symptom: `C[0][0]` came back as the
   result of an *unsigned row/column division* of the raw input bytes
   (`0x3FFFFFFB`-style corruption, same family as the earlier signed-
   divide-in-a-ternary bug, but this time a pure test-code register-
   allocation mistake, not an RTL one). Fixed by reordering: load the
   loop-trip-count `li` before the value that needs `x28` to survive, not
   after.
2. **A label collision between the image filter's horizontal and
   vertical passes.** Both passes are built by the same
   `flat_tap_sum_asm` generator, which originally used unqualified loop
   labels (`ft_vector_tap0` etc) regardless of caller. Concatenating two
   calls' output into one assembled program meant the *second* call's
   labels silently overwrote the *first*'s in `Assembler64`'s one flat
   label table, so the horizontal pass's own loop back-edges resolved
   into the vertical pass's code instead of looping on themselves. The
   signature that gave this away: each pass, tested in complete
   isolation, produced the exact right answer — only the combined
   two-pass program failed, which is a label/addressing problem, not a
   logic error in either pass (a real logic bug would have failed in
   isolation too). Fixed by adding a required `tag` parameter so every
   call site gets its own unique label namespace.
3. **Branch-immediate range, hit for the first time by a large check
   count.** `bne`'s branch immediate is 13 bits signed (±4KB). Checking
   all 128 image-filter output elements individually (`check_eq` per
   element) built a program large enough that early checks' jumps to
   their fail handler — placed after *all* checks, at the very end of the
   program — exceeded that range, something no earlier test in this
   session's much smaller check counts had ever triggered. Fixed by
   checking a representative sample (both corners, both edge midpoints,
   the center) instead of exhaustively checking every element — real,
   direct evidence at the positions most likely to expose a row/column
   boundary mistake, without hitting the assembler's own range limit.

## Configurable vector width

`riscv64_processor`'s `LANES` parameter (default `4`, matching every
number reported everywhere else in this document) is a genuine synthesis-
time knob, not cosmetic: `VLEN = LANES*32` is derived from it, and every
width-dependent construct in the vector datapath —
`vector_register_file.v`'s register width, `vector_alu.v`'s per-lane
`generate` loop, `data_memory.v`'s vector port (generalized from a fixed
2-doubleword access to `VLEN/64` doublewords via its own `generate`
loop), and `riscv64_proc.v`'s `vl_reg` width/tail-mask/lane-mask/
reduction-loop-bound logic — is now built generically off `LANES` rather
than hardcoded to 4 or 128.

**Proof, not just a parameter that exists and does nothing:**
`verify/tb_core64_lanes8.v` explicitly instantiates the core with
`LANES=8` (`VLEN=256`), and `verify/build_tests_lanes8.py` runs an
8-element `vadd.vv` through it end-to-end — genuinely computing all 8
elements in one vector instruction, not 4, checked against a Python-
computed expected array:

```
[PASS] lanes8_vadd | 270 cycles
```

This is deliberately a single, focused proof, not a parallel `LANES=8`
copy of the entire RVV test suite — the parameterization itself is what
makes re-verifying every instruction at a second width structurally
redundant, since nothing in the width-dependent logic is hand-duplicated
per width; it's all generated once, generically, off `LANES`. The full
regression suite (57/57 across every other, pre-existing test) still passes unchanged
at the default `LANES=4`, confirming the parameterization is behavior-
preserving at the configuration every other number in this document
assumes.

**Not (yet) configurable**: `SEW` is fixed at 32 regardless of `LANES` —
this scoped implementation never varies element width, only element
*count*, which is a real, narrower claim than full RVV's independently
configurable `SEW`/`LMUL`/`VLEN`. `LANES` should stay even (`VLEN` a
multiple of 64) since `data_memory.v`'s vector port is doubleword-
granular; an odd `LANES` isn't a configuration this core is designed to
support.

## Area and frequency

Attempted with a real, unmodified open-source toolchain (Yosys 0.68 +
ABC9 + nextpnr, the [oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build)
distribution) targeting Lattice iCE40 and ECP5, not estimated or guessed.

```sh
export PATH="/c/oss-cad-suite/bin:/c/oss-cad-suite/lib:$PATH"
cd RV64I/synth
yosys -s synth_ice40.ys   # area: real LUT/carry/FF counts (below); doesn't fit iCE40, no P&R
yosys -s synth_ecp5.ys    # larger target -- attempted for a real, P&R-based Fmax
```

Getting honest numbers here required working around two real obstacles
specific to this design, both worth documenting on their own:

1. **`fpu.v` is not synthesizable at all, by any tool.** Its F/D
   arithmetic is built on Verilog's `real` type
   (`$bitstoreal`/`$realtobits`) — a simulation-only construct with no
   hardware mapping, full stop, not a Yosys limitation. `RV64I/synth/`
   contains a synthesis-only variant of the top level
   (`riscv64_proc_synth.v`) with `fpu.v`/`fp_register_file.v` removed and
   their outputs tied to constants, so it correctly reports area for "the
   synthesizable RV64IMAC + RVV integer/vector datapath, excluding F/D" —
   a real, meaningful, but explicitly *narrower* number than "the whole
   core," stated as such rather than silently presented as the whole
   core's area. A real synthesizable FPU would be a separate, substantial
   undertaking on its own.
2. **Naive synthesis of the instruction/data memories collapses the
   entire design to almost nothing.** `instruction_fetch.v`'s memory is
   a write-port-less array (a ROM in hardware terms); with its `$readmemh`
   content fixed at elaboration time (e.g. all zeros, the placeholder
   used to get synthesis running at all), Yosys correctly — not buggily —
   constant-folds the ROM straight through the *entire* downstream
   design, since a permanently-NOP instruction stream really does
   trivialize to almost nothing. Removing the `$readmemh` entirely (so
   content is fully unknown) fails the opposite way: undefined memory
   content propagates as "don't care" and the design *still* collapses.
   Neither is useful for "how much hardware does this general-purpose CPU
   need." Fixed the standard, correct way real chip area is reported
   anyway: `instruction_fetch.v`/`data_memory.v` are `(* blackbox *)`
   stubs in the synthesis-only copy, so their own area (a real memory
   macro/BRAM in any actual deployment) is reported separately from core
   logic, not folded into or through it — this also sidesteps a real,
   separate architectural fact: this core's instruction fetch is
   deliberately *combinational* (async read, no write port — a design
   choice from earlier in this project that fixed a real PC/instruction
   race condition), and iCE40's block RAM only supports *synchronous*
   (clocked) reads, so this exact RTL could not use dedicated BRAM on
   that target even if its content weren't a problem — it would need
   full LUT-based distributed ROM, a real, substantial cost of the
   combinational-fetch design choice that a naive "just check the LUT
   count" attempt would have hidden by silently constant-folding around
   it instead of hitting it honestly.

### Area: real numbers (`synth_ice40.ys`)

`synth_ice40` completed after a long ABC9 technology-mapping run (tens
of minutes — see below for why). Real cell counts for the synthesizable
RV64IMAC + RVV core (`LANES=4`, F/D excluded, `RV64I/synth/`'s blackbox-
memory variant):

| Resource | Count |
|---|---|
| 4-input LUTs (`SB_LUT4`) | 52,126 |
| Dedicated carry cells (`SB_CARRY`) | 19,108 |
| Flip-flops (`SB_DFFE`/`SB_DFFER`/`SB_DFFES`/`SB_DFFR`) | 6,212 |

**This does not fit on any real iCE40 device** — the largest part in the
family (iCE40 HX8K) has ~7,680 LUTs, roughly 15% of what this core needs.
That's not a synthesis failure; LUT count is real, valid data regardless
of whether a specific chip is big enough to hold it, and reporting it
honestly (rather than not reporting area at all because "it doesn't fit
iCE40") is more useful than silence. It does mean iCE40 place-and-route
(and therefore an iCE40-specific Fmax) isn't attainable for this design;
`RV64I/synth/synth_ecp5.ys` targets Lattice ECP5 instead (largest part,
LFE5UM5G-85F, has ~84,000 LUTs — enough headroom to actually attempt
placement) for a real, timing-driven Fmax, [status TBD — see the run
command below if this document doesn't yet show a completed ECP5 result].

**Why this took tens of minutes, not seconds — a real, expected cost, not
a stuck process:** this scoped core's divide/remainder support means
**five independent combinational (single-cycle, non-iterative) hardware
dividers** exist in the synthesizable netlist — one for the scalar
M-extension (`execute.v`) and, since `vector_alu.v`'s divide/remainder
lanes are fully parallel, **one per vector lane** (four, at the default
`LANES=4`) for `vdivu`/`vdiv`/`vremu`/`vrem`. A full combinational
divider is a genuinely large, difficult-to-optimize Boolean function —
well-documented as expensive for logic synthesis tools in general — and
this design pays that cost five times over, once for scalar and four
times more for the vector unit's per-lane parallelism. Concretely, the
full netlist Yosys extracted before technology mapping was ~198,000 AND
gates across 6,438 inputs/911 outputs; a partial, isolated result from
the same run already confirmed the divider cost directly: the
top-level module's own glue logic (control/muxing, excluding the
`execute` and `vector_alu` submodules where the dividers live) mapped to
roughly 7,360 gates in seconds; `execute.v` alone had not finished
mapping after several minutes on the same run.

**What this means, stated plainly:** the 52K-LUT area figure above is
real and final for `LANES=4` on this target flow. A real Fmax number
still needs a device the design actually fits on (ECP5, attempted
separately — see above) and place-and-route timing analysis, which
wasn't available within this session for the reasons already stated. If
this core were headed toward real silicon or a real FPGA deployment
rather than a functional/architectural demonstration, the standard,
well-known fix for the divider cost specifically would be trading the
single-cycle combinational divider for a multi-cycle iterative one (SRT
division or a simple restoring-division state machine) — plausibly an
order-of-magnitude area reduction for that specific bottleneck, at the
cost of turning DIV/REM into multi-cycle operations. That's a real
microarchitecture change (this core is single-cycle throughout by
design; introducing the first multi-cycle instruction is a bigger
decision than this session's scope) and wasn't attempted here — noted as
the concrete next step for anyone who *does* need this core to fit a
real device, not as an excuse for the numbers above.

## What's not covered

Everything the sibling RV32I core doesn't cover (no pipelining, no
traps/CSRs/interrupts, misaligned accesses only within a word/doubleword,
not across one) still applies. On top of that, RV64-specific:

- **RVV 1.0 is deliberately scoped down**, not fully implemented — see the
  RVV section above for the complete list. `VLEN=128`/`SEW=32`/`LMUL=1`
  only (no other widths or grouping); masking, compares, min/max,
  reductions, divide/remainder, and shifts **are** implemented, but
  permutation instructions (`vslide*`/`vrgather`/`vcompress`),
  vector-to-scalar move (`vmv.x.s`/`vmv.s.x`), widening/narrowing
  arithmetic, and vector floating point are not; `vsetvli` only ever
  configures the one fixed `vtype` it supports (ignoring the requested
  `SEW`/`LMUL` in its immediate); masking is mask-agnostic (zero-fill)
  rather than mask-undisturbed, for the same reason tail handling is
  tail-agnostic (no third vector-register read port). This is a stated
  scope decision — the full spec is an order of magnitude larger than
  everything else in this core combined — not an oversight. The
  benchmark suite (see above) works around the missing `vmv.x.s` for
  cross-group reduction accumulation by chaining `vredsum.vs`'s scalar
  seed operand instead of moving a value to a scalar register mid-loop.
- Compressed floating-point loads/stores (`C.FLD`/`C.FSD`/`C.FLDSP`/
  `C.FSDSP`) aren't implemented — they'd be straightforward additions to
  `compressed_decoder.v` following the existing `C.LD`/`C.SD`/`C.LDSP`/
  `C.SDSP` pattern, just routed to the FP register file, but weren't
  needed by anything built so far.
- **F/D are RNE-only**: the `rm` field and `fcsr`/`frm`/`fflags` are
  decoded where relevant but not otherwise implemented — every FP
  operation always rounds round-to-nearest-even, and no exception flags
  are recorded (there is no CSR file at all yet). See the F extension
  section above for the reasoning; this is a stated scope decision, not an
  oversight.
- Subnormal rounding in the float32↔float64 conversion (used internally by
  single-precision arithmetic) is best-effort, not exhaustively verified
  across the whole subnormal domain the way the normal-range path is.
- `ECALL`/`EBREAK` still only raise a testbench-visible `ecall_halt`, not a
  real trap.
- The A extension's `aq`/`rl` (acquire/release) bits are decoded (unused)
  but architecturally meaningless here: a single-hart, non-pipelined,
  single-cycle core can't reorder memory operations in the first place, so
  there's nothing for those bits to constrain.
