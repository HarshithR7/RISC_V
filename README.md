# RISC_V — Single-Cycle RV32I Processor

A single-cycle RV32I core written in Verilog, developed as a Vivado project
(`RISC_V.xpr`). This README documents the architecture, the bugs found and
fixed in an August 2026 review, the instructions added, and the verification
suite (with real simulation results) that now backs the design.

**A 64-bit sibling core lives in [`RV64I/`](RV64I/README.md)**: RV64IMACFD
plus a scoped RVV 1.0 — base integer, M (multiply/divide), A (atomics), C
(compressed instructions), F (single-precision floating point), D
(double-precision), and a deliberately bounded, width-configurable vector
unit (128-bit registers by default, 32-bit elements, with masking/
compares/reductions/divide/shifts but no permutation instructions,
widening, or vector float) — all implemented and verified (58/58 tests
pass), built with the same architecture and verification discipline as
this core. A real GCC autovectorizer's actual compiled output has been
run against it end-to-end; a 13-benchmark scalar-vs-vector suite (through
matrix multiply, convolution, and image filtering) measures real
1.09–2.12x speedups and up to 2.93x memory-bandwidth gains from the
vector unit; and the vector width itself is a real synthesis-time
parameter, proven by instantiating and running the core at `LANES=8`
instead of the default 4.

## Architecture

Classic Patterson & Hennessy–style single-cycle datapath: one instruction
fetches, decodes, executes, accesses memory, and writes back per clock cycle.

```
        +-----+     +-----------------+     +----------------------------+
 reset->|     |     |                 |     | Instruction_decode_        |
        | PC  |---->| instruction_    |---->| control_unit               |
        |     |<-+  | fetch (async    |     | (combinational decode +    |
        +-----+  |  |  ROM read)      |     |  control signals)          |
                 |  +-----------------+     +--------------+-------------+
                 |                                         |
                 |   +----------------+     rs1,rs2,rd,imm, |
                 |   | register_rw    |<--------------------+
                 |   | (32 x 32b,     |     alu_op,alu_src,
                 |   |  sync write,   |     mem_read/write,
                 |   |  async read,   |     mem_to_reg,func3
                 |   |  x0 hardwired) |
                 |   +-------+--------+
                 |           | rs1_data, rs2_data
                 |           v
                 |   +----------------+     +----------------+
                 |   | execute1 (ALU, |---->| data_memory     |
                 +---| branch/jump    |     | (byte/half/word,|
    next_pc          | target calc)   |     |  sign/zero ext) |
                     +-------+--------+     +--------+-------+
                             |                        |
                             +---- write-back mux -----+
                          (mem_to_reg ? mem_read : alu_result)
```

Modules (`RISC_V.srcs/sources_1/new/`):

| File | Module | Role |
|---|---|---|
| `program_counter.v` | `Program_counter` | Synchronous PC register |
| `Instruction_Fetch.v` | `instruction_fetch` | Asynchronous instruction ROM (parameterized file/size) |
| `Instruction_decode_control_unit.v` | `Instruction_decode_control_unit` | Combinational decode + control (the module actually used in the datapath) |
| `register_rw.v` | `register_rw` | 32×32-bit register file, x0 hardwired to zero |
| `alu.v` | `execute1` | ALU, branch comparison, jump/branch target calc, write-back value select |
| `data_memory.v` | `data_memory` | Byte/halfword/word-addressable data RAM (parameterized file/size) |
| `riscV_proc.v` | `riscv_processor` | Top-level datapath wiring |
| `riscV_tb.v` | `riscv_processor_tb` | Vivado's default `sim_1` testbench |
| `control_unit.v`, `instruction_decode.v` | `ControlUnit`, `instruction_decode` | Earlier, decomposed decode/control modules — **not** part of the live datapath, kept only so `RISC_V.srcs/sim_1/new/id_tb.v` can unit-test decode logic standalone |

Vivado fileset tops (`RISC_V.xpr`): synthesis top `riscv_processor`, simulation
top `riscv_processor_tb` — unchanged by this pass.

### ISA support

Full **RV32I base integer ISA**: R-type and I-type ALU ops (including
`SLT`/`SLTU`/`SLTI`/`SLTIU`, which were previously unimplemented), all six
branches (`BEQ`/`BNE`/`BLT`/`BGE`/`BLTU`/`BGEU`), `LUI`/`AUIPC`, `JAL`/`JALR`,
and all load/store widths (`LB`/`LH`/`LW`/`LBU`/`LHU`, `SB`/`SH`/`SW`) with
correct sign/zero extension. `ECALL`/`EBREAK`/`FENCE` are treated as a
simulation-halt signal (see below) / no-op respectively — there is no trap
handling, CSR file, or M/A/F/D extension support. No pipelining, hazard
forwarding, or interrupts.

### ECALL-based halt (new)

`riscv_processor` now exposes an `ecall_halt` output (opcode `SYSTEM`,
`1110011`), so a test program can signal "I'm done" instead of the testbench
guessing a cycle count. `riscv_processor_tb` watches this and calls
`$finish` on it (falling back to a 2000-cycle timeout if a program never
reaches one). Convention used throughout the test suite: write a status code
to `x31` before the `ecall` — `0xFFFF0000` for pass, a small nonzero integer
identifying which check failed otherwise.

### Parameterization (new)

`riscv_processor`, `instruction_fetch`, and `data_memory` now take
`IMEM_FILE` / `DMEM_FILE` / `*_WORDS` parameters (defaulting to
`instructions.mem` / `data.mem` / 4096 words = 16 KB each), so different
programs can be simulated without editing RTL — see [`verify/`](#verification-suite-verify).

## Bugs found and fixed

The core was previously **not functionally correct** — most instruction
classes either did nothing, did the wrong thing, or corrupted register/memory
state. In severity order:

1. **Branches never took.** The control unit forced `alu_op = SUB` for every
   branch and stuffed `func3` into a 1-bit `branch_type` signal that wasn't
   even wired to the execute stage. The execute stage's branch-taken logic
   switched on `alu_op` (always `SUB`), which matched none of its four cases.
   Net effect: `BEQ`/`BNE`/`BLT`/`BGE` were dead code — no branch instruction
   ever changed control flow. Fixed by routing `func3` into `execute1` and
   deciding `BEQ`/`BNE`/`BLT`/`BGE`/`BLTU`/`BGEU` there (the last two didn't
   exist before).
2. **`instruction_fetch` was clocked, creating a one-cycle fetch/PC race.**
   `next_pc` (computed by `execute1` from the *current* instruction) and the
   PC register update on the same clock edge; a registered instruction-memory
   read made `instruction` depend on `pc` from that same edge, and `pc`'s
   own update depended on `instruction` — a same-edge circular dependency
   between two independently-clocked always-blocks. Empirically this made
   `instruction` permanently lag `pc` by a full cycle, so every branch/jump
   decision was computed against the *previous* instruction (this is what
   caused every branch/loop test to fail or hang during verification — see
   below). Fixed by making instruction memory an asynchronous (combinational)
   read, matching standard single-cycle datapath design and the same style
   already used for `data_memory`'s read port.
3. **No write-back mux — `LOAD` never wrote loaded data to a register.**
   The register file's `write_data` was hardwired to `alu_result`. For a
   `LOAD`, `alu_result` holds the *address* calculation, not the memory
   read — so `lw`/`lb`/`lh` etc. wrote the address into `rd`, not the loaded
   value. Fixed with `write_back_data = mem_to_reg ? output_mem_read :
   alu_result` at the top level.
4. **`STORE` wrote the wrong data to memory.** `data_memory.write_data` was
   wired to `alu_result` (the computed *address*, since `alu_op=ADD` for
   stores) instead of `rs2_data` (the value to store). Fixed.
5. **`LUI`/`AUIPC` immediates were never decoded.** Neither opcode appeared
   in `Instruction_decode_control_unit`'s immediate-generation `case`, so
   `imm` silently defaulted to 0 for both. Fixed: `imm = {instruction[31:12],
   12'b0}`.
6. **`LUI` used `rs1` as an operand.** `rs1`'s bit-field in a U-type
   instruction isn't a real source register — those bits are part of the
   immediate. The ALU computed `rs1_data | imm` using whatever register that
   bit pattern happened to name. Fixed: execute stage now forces operand-1 to
   0 for `LUI` (and to `pc` for `AUIPC`) rather than reading `rs1`.
7. **`AUIPC` used `rs1_data + imm` instead of `pc + imm`.** Same root cause
   as #6; fixed by the same operand-1 mux.
8. **`JAL`/`JALR` didn't write a return address.** `rd` was written with
   whatever the generic ALU case produced, not `pc + 4`. Fixed: execute
   stage overrides `alu_result = pc + 4` for `JAL`/`JALR`.
9. **`SLT`/`SLTU`/`SLTI`/`SLTIU` were unimplemented.** The ALU's op `case`
   had no entries for `ALU_SLT`/`ALU_SLTU`; `alu_result` silently kept its
   previous value (a simulation latch). Implemented properly, with signed
   vs. unsigned comparison tested explicitly (see corner cases below).
10. **`data_memory` ignored `func3` entirely.** The port existed but was
    unused in the module body — every load/store behaved as a full 32-bit
    word access regardless of width, and byte/halfword instructions would
    corrupt neighboring bytes. Implemented real byte/halfword read
    (sign-extending `LB`/`LH`, zero-extending `LBU`/`LHU`) and
    read-modify-write byte/halfword stores that leave the other bytes in the
    word untouched.
11. **`execute_tb.v` didn't compile.** It instantiated a module named
    `execute` with a port list (including a nonexistent `write_back_data`
    output) that matched neither the actual module name (`execute1`) nor its
    ports. Rewritten as a real self-checking unit test (see below).
12. **Register-file write timing was non-standard.** Writes happened in an
    `always @(*)` block using a nonblocking assignment — i.e., writes fired
    combinationally whenever `write_data`/`reg_write` settled, not on a clock
    edge. This happened to mostly work in simulation only because Verilog's
    implicit `@*` sensitivity doesn't track memory-array element reads, but
    it's fragile and not what real register-file hardware does. Changed to a
    synchronous write on `posedge clk` (same edge as the PC and data-memory
    writes), with `x0` additionally hardwired to read as zero regardless of
    what's stored.
13. **`branch_type` truncated `func3`.** Declared as a 1-bit `reg` but
    assigned the 3-bit `func3`; harmless only because the signal was already
    dead (see #1). Widened to 3 bits for correctness in the legacy/unused
    `control_unit.v`, and dropped entirely from the live datapath in favor of
    wiring `func3` straight into `execute1`.

None of these are style nits — #1–#10 mean the processor, as originally
written, could not correctly run *any* program containing a taken branch, a
load, a store, `LUI`, `AUIPC`, a jump, or `SLT`/`SLTU`. All are covered by
regression tests below so they can't silently regress.

## Verification suite (`verify/`)

No Verilog simulator was available on this machine, and Chocolatey installs
failed (admin-restricted: `C:\ProgramData\chocolatey` isn't writable by this
user). **Icarus Verilog 12.0** was installed instead via
`winget install --id Icarus.Verilog` (lands at `C:\iverilog`, no admin
prompt) and used for every result below — these are real `iverilog`/`vvp`
runs, not hand-traced expectations.

```
verify/
  asm.py                 Minimal RV32I assembler (registers, all RV32I
                          encodings, li/mv/j/nop/ecall pseudo-ops) used to
                          generate correctly-encoded .mem test programs
                          instead of hand-encoding hex.
  tb_core.v               Generic self-checking testbench: runs a program,
                          watches ecall_halt, reports PASS/FAIL/TIMEOUT by
                          reading x31 (0xFFFF0000 = pass, else = failing
                          check id).
  build_tests.py          11 test programs (below) + runner: assembles each,
                          compiles against the real RTL with iverilog,
                          runs it, prints a summary.
  build_default_program.py
                          Generates the instructions.mem/data.mem that ship
                          in RISC_V.srcs/sources_1/new/ (Vivado's default
                          sim_1 program) — a condensed regression covering
                          every instruction category plus Fibonacci(15).
  generated/              Build output (gitignored): .mem files, per-test
                          wrapper testbenches, compiled .vvp, full log.
```

### How to reproduce

```sh
# one-time: winget install --id Icarus.Verilog
export PATH="/c/iverilog/bin:$PATH"       # or add to PATH permanently
cd verify
python3 build_tests.py
```

### Test programs and latest results (11/11 pass)

| Test | Covers | Corner cases | Result |
|---|---|---|---|
| `alu_rtype` | ADD/SUB/AND/OR/XOR/SLL/SRL/SRA/SLT/SLTU (R-type) | signed overflow wrap (`0x7FFFFFFF+1`), underflow (`0-1`), shift by 0 and by 31, arithmetic-shift sign preservation, signed-vs-unsigned `SLT`/`SLTU` on `0xFFFFFFFF`, write to `x0` ignored | **PASS** — 101 cycles |
| `alu_itype` | ADDI/ANDI/ORI/XORI/SLTI/SLTIU/SLLI/SRLI/SRAI (I-type) | max positive (`2047`) / min negative (`-2048`) 12-bit immediate, negative-immediate `SLTI` vs `SLTIU` | **PASS** — 53 cycles |
| `branch` | BEQ/BNE/BLT/BGE/BLTU/BGEU, taken and not-taken | signed-vs-unsigned divergence: `-1 < 1` is true for `BLT`, false for `BLTU` (and symmetric for `BGE`/`BGEU`) | **PASS** — 67 cycles |
| `mem` | LB/LH/LW/LBU/LHU, SB/SH/SW | sign extension (`0xFF`→`-1`) vs zero extension (`0xFF`→`255`) for byte and halfword, all 4 byte offsets and both halfword offsets within a word, byte/halfword store doesn't clobber neighboring bytes (read-modify-write correctness) | **PASS** — 79 cycles |
| `jump` | JAL, JALR | return-address correctness, JALR LSB-clear on an odd target | **PASS** — 19 cycles |
| `upper` | LUI, AUIPC | `LUI` with max (`0xFFFFF`) and zero immediate, combined `LUI`+`ADDI` building an arbitrary 32-bit constant | **PASS** — 18 cycles |
| `app_gcd` | Euclidean GCD(462, 1071) by repeated subtraction | multi-iteration data-dependent branching | **PASS** — 55 cycles, result 21 |
| `app_factorial` | 6! via nested loops (repeated addition stands in for multiply — RV32I base has no `M` extension) | nested loop control flow | **PASS** — 163 cycles, result 720 |
| `app_fibonacci` | Fibonacci(15) iteratively | 15-iteration loop, register rotation | **PASS** — 105 cycles, result 610 |
| `app_sum_array` | Sum a 10-element array from data memory | address computation via `SLLI`+`ADD`, loop-carried accumulator | **PASS** — 85 cycles, result 55 |
| `app_bubble_sort` | Bubble-sort a 6-element array in place | nested loops, conditional swap, memory read-modify-write, post-sort verification of all 6 elements | **PASS** — 244 cycles |

Latest run:

```
[PASS] alu_rtype | 101 cycles | PC=00000194
[PASS] alu_itype | 53 cycles | PC=000000d4
[PASS] branch | 67 cycles | PC=00000124
[PASS] mem | 79 cycles | PC=0000013c
[PASS] jump | 19 cycles | PC=0000005c
[PASS] upper | 18 cycles | PC=00000048
[PASS] app_gcd | 55 cycles | PC=00000040
[PASS] app_factorial | 163 cycles | PC=00000068
[PASS] app_fibonacci | 105 cycles | PC=00000050
[PASS] app_sum_array | 85 cycles | PC=00000054
[PASS] app_bubble_sort | 244 cycles | PC=000000d8

--- SUMMARY ---
11/11 passed
```

### Unit-level testbenches

- **`execute_tb.v`** (rewritten — see bug #11): 19/19 checks pass, covering
  ALU ops, all 6 branch comparisons, JAL/JALR return address + target, LUI
  ignoring its garbage `rs1` field, AUIPC, and load/store address
  calculation.
- **`RISC_V.srcs/sim_1/new/id_tb.v`**: standalone decode-only unit test
  (fixed a mislabeled stimulus — the `jal x1, 1024` test vector actually
  hand-encoded an immediate of 4; corrected to the real encoding of 1024).
- **`riscv_processor_tb`** (`riscV_tb.v`, Vivado's default `sim_1` top):
  now runs the condensed default-regression program (below) and halts
  cleanly on `ECALL` instead of always running to a fixed timeout.

### Full-fileset compile check

Everything Vivado would elaborate together — all of `sources_1/new` plus
`sim_1/new` — compiles cleanly with no errors or warnings:

```sh
iverilog -g2012 -o /tmp/full_check.vvp RISC_V.srcs/sources_1/new/*.v RISC_V.srcs/sim_1/new/*.v
```

## Default Vivado simulation

`RISC_V.srcs/sources_1/new/instructions.mem` / `data.mem` (loaded by
`riscv_processor_tb`, Vivado's `sim_1` top, and by opening the project and
running the default simulation) were regenerated from
`verify/build_default_program.py` into a self-checking regression covering
every instruction category above plus Fibonacci(15), ending in `ECALL` with
a pass/fail code in `x31`. Result:

```
[PASS] default_regression | 183 cycles | PC=00000194
```

The original `instructions.mem` was a 10-instruction smoke program using only
arithmetic and one load/store — no branches, jumps, `LUI`, or `AUIPC`, i.e.
it happened to avoid every instruction category that was broken. It's been
replaced as the default simulation because it didn't exercise enough of the
ISA to catch anything; it's still recoverable from git history if needed.

## What's not covered

- No pipelining (this is a single-cycle core), so no hazards/forwarding to
  test.
- No traps/exceptions/interrupts, CSR file, or privileged modes —
  `ECALL`/`EBREAK` only raise a `ecall_halt` output for testbench use.
- No `M` (multiply/divide), `A`, `F`/`D`, or `C` extension — all application
  tests are written in pure RV32I (e.g. factorial multiplies via a loop of
  additions).
- Misaligned loads/stores that would cross a word boundary aren't supported
  (byte/halfword accesses are supported at any offset *within* a word, per
  RV32I's optional-misalignment allowance — but a halfword/word access that
  spans two words isn't handled, consistent with most small RV32I teaching
  cores).
