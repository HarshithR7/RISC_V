`timescale 1ns / 1ps
// Out-of-order RV64I+M core. Phase 1: Tomasulo + ROB, single-issue
// dispatch, out-of-order execution, in-order commit. Phase 2: branch
// prediction + speculative execution on top of that same ROB. Phase 3
// (this file): widen dispatch/rename to 2-wide superscalar. See
// RV64I_OOO's design notes (rob.v/rat.v headers, div_fu.v header) for the
// rationale behind the commit-order renaming discipline and the
// iterative divider, and see the README for the full build history.
//
// Phase 2 scope, deliberately narrower than "predict everything": only
// conditional branches get real prediction (a BHT, see bht.v) and
// speculative execution, with exactly one outstanding speculative branch
// per thread at a time (matching branch_rs.v's own single-entry design,
// so a single RAT checkpoint -- see rat.v -- is always enough; no
// checkpoint stack). JAL never speculates at all: its target is exact at
// decode (pc+imm, no register dependency), so dispatch redirects fetch to
// it immediately and unconditionally. JALR keeps Phase 1's conservative
// stall-until-resolved behavior.
//
// Phase 3 scope: dispatch and rename go to 2-wide (lane 0 = older, lane 1
// = younger), but the CDB, execution completion, and commit all stay
// exactly 1-wide per Phase 3 -- widened again in Phase 5 (see below).
// Lane 1 can never itself be branch-class; both lanes are fetched every
// cycle from straight-line addresses regardless of what lane 0 turns out
// to be; lane 1's operands get an intra-group RAW bypass against lane 0's
// own newly-allocated ROB tag when needed.
//
// Phase 5 widened the CDB and commit path to 2-wide (cdbA/cdbB, dual ROB
// retire, dual register-file write port) -- Phase 4 had measured that a
// 1-wide broadcast/retire path, not dispatch bandwidth, was the real IPC
// bottleneck.
//
// Phase 6 (DLP) added a scoped RVV vector datapath (.vv-form-only,
// single-issue, lane-0-only) alongside the scalar pipeline: vec_rat,
// vec_rs (wrapping vector_alu), vector_register_file, and a dedicated
// vec_mark broadcast bus separate from the scalar CDB.
//
// Phase 7 (this file, SMT) adds 2-thread simultaneous multithreading.
// Scope, settled after weighing depth vs. breadth against the originally
// requested 4 threads: exactly 2 threads, built with the same rigor and
// test coverage as every earlier phase, rather than 4 threads built
// thin. Design:
//   - Fully duplicated per-thread front end: PC, 2-wide fetch, 2-wide
//     decode, RAT, dual-ported architectural register file, ROB,
//     branch_rs. Each is a complete, independent instance (t0_*/t1_*
//     prefixes) -- explicit concrete instantiation, not a generate loop,
//     matching this project's stated preference (see alu_rs.v's header)
//     for concrete, purpose-built logic over maximally generic
//     abstractions.
//   - Shared, thread-tagged execution backend: alu_rs, mul_rs, div_rs,
//     lsq are still single shared instances (one alu adder shouldn't need
//     duplicating just because there are two threads), each RS entry now
//     carrying a 1-bit tid. Their issue-priority scheme was redesigned
//     from Phase 4's plain age-ordering (which has no meaningful
//     definition *across* two independent ROBs' tag spaces) to
//     "is-own-thread-ROB-head-first, else fixed lowest-index" -- see each
//     bank's own header.
//   - Two independent per-thread squash ports on every shared bank
//     (squash0_*/squash1_*), not one port muxed by a tid selector: branch
//     resolution is asynchronous to which thread is currently dispatching,
//     so both threads' outstanding branches can resolve -- and both
//     mispredict -- in the exact same cycle. A single muxed port could
//     only ever squash one of them, leaving the other thread's wrong-path
//     entries stale in a shared bank.
//   - Coarse-grained round-robin dispatch: one thread's full 2-wide
//     dispatch slate fires per cycle, chosen by an unconditionally
//     toggling `active_thread` register, not mixed-lane. This is a
//     deliberate simplicity-over-utilization tradeoff -- a thread stalled
//     on, say, an outstanding JALR still only gets skipped on its own
//     turn, so a "smarter" arbiter that let the other thread take an
//     extra turn during such a stall would claw back some throughput.
//     That arbiter was cut in favor of getting plain alternation
//     genuinely correct and well-tested, per the same "2 threads done
//     solidly" scope decision that dropped 4-thread SMT entirely; it's a
//     documented limitation, not an oversight.
//   - Almost the entire dispatch/operand-readiness combinational block
//     (unchanged from Phase 3/5/6) is reused *verbatim*, fed by "muxed"
//     views of whichever thread is `active_thread` this cycle (both
//     threads' front ends are always computed combinationally regardless
//     of whose turn it is -- only *state-changing* effects, RAT/ROB/RS
//     writes and PC advance, are gated to the active thread). Its
//     outputs (rat_write_en, rob_alloc_req, next_pc, ...) are then
//     "demuxed" back out to the correct per-thread instance.
//   - The CDB arbiter is widened from 5-way to 6-way (alu, t0's branch,
//     t1's branch, mul, div, lsq -- branch_rs is no longer a single
//     shared requester now that it's two independent per-thread
//     instances) and switched from the hand-unrolled pairwise-reduction
//     age() comparison to a procedural for-loop reduction using the same
//     "is-own-thread-head-first, else fixed lowest-index" priority as the
//     RS banks -- for the same reason: a requester's raw ROB tag age is
//     only meaningful relative to its *own* thread's head, so a numeric
//     age() comparison across two threads' requesters has no honest
//     interpretation.
//   - CDB winners route to the correct thread's ROB via their tid,
//     reusing rob.v's existing dual mark/mark_b ports (no new ROB ports
//     needed).
//   - Vector stays thread-0-only (an explicit, structurally-enforced
//     scope cut, matching Phase 6's own narrow scope): thread 1's decode
//     outputs are simply never read for is_vec, so a vector opcode
//     appearing in thread 1's instruction stream is a documented
//     limitation (it will never dispatch, the same "unhandled opcode
//     stalls, doesn't corrupt state" behavior this project already uses
//     elsewhere), not a silent correctness bug.
//   - The BHT is a single shared instance (real SMT processors commonly
//     share predictor state, and it needs no per-thread ROB-ordering
//     guarantee the way RS-bank squash does): its single update port is
//     fed by whichever thread resolves a conditional branch this cycle,
//     with thread 0 winning if both resolve the same cycle -- a second
//     documented, rare-but-real simplification, same spirit as the
//     dispatch round-robin's.
//   - Cross-thread memory consistency is explicitly out of scope: lsq.v's
//     store-vs-load disambiguation only reasons about same-thread
//     entries (see its header) -- two threads deliberately sharing
//     addresses is not guaranteed safe, the same category of scope cut
//     as this project having no MMU/virtual memory at all.
//   - Cross-thread store-commit arbitration: data_memory.v still has one
//     write port. Each thread computes its own store-commit candidate
//     exactly as before (untouched per-thread logic); a fixed-priority
//     arbiter (thread 0 always wins) then decides which thread's store,
//     if any, actually uses the port this cycle. If thread 1 loses and
//     its own *head* (not just head2) was the contended store, thread
//     1's entire commit is suppressed that cycle (not just the store),
//     since per-thread commit must stay in-order -- head2 can never
//     retire past a blocked head.
//   - Two independent instruction memories (IMEM_FILE0/IMEM_FILE1, one
//     per thread) so two genuinely independent programs can run
//     concurrently; a single shared data memory (DMEM_FILE), matching the
//     lsq.v cross-thread-memory scope note above (which only makes sense
//     if the two threads' loads/stores land in the same address space).
//
// Reused unchanged from the single-cycle RV64I core: program_counter.v,
// instruction_fetch.v, register_file.v (written only at commit).
module riscv64_ooo_proc #(
    // Phase 7: one program image per thread -- see module header for why
    // this replaced Phase 1-6's single IMEM_FILE.
    parameter IMEM_FILE0 = "instructions0.mem",
    parameter IMEM_FILE1 = "instructions1.mem",
    parameter IMEM_WORDS = 8192,
    parameter ROB_DEPTH = 8,
    parameter ALU_RS_DEPTH = 4,
    parameter MUL_RS_DEPTH = 2,
    parameter LSQ_DEPTH = 4,
    // Phase 4 benchmarking knob: forces lane 1 to never fire, turning
    // this same RTL into a single-issue machine for a direct,
    // apples-to-apples dispatch-width comparison. Applies identically to
    // both threads.
    parameter ENABLE_DUAL_ISSUE = 1,
    // Phase 6 (DLP): vector width -- SEW=32/LMUL=1 always, so VLEN =
    // LANES*32. Vector stays thread-0-only (see module header).
    parameter LANES = 4,
    parameter VEC_RS_DEPTH = 2,
    // Phase 8 (cache/MESI): this core's own private L1 data cache and
    // store buffer -- see l1_cache.v/lsq.v's headers. DMEM_FILE/
    // DMEM_WORDS are gone from this module's own parameter list: the
    // backing memory now lives behind a shared l2_cache.v, entirely
    // external to this module (see the new l2_*/snoop_* ports below) --
    // a single core no longer owns (or even can reach) memory directly.
    parameter L1_LINES = 16,
    parameter L1_LINE_BYTES = 32,
    parameter SBUF_DEPTH = 4
    // BRANCH_RS and DIV_RS are fixed at exactly 1 entry per thread -- see
    // their headers for why that's not a knob worth exposing. NUM_THREADS
    // is likewise fixed at exactly 2 -- see module header for the scope
    // decision -- not a parameter.
)(
    input clk,
    input reset,
    output wire [63:0] pc_out0,
    output wire [63:0] pc_out1,
    output wire ecall_halt0,
    output wire ecall_halt1,

    // ---- Phase 8: this core's private L1's connection to the external,
    // shared l2_cache.v -- see l1_cache.v's header for the protocol.
    output wire l2_req_valid,
    output wire [1:0] l2_req_type,
    output wire [63:0] l2_req_addr,
    output wire [L1_LINE_BYTES*8-1:0] l2_req_wb_data,
    input l2_resp_valid,
    input [L1_LINE_BYTES*8-1:0] l2_resp_data,
    input l2_resp_exclusive,

    // Coherency snoop: l2_cache.v asking this core's L1 about an address
    // the *other* core is contending for.
    input snoop_req_valid,
    input [1:0] snoop_req_type,
    input [63:0] snoop_req_addr,
    output wire snoop_resp_hit,
    output wire snoop_resp_dirty,
    output wire [L1_LINE_BYTES*8-1:0] snoop_resp_data
);
    localparam TB = $clog2(ROB_DEPTH);
    localparam VLEN = LANES * 32;

    // ---- Phase 7: dispatch round-robin -------------------------------------
    // Unconditional per-cycle toggle -- see module header for why this
    // simple alternation (not a stall-aware arbiter) was the deliberate
    // choice.
    reg active_thread;
    always @(posedge clk or posedge reset) begin
        if (reset) active_thread <= 1'b0;
        else active_thread <= ~active_thread;
    end

    // ================================================================
    // ---- Thread 0 front end: fetch, decode -------------------------
    // ================================================================
    wire [63:0] t0_pc;
    wire [63:0] t0_pc1 = t0_pc + 64'd4;
    reg  [63:0] t0_next_pc;
    wire [31:0] t0_instruction0, t0_instruction1;

    program_counter t0_pc_module (.clk(clk), .reset(reset), .pc_in(t0_next_pc), .pc_out(t0_pc));
    instruction_fetch #(.IMEM_FILE(IMEM_FILE0), .IMEM_WORDS(IMEM_WORDS)) t0_if0 (
        .clk(clk), .pc(t0_pc), .instruction(t0_instruction0)
    );
    instruction_fetch #(.IMEM_FILE(IMEM_FILE0), .IMEM_WORDS(IMEM_WORDS)) t0_if1 (
        .clk(clk), .pc(t0_pc1), .instruction(t0_instruction1)
    );
    assign pc_out0 = t0_pc;

    wire [4:0] t0_d0_rs1, t0_d0_rs2, t0_d0_rd;
    wire [2:0] t0_d0_func3;
    wire [63:0] t0_d0_imm;
    wire t0_d0_is_alu, t0_d0_is_muldiv, t0_d0_is_branch, t0_d0_is_jal, t0_d0_is_jalr, t0_d0_is_load, t0_d0_is_store, t0_d0_is_ecall;
    wire [3:0] t0_d0_alu_op;
    wire t0_d0_word_op;
    wire [2:0] t0_d0_muldiv_op;
    wire t0_d0_reg_write, t0_d0_src2_is_imm, t0_d0_src1_is_zero, t0_d0_src1_is_pc;
    wire t0_d0_is_vec;
    wire [4:0] t0_d0_v_op;
    wire t0_d0_is_vmv;

    decode_ooo t0_dec0 (
        .instruction(t0_instruction0),
        .rs1(t0_d0_rs1), .rs2(t0_d0_rs2), .rd(t0_d0_rd), .func3(t0_d0_func3), .imm(t0_d0_imm),
        .is_alu(t0_d0_is_alu), .is_muldiv(t0_d0_is_muldiv), .is_branch(t0_d0_is_branch),
        .is_jal(t0_d0_is_jal), .is_jalr(t0_d0_is_jalr), .is_load(t0_d0_is_load), .is_store(t0_d0_is_store),
        .is_ecall(t0_d0_is_ecall),
        .alu_op(t0_d0_alu_op), .word_op(t0_d0_word_op), .muldiv_op(t0_d0_muldiv_op),
        .reg_write(t0_d0_reg_write), .src2_is_imm(t0_d0_src2_is_imm),
        .src1_is_zero(t0_d0_src1_is_zero), .src1_is_pc(t0_d0_src1_is_pc),
        .is_vec(t0_d0_is_vec), .v_op(t0_d0_v_op), .is_vmv(t0_d0_is_vmv)
    );

    wire [4:0] t0_d1_rs1, t0_d1_rs2, t0_d1_rd;
    wire [2:0] t0_d1_func3;
    wire [63:0] t0_d1_imm;
    wire t0_d1_is_alu, t0_d1_is_muldiv, t0_d1_is_branch, t0_d1_is_jal, t0_d1_is_jalr, t0_d1_is_load, t0_d1_is_store, t0_d1_is_ecall;
    wire [3:0] t0_d1_alu_op;
    wire t0_d1_word_op;
    wire [2:0] t0_d1_muldiv_op;
    wire t0_d1_reg_write, t0_d1_src2_is_imm, t0_d1_src1_is_zero, t0_d1_src1_is_pc;

    decode_ooo t0_dec1 (
        .instruction(t0_instruction1),
        .rs1(t0_d1_rs1), .rs2(t0_d1_rs2), .rd(t0_d1_rd), .func3(t0_d1_func3), .imm(t0_d1_imm),
        .is_alu(t0_d1_is_alu), .is_muldiv(t0_d1_is_muldiv), .is_branch(t0_d1_is_branch),
        .is_jal(t0_d1_is_jal), .is_jalr(t0_d1_is_jalr), .is_load(t0_d1_is_load), .is_store(t0_d1_is_store),
        .is_ecall(t0_d1_is_ecall),
        .alu_op(t0_d1_alu_op), .word_op(t0_d1_word_op), .muldiv_op(t0_d1_muldiv_op),
        .reg_write(t0_d1_reg_write), .src2_is_imm(t0_d1_src2_is_imm),
        .src1_is_zero(t0_d1_src1_is_zero), .src1_is_pc(t0_d1_src1_is_pc)
    );

    // ================================================================
    // ---- Thread 1 front end: fetch, decode (no vector support) -----
    // ================================================================
    wire [63:0] t1_pc;
    wire [63:0] t1_pc1 = t1_pc + 64'd4;
    reg  [63:0] t1_next_pc;
    wire [31:0] t1_instruction0, t1_instruction1;

    program_counter t1_pc_module (.clk(clk), .reset(reset), .pc_in(t1_next_pc), .pc_out(t1_pc));
    instruction_fetch #(.IMEM_FILE(IMEM_FILE1), .IMEM_WORDS(IMEM_WORDS)) t1_if0 (
        .clk(clk), .pc(t1_pc), .instruction(t1_instruction0)
    );
    instruction_fetch #(.IMEM_FILE(IMEM_FILE1), .IMEM_WORDS(IMEM_WORDS)) t1_if1 (
        .clk(clk), .pc(t1_pc1), .instruction(t1_instruction1)
    );
    assign pc_out1 = t1_pc;

    wire [4:0] t1_d0_rs1, t1_d0_rs2, t1_d0_rd;
    wire [2:0] t1_d0_func3;
    wire [63:0] t1_d0_imm;
    wire t1_d0_is_alu, t1_d0_is_muldiv, t1_d0_is_branch, t1_d0_is_jal, t1_d0_is_jalr, t1_d0_is_load, t1_d0_is_store, t1_d0_is_ecall;
    wire [3:0] t1_d0_alu_op;
    wire t1_d0_word_op;
    wire [2:0] t1_d0_muldiv_op;
    wire t1_d0_reg_write, t1_d0_src2_is_imm, t1_d0_src1_is_zero, t1_d0_src1_is_pc;

    // Deliberately NOT wired: is_vec/v_op/is_vmv -- vector is thread-0-only
    // (see module header). Leaving these outputs unconnected is a safe
    // Verilog no-op (nothing ever reads them); the muxed d0_is_vec below is
    // hard-tied to 0 whenever active_thread==1, so even if this decoder
    // happened to classify some instruction as vector-class, it would
    // structurally never be allowed to dispatch as one.
    decode_ooo t1_dec0 (
        .instruction(t1_instruction0),
        .rs1(t1_d0_rs1), .rs2(t1_d0_rs2), .rd(t1_d0_rd), .func3(t1_d0_func3), .imm(t1_d0_imm),
        .is_alu(t1_d0_is_alu), .is_muldiv(t1_d0_is_muldiv), .is_branch(t1_d0_is_branch),
        .is_jal(t1_d0_is_jal), .is_jalr(t1_d0_is_jalr), .is_load(t1_d0_is_load), .is_store(t1_d0_is_store),
        .is_ecall(t1_d0_is_ecall),
        .alu_op(t1_d0_alu_op), .word_op(t1_d0_word_op), .muldiv_op(t1_d0_muldiv_op),
        .reg_write(t1_d0_reg_write), .src2_is_imm(t1_d0_src2_is_imm),
        .src1_is_zero(t1_d0_src1_is_zero), .src1_is_pc(t1_d0_src1_is_pc)
    );

    wire [4:0] t1_d1_rs1, t1_d1_rs2, t1_d1_rd;
    wire [2:0] t1_d1_func3;
    wire [63:0] t1_d1_imm;
    wire t1_d1_is_alu, t1_d1_is_muldiv, t1_d1_is_branch, t1_d1_is_jal, t1_d1_is_jalr, t1_d1_is_load, t1_d1_is_store, t1_d1_is_ecall;
    wire [3:0] t1_d1_alu_op;
    wire t1_d1_word_op;
    wire [2:0] t1_d1_muldiv_op;
    wire t1_d1_reg_write, t1_d1_src2_is_imm, t1_d1_src1_is_zero, t1_d1_src1_is_pc;

    decode_ooo t1_dec1 (
        .instruction(t1_instruction1),
        .rs1(t1_d1_rs1), .rs2(t1_d1_rs2), .rd(t1_d1_rd), .func3(t1_d1_func3), .imm(t1_d1_imm),
        .is_alu(t1_d1_is_alu), .is_muldiv(t1_d1_is_muldiv), .is_branch(t1_d1_is_branch),
        .is_jal(t1_d1_is_jal), .is_jalr(t1_d1_is_jalr), .is_load(t1_d1_is_load), .is_store(t1_d1_is_store),
        .is_ecall(t1_d1_is_ecall),
        .alu_op(t1_d1_alu_op), .word_op(t1_d1_word_op), .muldiv_op(t1_d1_muldiv_op),
        .reg_write(t1_d1_reg_write), .src2_is_imm(t1_d1_src2_is_imm),
        .src1_is_zero(t1_d1_src1_is_zero), .src1_is_pc(t1_d1_src1_is_pc)
    );

    // ================================================================
    // ---- Active-thread mux: the "shared dispatch logic" view -------
    // ================================================================
    // Both threads' front ends are always computed combinationally
    // (above); this section just picks out whichever thread is
    // `active_thread` this cycle, under the SAME variable names Phase
    // 1-6's dispatch logic already used -- so that logic (further below)
    // is reused completely unchanged, just now consuming a per-cycle
    // muxed view instead of a single thread's own wires.
    wire [63:0] pc      = active_thread ? t1_pc      : t0_pc;
    wire [63:0] pc1     = active_thread ? t1_pc1     : t0_pc1;

    wire [4:0] d0_rs1 = active_thread ? t1_d0_rs1 : t0_d0_rs1;
    wire [4:0] d0_rs2 = active_thread ? t1_d0_rs2 : t0_d0_rs2;
    wire [4:0] d0_rd  = active_thread ? t1_d0_rd  : t0_d0_rd;
    wire [2:0] d0_func3 = active_thread ? t1_d0_func3 : t0_d0_func3;
    wire [63:0] d0_imm  = active_thread ? t1_d0_imm  : t0_d0_imm;
    wire d0_is_alu    = active_thread ? t1_d0_is_alu    : t0_d0_is_alu;
    wire d0_is_muldiv = active_thread ? t1_d0_is_muldiv : t0_d0_is_muldiv;
    wire d0_is_branch = active_thread ? t1_d0_is_branch : t0_d0_is_branch;
    wire d0_is_jal    = active_thread ? t1_d0_is_jal    : t0_d0_is_jal;
    wire d0_is_jalr   = active_thread ? t1_d0_is_jalr   : t0_d0_is_jalr;
    wire d0_is_load   = active_thread ? t1_d0_is_load   : t0_d0_is_load;
    wire d0_is_store  = active_thread ? t1_d0_is_store  : t0_d0_is_store;
    wire d0_is_ecall  = active_thread ? t1_d0_is_ecall  : t0_d0_is_ecall;
    wire [3:0] d0_alu_op = active_thread ? t1_d0_alu_op : t0_d0_alu_op;
    wire d0_word_op       = active_thread ? t1_d0_word_op   : t0_d0_word_op;
    wire [2:0] d0_muldiv_op = active_thread ? t1_d0_muldiv_op : t0_d0_muldiv_op;
    wire d0_reg_write    = active_thread ? t1_d0_reg_write    : t0_d0_reg_write;
    wire d0_src2_is_imm  = active_thread ? t1_d0_src2_is_imm  : t0_d0_src2_is_imm;
    wire d0_src1_is_zero = active_thread ? t1_d0_src1_is_zero : t0_d0_src1_is_zero;
    wire d0_src1_is_pc   = active_thread ? t1_d0_src1_is_pc   : t0_d0_src1_is_pc;
    // Vector: thread-0-only, structurally enforced -- see module header.
    wire d0_is_vec = (active_thread == 1'b0) ? t0_d0_is_vec : 1'b0;
    wire [4:0] d0_v_op = (active_thread == 1'b0) ? t0_d0_v_op : 5'd0;
    wire d0_is_vmv = (active_thread == 1'b0) ? t0_d0_is_vmv : 1'b0;

    wire [4:0] d1_rs1 = active_thread ? t1_d1_rs1 : t0_d1_rs1;
    wire [4:0] d1_rs2 = active_thread ? t1_d1_rs2 : t0_d1_rs2;
    wire [4:0] d1_rd  = active_thread ? t1_d1_rd  : t0_d1_rd;
    wire [2:0] d1_func3 = active_thread ? t1_d1_func3 : t0_d1_func3;
    wire [63:0] d1_imm  = active_thread ? t1_d1_imm  : t0_d1_imm;
    wire d1_is_alu    = active_thread ? t1_d1_is_alu    : t0_d1_is_alu;
    wire d1_is_muldiv = active_thread ? t1_d1_is_muldiv : t0_d1_is_muldiv;
    wire d1_is_load   = active_thread ? t1_d1_is_load   : t0_d1_is_load;
    wire d1_is_store  = active_thread ? t1_d1_is_store  : t0_d1_is_store;
    wire [3:0] d1_alu_op = active_thread ? t1_d1_alu_op : t0_d1_alu_op;
    wire d1_word_op       = active_thread ? t1_d1_word_op   : t0_d1_word_op;
    wire [2:0] d1_muldiv_op = active_thread ? t1_d1_muldiv_op : t0_d1_muldiv_op;
    wire d1_reg_write    = active_thread ? t1_d1_reg_write    : t0_d1_reg_write;
    wire d1_src2_is_imm  = active_thread ? t1_d1_src2_is_imm  : t0_d1_src2_is_imm;
    wire d1_src1_is_zero = active_thread ? t1_d1_src1_is_zero : t0_d1_src1_is_zero;
    wire d1_src1_is_pc   = active_thread ? t1_d1_src1_is_pc   : t0_d1_src1_is_pc;

    wire d0_is_mul = d0_is_muldiv && !d0_muldiv_op[2];
    wire d0_is_div = d0_is_muldiv && d0_muldiv_op[2];
    wire d1_is_mul = d1_is_muldiv && !d1_muldiv_op[2];
    wire d1_is_div = d1_is_muldiv && d1_muldiv_op[2];

    // ================================================================
    // ---- Register renaming (RAT) + architectural register file -----
    // ================================================================
    // Per-thread instances, each continuously reading its OWN raw
    // (un-muxed) decode outputs every cycle -- reads are free and a
    // thread's rename state must be ready to use the instant it becomes
    // `active_thread`, not one cycle later. Only the WRITE side
    // (write_en/write2_en) is gated to the active thread, via the demux
    // section further below.
    wire t0_rs1_busy, t0_rs2_busy;
    wire [TB-1:0] t0_rs1_tag, t0_rs2_tag;
    wire t0_rs1b_busy_raw, t0_rs2b_busy_raw;
    wire [TB-1:0] t0_rs1b_tag_raw, t0_rs2b_tag_raw;
    wire t0_rat_write_en, t0_rat_write2_en;
    wire t0_rat_commit_clear_en, t0_rat_commit_clear_en2;
    wire [4:0] t0_rat_commit_rd, t0_rat_commit_rd2;
    wire [TB-1:0] t0_rat_commit_tag, t0_rat_commit_tag2;

    rat #(.TAG_BITS(TB)) t0_rat_i (
        .clk(clk), .reset(reset),
        .rs1(t0_d0_rs1), .rs2(t0_d0_rs2),
        .rs1_busy(t0_rs1_busy), .rs1_tag(t0_rs1_tag),
        .rs2_busy(t0_rs2_busy), .rs2_tag(t0_rs2_tag),
        .rs1b(t0_d1_rs1), .rs2b(t0_d1_rs2),
        .rs1b_busy(t0_rs1b_busy_raw), .rs1b_tag(t0_rs1b_tag_raw),
        .rs2b_busy(t0_rs2b_busy_raw), .rs2b_tag(t0_rs2b_tag_raw),
        .write_en(t0_rat_write_en), .rd(t0_d0_rd), .new_tag(rat_new_tag),
        .write2_en(t0_rat_write2_en), .rd2(t0_d1_rd), .new_tag2(rat_new_tag2),
        .commit_clear_en(t0_rat_commit_clear_en), .commit_rd(t0_rat_commit_rd), .commit_tag(t0_rat_commit_tag),
        .commit_clear_en2(t0_rat_commit_clear_en2), .commit_rd2(t0_rat_commit_rd2), .commit_tag2(t0_rat_commit_tag2),
        .checkpoint_save(t0_rat_checkpoint_save), .checkpoint_restore(t0_mispredict)
    );

    wire t1_rs1_busy, t1_rs2_busy;
    wire [TB-1:0] t1_rs1_tag, t1_rs2_tag;
    wire t1_rs1b_busy_raw, t1_rs2b_busy_raw;
    wire [TB-1:0] t1_rs1b_tag_raw, t1_rs2b_tag_raw;
    wire t1_rat_write_en, t1_rat_write2_en;
    wire t1_rat_commit_clear_en, t1_rat_commit_clear_en2;
    wire [4:0] t1_rat_commit_rd, t1_rat_commit_rd2;
    wire [TB-1:0] t1_rat_commit_tag, t1_rat_commit_tag2;

    rat #(.TAG_BITS(TB)) t1_rat_i (
        .clk(clk), .reset(reset),
        .rs1(t1_d0_rs1), .rs2(t1_d0_rs2),
        .rs1_busy(t1_rs1_busy), .rs1_tag(t1_rs1_tag),
        .rs2_busy(t1_rs2_busy), .rs2_tag(t1_rs2_tag),
        .rs1b(t1_d1_rs1), .rs2b(t1_d1_rs2),
        .rs1b_busy(t1_rs1b_busy_raw), .rs1b_tag(t1_rs1b_tag_raw),
        .rs2b_busy(t1_rs2b_busy_raw), .rs2b_tag(t1_rs2b_tag_raw),
        .write_en(t1_rat_write_en), .rd(t1_d0_rd), .new_tag(rat_new_tag),
        .write2_en(t1_rat_write2_en), .rd2(t1_d1_rd), .new_tag2(rat_new_tag2),
        .commit_clear_en(t1_rat_commit_clear_en), .commit_rd(t1_rat_commit_rd), .commit_tag(t1_rat_commit_tag),
        .commit_clear_en2(t1_rat_commit_clear_en2), .commit_rd2(t1_rat_commit_rd2), .commit_tag2(t1_rat_commit_tag2),
        .checkpoint_save(t1_rat_checkpoint_save), .checkpoint_restore(t1_mispredict)
    );

    wire rs1_busy       = active_thread ? t1_rs1_busy       : t0_rs1_busy;
    wire [TB-1:0] rs1_tag = active_thread ? t1_rs1_tag       : t0_rs1_tag;
    wire rs2_busy        = active_thread ? t1_rs2_busy       : t0_rs2_busy;
    wire [TB-1:0] rs2_tag = active_thread ? t1_rs2_tag       : t0_rs2_tag;
    wire rs1b_busy_raw    = active_thread ? t1_rs1b_busy_raw : t0_rs1b_busy_raw;
    wire [TB-1:0] rs1b_tag_raw = active_thread ? t1_rs1b_tag_raw : t0_rs1b_tag_raw;
    wire rs2b_busy_raw    = active_thread ? t1_rs2b_busy_raw : t0_rs2b_busy_raw;
    wire [TB-1:0] rs2b_tag_raw = active_thread ? t1_rs2b_tag_raw : t0_rs2b_tag_raw;

    // Widened-commit support (Phase 5): both register_file instances per
    // thread get a second write port, fed identically.
    wire [63:0] t0_rf0_read1, t0_rf0_read2, t0_rf1_read1, t0_rf1_read2;
    register_file t0_regfile0 (
        .clk(clk), .reset(reset),
        .reg_write(t0_commit_rf_write_en),
        .read_reg1(t0_d0_rs1), .read_reg2(t0_d0_rs2), .write_reg(t0_commit_rf_write_reg),
        .write_data(t0_commit_rf_write_data),
        .read_data1(t0_rf0_read1), .read_data2(t0_rf0_read2),
        .reg_write2(t0_commit_rf_write_en2), .write_reg2(t0_commit_rf_write_reg2), .write_data2(t0_commit_rf_write_data2)
    );
    register_file t0_regfile1 (
        .clk(clk), .reset(reset),
        .reg_write(t0_commit_rf_write_en),
        .read_reg1(t0_d1_rs1), .read_reg2(t0_d1_rs2), .write_reg(t0_commit_rf_write_reg),
        .write_data(t0_commit_rf_write_data),
        .read_data1(t0_rf1_read1), .read_data2(t0_rf1_read2),
        .reg_write2(t0_commit_rf_write_en2), .write_reg2(t0_commit_rf_write_reg2), .write_data2(t0_commit_rf_write_data2)
    );

    wire [63:0] t1_rf0_read1, t1_rf0_read2, t1_rf1_read1, t1_rf1_read2;
    register_file t1_regfile0 (
        .clk(clk), .reset(reset),
        .reg_write(t1_commit_rf_write_en),
        .read_reg1(t1_d0_rs1), .read_reg2(t1_d0_rs2), .write_reg(t1_commit_rf_write_reg),
        .write_data(t1_commit_rf_write_data),
        .read_data1(t1_rf0_read1), .read_data2(t1_rf0_read2),
        .reg_write2(t1_commit_rf_write_en2), .write_reg2(t1_commit_rf_write_reg2), .write_data2(t1_commit_rf_write_data2)
    );
    register_file t1_regfile1 (
        .clk(clk), .reset(reset),
        .reg_write(t1_commit_rf_write_en),
        .read_reg1(t1_d1_rs1), .read_reg2(t1_d1_rs2), .write_reg(t1_commit_rf_write_reg),
        .write_data(t1_commit_rf_write_data),
        .read_data1(t1_rf1_read1), .read_data2(t1_rf1_read2),
        .reg_write2(t1_commit_rf_write_en2), .write_reg2(t1_commit_rf_write_reg2), .write_data2(t1_commit_rf_write_data2)
    );

    wire [63:0] rf_read1  = active_thread ? t1_rf0_read1 : t0_rf0_read1;
    wire [63:0] rf_read2  = active_thread ? t1_rf0_read2 : t0_rf0_read2;
    wire [63:0] rf1_read1 = active_thread ? t1_rf1_read1 : t0_rf1_read1;
    wire [63:0] rf1_read2 = active_thread ? t1_rf1_read2 : t0_rf1_read2;

    // ---- Vector register renaming (vec_rat) + vector register file --------
    // Thread-0-only (see module header): reads/writes always use thread
    // 0's own signals directly, never the active-thread mux -- a
    // misprediction on thread 0 must roll back vector state regardless of
    // whose dispatch turn it currently is.
    wire vs1_busy, vs2_busy;
    wire [TB-1:0] vs1_tag, vs2_tag;
    wire vec_write_en;
    wire vec_commit_clear_en_w;
    wire [4:0] vec_commit_clear_rd_w;
    wire [TB-1:0] vec_commit_clear_tag_w;

    vec_rat #(.TAG_BITS(TB)) vec_rat_i (
        .clk(clk), .reset(reset),
        .vs1(t0_d0_rs1), .vs2(t0_d0_rs2),
        .vs1_busy(vs1_busy), .vs1_tag(vs1_tag),
        .vs2_busy(vs2_busy), .vs2_tag(vs2_tag),
        .write_en(vec_write_en), .vd(t0_d0_rd), .new_tag(t0_rob_alloc_tag),
        .commit_clear_en(vec_commit_clear_en_w), .commit_vd(vec_commit_clear_rd_w), .commit_tag(vec_commit_clear_tag_w),
        .checkpoint_save(t0_rat_checkpoint_save), .checkpoint_restore(t0_mispredict)
    );

    wire [VLEN-1:0] vreg_read1, vreg_read2;
    wire vec_commit_write_en_w;
    wire [4:0] vec_commit_write_reg_w;
    wire [VLEN-1:0] vec_commit_write_data_w;
    vector_register_file #(.LANES(LANES), .VLEN(VLEN)) vregfile_i (
        .clk(clk), .reset(reset),
        .vreg_write(vec_commit_write_en_w),
        .read_reg1(t0_d0_rs2), .read_reg2(t0_d0_rs1),
        .write_reg(vec_commit_write_reg_w), .write_data(vec_commit_write_data_w),
        .read_data1(vreg_read1), .read_data2(vreg_read2)
    );

    // ================================================================
    // ---- Reorder buffers (one full instance per thread) ------------
    // ================================================================
    wire t0_rob_alloc_req, t0_rob_alloc2_req;
    wire [TB-1:0] t0_rob_alloc_tag, t0_rob_alloc2_tag;
    wire [TB:0] t0_rob_free_count;
    wire t0_rob_mark_valid, t0_rob_mark_b_valid;
    wire [TB-1:0] t0_rob_mark_tag, t0_rob_mark_b_tag;
    wire [63:0] t0_rob_mark_value, t0_rob_mark_b_value;
    wire t0_rob_mark2_valid;
    wire [TB-1:0] t0_rob_mark2_tag;
    wire t0_rob_head_ready;
    wire [TB-1:0] t0_rob_head_tag;
    wire t0_rob_head_has_dest;
    wire [4:0] t0_rob_head_rd;
    wire [63:0] t0_rob_head_value;
    wire [VLEN-1:0] t0_rob_head_vec_value;
    wire t0_rob_head_is_vec_dest;
    wire t0_rob_head_is_store, t0_rob_head_is_ecall;
    wire t0_rob_commit_req;
    wire t0_rob_head2_ready;
    wire [TB-1:0] t0_rob_head2_tag;
    wire t0_rob_head2_has_dest;
    wire [4:0] t0_rob_head2_rd;
    wire [63:0] t0_rob_head2_value;
    wire [VLEN-1:0] t0_rob_head2_vec_value;
    wire t0_rob_head2_is_vec_dest;
    wire t0_rob_head2_is_store, t0_rob_head2_is_ecall;
    wire t0_rob_commit_req2;
    wire t0_rob_rs1_done, t0_rob_rs2_done, t0_rob_rs1b_done, t0_rob_rs2b_done;
    wire [63:0] t0_rob_rs1_value, t0_rob_rs2_value, t0_rob_rs1b_value, t0_rob_rs2b_value;
    wire t0_rob_vs1_done, t0_rob_vs2_done;
    wire [VLEN-1:0] t0_rob_vs1_value, t0_rob_vs2_value;

    rob #(.DEPTH(ROB_DEPTH), .EXTRA_MARK_N(LSQ_DEPTH), .VLEN(VLEN)) t0_rob_i (
        .clk(clk), .reset(reset),
        .alloc_req(t0_rob_alloc_req), .alloc_has_dest(t0_d0_reg_write), .alloc_rd(t0_d0_rd),
        .alloc_is_store(t0_d0_is_store), .alloc_is_ecall(t0_d0_is_ecall), .alloc_is_vec_dest(t0_d0_is_vec),
        .alloc_tag(t0_rob_alloc_tag),
        .alloc2_req(t0_rob_alloc2_req), .alloc2_has_dest(t0_d1_reg_write), .alloc2_rd(t0_d1_rd),
        .alloc2_is_store(t0_d1_is_store), .alloc2_is_ecall(t0_d1_is_ecall), .alloc2_is_vec_dest(1'b0),
        .alloc2_tag(t0_rob_alloc2_tag), .free_count(t0_rob_free_count),
        .mark_valid(t0_rob_mark_valid), .mark_tag(t0_rob_mark_tag), .mark_value(t0_rob_mark_value),
        .mark_b_valid(t0_rob_mark_b_valid), .mark_b_tag(t0_rob_mark_b_tag), .mark_b_value(t0_rob_mark_b_value),
        .vec_mark_valid(vec_mark_valid), .vec_mark_tag(vec_mark_tag), .vec_mark_value(vec_mark_value),
        .mark2_valid(t0_rob_mark2_valid), .mark2_tag(t0_rob_mark2_tag),
        .extra_mark_valid(t0_lsq_extra_mark_valid), .extra_mark_tag_flat(lsq_store_ready_tag_flat),
        .squash_valid(t0_mispredict), .squash_tag(t0_branch_resolved_tag),
        .lookup1_tag(t0_rs1_tag), .lookup1_done(t0_rob_rs1_done), .lookup1_value(t0_rob_rs1_value),
        .lookup2_tag(t0_rs2_tag), .lookup2_done(t0_rob_rs2_done), .lookup2_value(t0_rob_rs2_value),
        .lookup3_tag(t0_rs1b_tag_raw), .lookup3_done(t0_rob_rs1b_done), .lookup3_value(t0_rob_rs1b_value),
        .lookup4_tag(t0_rs2b_tag_raw), .lookup4_done(t0_rob_rs2b_done), .lookup4_value(t0_rob_rs2b_value),
        .vec_lookup1_tag(vs1_tag), .vec_lookup1_done(t0_rob_vs1_done), .vec_lookup1_value(t0_rob_vs1_value),
        .vec_lookup2_tag(vs2_tag), .vec_lookup2_done(t0_rob_vs2_done), .vec_lookup2_value(t0_rob_vs2_value),
        .head_ready(t0_rob_head_ready), .head_tag(t0_rob_head_tag), .head_has_dest(t0_rob_head_has_dest),
        .head_rd(t0_rob_head_rd), .head_value(t0_rob_head_value),
        .head_vec_value(t0_rob_head_vec_value), .head_is_vec_dest(t0_rob_head_is_vec_dest),
        .head_is_store(t0_rob_head_is_store), .head_is_ecall(t0_rob_head_is_ecall),
        .commit_req(t0_rob_commit_req),
        .head2_ready(t0_rob_head2_ready), .head2_tag(t0_rob_head2_tag), .head2_has_dest(t0_rob_head2_has_dest),
        .head2_rd(t0_rob_head2_rd), .head2_value(t0_rob_head2_value),
        .head2_vec_value(t0_rob_head2_vec_value), .head2_is_vec_dest(t0_rob_head2_is_vec_dest),
        .head2_is_store(t0_rob_head2_is_store), .head2_is_ecall(t0_rob_head2_is_ecall),
        .commit_req2(t0_rob_commit_req2)
    );

    wire t1_rob_alloc_req, t1_rob_alloc2_req;
    wire [TB-1:0] t1_rob_alloc_tag, t1_rob_alloc2_tag;
    wire [TB:0] t1_rob_free_count;
    wire t1_rob_mark_valid, t1_rob_mark_b_valid;
    wire [TB-1:0] t1_rob_mark_tag, t1_rob_mark_b_tag;
    wire [63:0] t1_rob_mark_value, t1_rob_mark_b_value;
    wire t1_rob_mark2_valid;
    wire [TB-1:0] t1_rob_mark2_tag;
    wire t1_rob_head_ready;
    wire [TB-1:0] t1_rob_head_tag;
    wire t1_rob_head_has_dest;
    wire [4:0] t1_rob_head_rd;
    wire [63:0] t1_rob_head_value;
    wire t1_rob_head_is_store, t1_rob_head_is_ecall;
    wire t1_rob_commit_req;
    wire t1_rob_head2_ready;
    wire [TB-1:0] t1_rob_head2_tag;
    wire t1_rob_head2_has_dest;
    wire [4:0] t1_rob_head2_rd;
    wire [63:0] t1_rob_head2_value;
    wire t1_rob_head2_is_store, t1_rob_head2_is_ecall;
    wire t1_rob_commit_req2;
    wire t1_rob_rs1_done, t1_rob_rs2_done, t1_rob_rs1b_done, t1_rob_rs2b_done;
    wire [63:0] t1_rob_rs1_value, t1_rob_rs2_value, t1_rob_rs1b_value, t1_rob_rs2b_value;

    rob #(.DEPTH(ROB_DEPTH), .EXTRA_MARK_N(LSQ_DEPTH), .VLEN(VLEN)) t1_rob_i (
        .clk(clk), .reset(reset),
        .alloc_req(t1_rob_alloc_req), .alloc_has_dest(t1_d0_reg_write), .alloc_rd(t1_d0_rd),
        .alloc_is_store(t1_d0_is_store), .alloc_is_ecall(t1_d0_is_ecall), .alloc_is_vec_dest(1'b0),
        .alloc_tag(t1_rob_alloc_tag),
        .alloc2_req(t1_rob_alloc2_req), .alloc2_has_dest(t1_d1_reg_write), .alloc2_rd(t1_d1_rd),
        .alloc2_is_store(t1_d1_is_store), .alloc2_is_ecall(t1_d1_is_ecall), .alloc2_is_vec_dest(1'b0),
        .alloc2_tag(t1_rob_alloc2_tag), .free_count(t1_rob_free_count),
        .mark_valid(t1_rob_mark_valid), .mark_tag(t1_rob_mark_tag), .mark_value(t1_rob_mark_value),
        .mark_b_valid(t1_rob_mark_b_valid), .mark_b_tag(t1_rob_mark_b_tag), .mark_b_value(t1_rob_mark_b_value),
        .mark2_valid(t1_rob_mark2_valid), .mark2_tag(t1_rob_mark2_tag),
        .extra_mark_valid(t1_lsq_extra_mark_valid), .extra_mark_tag_flat(lsq_store_ready_tag_flat),
        .squash_valid(t1_mispredict), .squash_tag(t1_branch_resolved_tag),
        .lookup1_tag(t1_rs1_tag), .lookup1_done(t1_rob_rs1_done), .lookup1_value(t1_rob_rs1_value),
        .lookup2_tag(t1_rs2_tag), .lookup2_done(t1_rob_rs2_done), .lookup2_value(t1_rob_rs2_value),
        .lookup3_tag(t1_rs1b_tag_raw), .lookup3_done(t1_rob_rs1b_done), .lookup3_value(t1_rob_rs1b_value),
        .lookup4_tag(t1_rs2b_tag_raw), .lookup4_done(t1_rob_rs2b_done), .lookup4_value(t1_rob_rs2b_value),
        .head_ready(t1_rob_head_ready), .head_tag(t1_rob_head_tag), .head_has_dest(t1_rob_head_has_dest),
        .head_rd(t1_rob_head_rd), .head_value(t1_rob_head_value),
        .head_is_store(t1_rob_head_is_store), .head_is_ecall(t1_rob_head_is_ecall),
        .commit_req(t1_rob_commit_req),
        .head2_ready(t1_rob_head2_ready), .head2_tag(t1_rob_head2_tag), .head2_has_dest(t1_rob_head2_has_dest),
        .head2_rd(t1_rob_head2_rd), .head2_value(t1_rob_head2_value),
        .head2_is_store(t1_rob_head2_is_store), .head2_is_ecall(t1_rob_head2_is_ecall),
        .commit_req2(t1_rob_commit_req2)
    );

    wire rob_rs1_done  = active_thread ? t1_rob_rs1_done  : t0_rob_rs1_done;
    wire [63:0] rob_rs1_value = active_thread ? t1_rob_rs1_value : t0_rob_rs1_value;
    wire rob_rs2_done  = active_thread ? t1_rob_rs2_done  : t0_rob_rs2_done;
    wire [63:0] rob_rs2_value = active_thread ? t1_rob_rs2_value : t0_rob_rs2_value;
    wire rob_rs1b_done = active_thread ? t1_rob_rs1b_done : t0_rob_rs1b_done;
    wire [63:0] rob_rs1b_value = active_thread ? t1_rob_rs1b_value : t0_rob_rs1b_value;
    wire rob_rs2b_done = active_thread ? t1_rob_rs2b_done : t0_rob_rs2b_done;
    wire [63:0] rob_rs2b_value = active_thread ? t1_rob_rs2b_value : t0_rob_rs2b_value;

    wire [TB-1:0] rob_alloc_tag  = active_thread ? t1_rob_alloc_tag  : t0_rob_alloc_tag;
    wire [TB-1:0] rob_alloc2_tag = active_thread ? t1_rob_alloc2_tag : t0_rob_alloc2_tag;
    wire [TB:0] rob_free_count   = active_thread ? t1_rob_free_count : t0_rob_free_count;

    // ================================================================
    // ---- CDB arbiter (Phase 7: 6-way, is-own-thread-head-first) ----
    // ================================================================
    // Requesters: alu, t0's branch, t1's branch, mul, div, lsq. alu/mul/
    // div/lsq are shared banks (their entries, and so their req_tid
    // output, may belong to either thread); branch_rs is no longer a
    // single shared requester now that it's two independent per-thread
    // instances (see module header), so it contributes two fixed-tid
    // requesters instead of one.
    //
    // Priority: a requester whose tag is its OWN thread's current ROB
    // head wins outright (this is what keeps in-order commit from ever
    // being needlessly delayed); otherwise fixed lowest-index order among
    // {alu, t0_branch, t1_branch, mul, div, lsq}. This replaces Phase 1-6's
    // plain age()-relative-to-head comparison, which has no honest
    // definition across two independent ROBs' tag spaces -- exactly the
    // same reasoning, and the same fallback scheme, as alu_rs.v's
    // entry_is_head-based issue priority.
    function is_head;
        input t;
        input [TB-1:0] tg;
        begin
            is_head = t ? (tg == t1_rob_head_tag) : (tg == t0_rob_head_tag);
        end
    endfunction

    localparam NREQ = 6;
    reg          arb_v  [0:NREQ-1];
    reg          arb_t  [0:NREQ-1];
    reg [TB-1:0] arb_tg [0:NREQ-1];
    reg [63:0]   arb_vl [0:NREQ-1];
    always @(*) begin
        arb_v[0]=alu_req_valid;       arb_t[0]=alu_req_tid;       arb_tg[0]=alu_req_tag;       arb_vl[0]=alu_req_value;
        arb_v[1]=t0_branch_req_valid; arb_t[1]=1'b0;              arb_tg[1]=t0_branch_req_tag; arb_vl[1]=t0_branch_req_value;
        arb_v[2]=t1_branch_req_valid; arb_t[2]=1'b1;              arb_tg[2]=t1_branch_req_tag; arb_vl[2]=t1_branch_req_value;
        arb_v[3]=mul_req_valid;       arb_t[3]=mul_req_tid;       arb_tg[3]=mul_req_tag;       arb_vl[3]=mul_req_value;
        arb_v[4]=div_req_valid;       arb_t[4]=div_req_tid;       arb_tg[4]=div_req_tag;       arb_vl[4]=div_req_value;
        arb_v[5]=lsq_req_valid;       arb_t[5]=lsq_req_tid;       arb_tg[5]=lsq_req_tag;       arb_vl[5]=lsq_req_value;
    end

    integer ai;
    reg have_a; reg [2:0] idx_a; reg a_is_head;
    always @(*) begin
        have_a = 1'b0; idx_a = 0; a_is_head = 1'b0;
        for (ai = 0; ai < NREQ; ai = ai + 1)
            if (arb_v[ai] && (!have_a || (is_head(arb_t[ai], arb_tg[ai]) && !a_is_head))) begin
                have_a = 1'b1; idx_a = ai; a_is_head = is_head(arb_t[ai], arb_tg[ai]);
            end
    end

    integer bi;
    reg have_b; reg [2:0] idx_b; reg b_is_head;
    always @(*) begin
        have_b = 1'b0; idx_b = 0; b_is_head = 1'b0;
        for (bi = 0; bi < NREQ; bi = bi + 1)
            if (arb_v[bi] && bi != idx_a && (!have_b || (is_head(arb_t[bi], arb_tg[bi]) && !b_is_head))) begin
                have_b = 1'b1; idx_b = bi; b_is_head = is_head(arb_t[bi], arb_tg[bi]);
            end
    end

    wire cdbA_valid = have_a;
    wire [TB-1:0] cdbA_tag = arb_tg[idx_a];
    wire [63:0] cdbA_value = arb_vl[idx_a];
    wire cdbA_tid = arb_t[idx_a];

    wire cdbB_valid = have_b;
    wire [TB-1:0] cdbB_tag = arb_tg[idx_b];
    wire [63:0] cdbB_value = arb_vl[idx_b];
    wire cdbB_tid = arb_t[idx_b];

    wire alu_grant       = (have_a && idx_a==0) || (have_b && idx_b==0);
    wire t0_branch_grant = (have_a && idx_a==1) || (have_b && idx_b==1);
    wire t1_branch_grant = (have_a && idx_a==2) || (have_b && idx_b==2);
    wire mul_grant        = (have_a && idx_a==3) || (have_b && idx_b==3);
    wire div_grant         = (have_a && idx_a==4) || (have_b && idx_b==4);
    wire lsq_grant          = (have_a && idx_a==5) || (have_b && idx_b==5);

    // Route each CDB winner to the correct thread's ROB via its tid,
    // reusing the existing dual mark/mark_b ports -- no new ROB ports
    // needed.
    assign t0_rob_mark_valid   = cdbA_valid && !cdbA_tid;
    assign t0_rob_mark_tag     = cdbA_tag;
    assign t0_rob_mark_value   = cdbA_value;
    assign t0_rob_mark_b_valid = cdbB_valid && !cdbB_tid;
    assign t0_rob_mark_b_tag   = cdbB_tag;
    assign t0_rob_mark_b_value = cdbB_value;

    assign t1_rob_mark_valid   = cdbA_valid && cdbA_tid;
    assign t1_rob_mark_tag     = cdbA_tag;
    assign t1_rob_mark_value   = cdbA_value;
    assign t1_rob_mark_b_valid = cdbB_valid && cdbB_tid;
    assign t1_rob_mark_b_tag   = cdbB_tag;
    assign t1_rob_mark_b_value = cdbB_value;

    // ---- Vector reservation station + functional unit (Phase 6, DLP) ------
    // Thread-0-only throughout (see module header): rob_head_tag/squash
    // wired directly to thread 0's own signals, never the active-thread
    // mux, since a thread-0 misprediction must squash vector state
    // regardless of whose dispatch turn it currently is.
    wire vec_rs_full;
    wire vec_req_valid;
    wire [TB-1:0] vec_req_tag;
    wire [VLEN-1:0] vec_req_value;
    wire vec_grant;
    wire vec_mark_valid;
    wire [TB-1:0] vec_mark_tag;
    wire [VLEN-1:0] vec_mark_value;

    wire vec_src1_ready = !vs2_busy || (vec_mark_valid && vec_mark_tag == vs2_tag) || t0_rob_vs2_done;
    wire [VLEN-1:0] vec_src1_val = !vs2_busy ? vreg_read1 :
                                    (vec_mark_valid && vec_mark_tag == vs2_tag) ? vec_mark_value :
                                    t0_rob_vs2_done ? t0_rob_vs2_value :
                                    {VLEN{1'b0}};
    wire vec_src2_ready = !vs1_busy || (vec_mark_valid && vec_mark_tag == vs1_tag) || t0_rob_vs1_done;
    wire [VLEN-1:0] vec_src2_val = !vs1_busy ? vreg_read2 :
                                    (vec_mark_valid && vec_mark_tag == vs1_tag) ? vec_mark_value :
                                    t0_rob_vs1_done ? t0_rob_vs1_value :
                                    {VLEN{1'b0}};

    // vmv.v.x: rs1 is an ordinary scalar source, resolved via the normal
    // (active-thread-muxed) scalar path -- but gated to only ever fire
    // when active_thread==0 via lane0_fire/d0_is_vmv's own thread-0-only
    // gating (d0_is_vmv is hard-0 for thread 1, see above), so
    // lane0_src1_ready/val below are always thread 0's own values
    // whenever this actually matters.
    wire lane0_vmv_stall = d0_is_vmv && !lane0_src1_ready;
    wire [VLEN-1:0] vmv_bcast_val = {LANES{lane0_src1_val[31:0]}};
    wire [4:0] vec_alloc_op        = d0_is_vmv ? 5'd0 : d0_v_op;
    wire vec_alloc_src1_ready      = d0_is_vmv ? 1'b1 : vec_src1_ready;
    wire [VLEN-1:0] vec_alloc_src1_val = d0_is_vmv ? vmv_bcast_val : vec_src1_val;
    wire [TB-1:0] vec_alloc_src1_tag   = d0_is_vmv ? {TB{1'b0}} : vs2_tag;
    wire vec_alloc_src2_ready      = d0_is_vmv ? 1'b1 : vec_src2_ready;
    wire [VLEN-1:0] vec_alloc_src2_val = d0_is_vmv ? {VLEN{1'b0}} : vec_src2_val;
    wire [TB-1:0] vec_alloc_src2_tag   = d0_is_vmv ? {TB{1'b0}} : vs1_tag;

    vec_rs #(.DEPTH(VEC_RS_DEPTH), .TAG_BITS(TB), .VLEN(VLEN)) vec_rs_i (
        .clk(clk), .reset(reset),
        .alloc_req(lane0_fire && d0_is_vec),
        .alloc_op(vec_alloc_op),
        .alloc_src1_ready(vec_alloc_src1_ready), .alloc_src1_val(vec_alloc_src1_val), .alloc_src1_tag(vec_alloc_src1_tag),
        .alloc_src2_ready(vec_alloc_src2_ready), .alloc_src2_val(vec_alloc_src2_val), .alloc_src2_tag(vec_alloc_src2_tag),
        .alloc_dest_tag(rob_alloc_tag),
        .full(vec_rs_full),
        .vec_cdb_valid(vec_mark_valid), .vec_cdb_tag(vec_mark_tag), .vec_cdb_value(vec_mark_value),
        .req_valid(vec_req_valid), .req_tag(vec_req_tag), .req_value(vec_req_value),
        .req_grant(vec_grant),
        .rob_head_tag(t0_rob_head_tag), .squash_valid(t0_mispredict), .squash_tag(t0_branch_resolved_tag)
    );
    assign vec_grant = vec_req_valid;
    assign vec_mark_valid = vec_req_valid;
    assign vec_mark_tag   = vec_req_tag;
    assign vec_mark_value = vec_req_value;
    assign vec_write_en   = lane0_fire && d0_is_vec;

    // ---- ALU reservation-station bank + functional unit (shared, tagged) --
    wire alu_rs_full, alu_rs_has_2_free;
    wire alu_req_valid, alu_req_tid;
    wire [TB-1:0] alu_req_tag;
    wire [63:0] alu_req_value;

    alu_rs #(.DEPTH(ALU_RS_DEPTH), .TAG_BITS(TB)) alu_rs_i (
        .clk(clk), .reset(reset),
        .alloc_req(lane0_fire && d0_is_alu), .alloc_tid(active_thread),
        .alloc_op(d0_alu_op), .alloc_word_op(d0_word_op),
        .alloc_src1_ready(lane0_src1_ready), .alloc_src1_val(lane0_src1_val), .alloc_src1_tag(rs1_tag),
        .alloc_src2_ready(lane0_src2_ready), .alloc_src2_val(lane0_src2_val), .alloc_src2_tag(rs2_tag),
        .alloc_dest_tag(rob_alloc_tag),
        .full(alu_rs_full),
        .alloc2_req(lane1_fire && d1_is_alu), .alloc2_tid(active_thread),
        .alloc2_op(d1_alu_op), .alloc2_word_op(d1_word_op),
        .alloc2_src1_ready(lane1_src1_ready), .alloc2_src1_val(lane1_src1_val), .alloc2_src1_tag(lane1_src1_tag),
        .alloc2_src2_ready(lane1_src2_ready), .alloc2_src2_val(lane1_src2_val), .alloc2_src2_tag(lane1_src2_tag),
        .alloc2_dest_tag(rob_alloc2_tag),
        .has_2_free(alu_rs_has_2_free),
        .cdbA_valid(cdbA_valid), .cdbA_tid(cdbA_tid), .cdbA_tag(cdbA_tag), .cdbA_value(cdbA_value),
        .cdbB_valid(cdbB_valid), .cdbB_tid(cdbB_tid), .cdbB_tag(cdbB_tag), .cdbB_value(cdbB_value),
        .req_valid(alu_req_valid), .req_tid(alu_req_tid), .req_tag(alu_req_tag), .req_value(alu_req_value),
        .req_grant(alu_grant),
        .rob_head_tag0(t0_rob_head_tag), .rob_head_tag1(t1_rob_head_tag),
        .squash0_valid(t0_mispredict), .squash0_tag(t0_branch_resolved_tag),
        .squash1_valid(t1_mispredict), .squash1_tag(t1_branch_resolved_tag)
    );

    // ---- Multiply reservation-station bank + functional unit (shared) -----
    wire mul_rs_full, mul_rs_has_2_free;
    wire mul_req_valid, mul_req_tid;
    wire [TB-1:0] mul_req_tag;
    wire [63:0] mul_req_value;

    mul_rs #(.DEPTH(MUL_RS_DEPTH), .TAG_BITS(TB)) mul_rs_i (
        .clk(clk), .reset(reset),
        .alloc_req(lane0_fire && d0_is_mul), .alloc_tid(active_thread),
        .alloc_op(d0_muldiv_op), .alloc_word_op(d0_word_op),
        .alloc_src1_ready(lane0_src1_ready), .alloc_src1_val(lane0_src1_val), .alloc_src1_tag(rs1_tag),
        .alloc_src2_ready(lane0_src2_ready), .alloc_src2_val(lane0_src2_val), .alloc_src2_tag(rs2_tag),
        .alloc_dest_tag(rob_alloc_tag),
        .full(mul_rs_full),
        .alloc2_req(lane1_fire && d1_is_mul), .alloc2_tid(active_thread),
        .alloc2_op(d1_muldiv_op), .alloc2_word_op(d1_word_op),
        .alloc2_src1_ready(lane1_src1_ready), .alloc2_src1_val(lane1_src1_val), .alloc2_src1_tag(lane1_src1_tag),
        .alloc2_src2_ready(lane1_src2_ready), .alloc2_src2_val(lane1_src2_val), .alloc2_src2_tag(lane1_src2_tag),
        .alloc2_dest_tag(rob_alloc2_tag),
        .has_2_free(mul_rs_has_2_free),
        .cdbA_valid(cdbA_valid), .cdbA_tid(cdbA_tid), .cdbA_tag(cdbA_tag), .cdbA_value(cdbA_value),
        .cdbB_valid(cdbB_valid), .cdbB_tid(cdbB_tid), .cdbB_tag(cdbB_tag), .cdbB_value(cdbB_value),
        .req_valid(mul_req_valid), .req_tid(mul_req_tid), .req_tag(mul_req_tag), .req_value(mul_req_value),
        .req_grant(mul_grant),
        .rob_head_tag0(t0_rob_head_tag), .rob_head_tag1(t1_rob_head_tag),
        .squash0_valid(t0_mispredict), .squash0_tag(t0_branch_resolved_tag),
        .squash1_valid(t1_mispredict), .squash1_tag(t1_branch_resolved_tag)
    );

    // ---- Divide reservation station + functional unit (shared, multi-cycle)
    wire div_rs_full;
    wire div_req_valid, div_req_tid;
    wire [TB-1:0] div_req_tag;
    wire [63:0] div_req_value;

    wire div_from_lane0 = lane0_fire && d0_is_div;
    wire div_from_lane1 = lane1_fire && d1_is_div;

    div_rs #(.TAG_BITS(TB)) div_rs_i (
        .clk(clk), .reset(reset),
        .alloc_req(div_from_lane0 || div_from_lane1), .alloc_tid(active_thread),
        .alloc_op(div_from_lane0 ? d0_muldiv_op : d1_muldiv_op),
        .alloc_word_op(div_from_lane0 ? d0_word_op : d1_word_op),
        .alloc_src1_ready(div_from_lane0 ? lane0_src1_ready : lane1_src1_ready),
        .alloc_src1_val(div_from_lane0 ? lane0_src1_val : lane1_src1_val),
        .alloc_src1_tag(div_from_lane0 ? rs1_tag : lane1_src1_tag),
        .alloc_src2_ready(div_from_lane0 ? lane0_src2_ready : lane1_src2_ready),
        .alloc_src2_val(div_from_lane0 ? lane0_src2_val : lane1_src2_val),
        .alloc_src2_tag(div_from_lane0 ? rs2_tag : lane1_src2_tag),
        .alloc_dest_tag(div_from_lane0 ? rob_alloc_tag : rob_alloc2_tag),
        .full(div_rs_full),
        .cdbA_valid(cdbA_valid), .cdbA_tid(cdbA_tid), .cdbA_tag(cdbA_tag), .cdbA_value(cdbA_value),
        .cdbB_valid(cdbB_valid), .cdbB_tid(cdbB_tid), .cdbB_tag(cdbB_tag), .cdbB_value(cdbB_value),
        .req_valid(div_req_valid), .req_tid(div_req_tid), .req_tag(div_req_tag), .req_value(div_req_value),
        .req_grant(div_grant),
        .rob_head_tag0(t0_rob_head_tag), .rob_head_tag1(t1_rob_head_tag),
        .squash0_valid(t0_mispredict), .squash0_tag(t0_branch_resolved_tag),
        .squash1_valid(t1_mispredict), .squash1_tag(t1_branch_resolved_tag)
    );

    // ---- Load-Store Queue + private L1 cache (shared across threads, tagged)
    wire lsq_full, lsq_has_2_free;
    wire lsq_req_valid, lsq_req_tid;
    wire [TB-1:0] lsq_req_tag;
    wire [63:0] lsq_req_value;
    wire lsq_commit_match;
    wire lsq_store_buffer_full;
    wire [LSQ_DEPTH-1:0] lsq_store_ready;
    wire [LSQ_DEPTH*TB-1:0] lsq_store_ready_tag_flat;
    wire [LSQ_DEPTH-1:0] lsq_store_ready_tid_flat;

    wire lane0_data_ready = d0_is_load || lane0_src2_ready;
    wire lane1_data_ready = d1_is_load || lane1_src2_ready;

    // ---- Phase 8: private L1 data cache (one per core, shared by both
    // SMT threads -- like the shared RS banks, distinguished only by the
    // tid each LSQ/store-buffer entry already carries, not by a second
    // L1 instance). CPU-side ports wire directly to lsq_i; L2-side and
    // snoop ports simply pass through to this module's own l2_*/snoop_*
    // ports (see l1_cache.v's header for the full protocol).
    wire l1_read_req; wire [63:0] l1_read_addr; wire [2:0] l1_read_func3;
    wire l1_read_valid; wire [63:0] l1_read_data;
    wire l1_write_req; wire [63:0] l1_write_addr, l1_write_data; wire [2:0] l1_write_func3;
    wire l1_write_done, l1_busy;

    l1_cache #(.LINES(L1_LINES), .LINE_BYTES(L1_LINE_BYTES), .ADDR_BITS(64)) l1_cache_i (
        .clk(clk), .reset(reset),
        .cpu_read_req(l1_read_req), .cpu_read_addr(l1_read_addr), .cpu_read_func3(l1_read_func3),
        .cpu_read_valid(l1_read_valid), .cpu_read_data(l1_read_data),
        .cpu_write_req(l1_write_req), .cpu_write_addr(l1_write_addr), .cpu_write_data(l1_write_data),
        .cpu_write_func3(l1_write_func3), .cpu_write_done(l1_write_done),
        .busy(l1_busy),
        .l2_req_valid(l2_req_valid), .l2_req_type(l2_req_type), .l2_req_addr(l2_req_addr),
        .l2_req_wb_data(l2_req_wb_data),
        .l2_resp_valid(l2_resp_valid), .l2_resp_data(l2_resp_data), .l2_resp_exclusive(l2_resp_exclusive),
        .snoop_req_valid(snoop_req_valid), .snoop_req_type(snoop_req_type), .snoop_req_addr(snoop_req_addr),
        .snoop_resp_hit(snoop_resp_hit), .snoop_resp_dirty(snoop_resp_dirty), .snoop_resp_data(snoop_resp_data)
    );

    lsq #(.DEPTH(LSQ_DEPTH), .TAG_BITS(TB), .SBUF_DEPTH(SBUF_DEPTH)) lsq_i (
        .clk(clk), .reset(reset),
        .alloc_req(lane0_fire && (d0_is_load || d0_is_store)), .alloc_tid(active_thread),
        .alloc_is_store(d0_is_store), .alloc_func3(d0_func3), .alloc_imm(d0_imm),
        .alloc_base_ready(lane0_src1_ready), .alloc_base_val(lane0_src1_val), .alloc_base_tag(rs1_tag),
        .alloc_data_ready(lane0_data_ready), .alloc_data_val(lane0_src2_val), .alloc_data_tag(rs2_tag),
        .alloc_dest_tag(rob_alloc_tag),
        .full(lsq_full),
        .alloc2_req(lane1_fire && (d1_is_load || d1_is_store)), .alloc2_tid(active_thread),
        .alloc2_is_store(d1_is_store), .alloc2_func3(d1_func3), .alloc2_imm(d1_imm),
        .alloc2_base_ready(lane1_src1_ready), .alloc2_base_val(lane1_src1_val), .alloc2_base_tag(lane1_src1_tag),
        .alloc2_data_ready(lane1_data_ready), .alloc2_data_val(lane1_src2_val), .alloc2_data_tag(lane1_src2_tag),
        .alloc2_dest_tag(rob_alloc2_tag),
        .has_2_free(lsq_has_2_free),
        .cdbA_valid(cdbA_valid), .cdbA_tid(cdbA_tid), .cdbA_tag(cdbA_tag), .cdbA_value(cdbA_value),
        .cdbB_valid(cdbB_valid), .cdbB_tid(cdbB_tid), .cdbB_tag(cdbB_tag), .cdbB_value(cdbB_value),
        .rob_head_tag0(t0_rob_head_tag), .rob_head_tag1(t1_rob_head_tag),
        .commit_lookup_tid(commit_lookup_tid), .commit_lookup_tag(commit_lookup_tag),
        .req_valid(lsq_req_valid), .req_tid(lsq_req_tid), .req_tag(lsq_req_tag), .req_value(lsq_req_value),
        .req_grant(lsq_grant),
        .l1_read_req(l1_read_req), .l1_read_addr(l1_read_addr), .l1_read_func3(l1_read_func3),
        .l1_read_valid(l1_read_valid), .l1_read_data(l1_read_data),
        .l1_write_req(l1_write_req), .l1_write_addr(l1_write_addr), .l1_write_data(l1_write_data),
        .l1_write_func3(l1_write_func3), .l1_write_done(l1_write_done), .l1_busy(l1_busy),
        .store_ready(lsq_store_ready), .store_ready_tag_flat(lsq_store_ready_tag_flat),
        .store_ready_tid_flat(lsq_store_ready_tid_flat),
        .commit_match(lsq_commit_match), .commit_fire(lsq_commit_fire), .store_buffer_full(lsq_store_buffer_full),
        .squash0_valid(t0_mispredict), .squash0_tag(t0_branch_resolved_tag),
        .squash1_valid(t1_mispredict), .squash1_tag(t1_branch_resolved_tag)
    );

    // Per-slot store-readiness routed to the correct thread's ROB extra_mark
    // port via store_ready_tid_flat -- a plain bitmask split, since ROB
    // extra_mark itself has no tid-demux built in (see rob.v's header).
    wire [LSQ_DEPTH-1:0] t0_lsq_extra_mark_valid = lsq_store_ready & ~lsq_store_ready_tid_flat;
    wire [LSQ_DEPTH-1:0] t1_lsq_extra_mark_valid = lsq_store_ready &  lsq_store_ready_tid_flat;

    // Widened commit + Phase 7 cross-thread store-port arbitration: at most
    // one store across BOTH threads' ROBs commits per cycle -- not because
    // of a physical memory port anymore (Phase 8's store buffer decouples
    // that), but because lsq_i still exposes exactly one commit_fire/
    // commit_lookup_tid/commit_lookup_tag port (deliberately not widened
    // to two -- the store buffer already absorbs the real back-pressure
    // concern a second port would have addressed). Thread 0 has fixed
    // priority; if thread 1 also wants the port this cycle, thread 1's
    // contended commit is suppressed -- its whole commit (not just head2)
    // if thread 1's own head was the contended store, since in-order
    // commit means head2 can never retire past a blocked head. See module
    // header. Phase 8 additionally requires !lsq_store_buffer_full: a
    // store cannot be considered a commit candidate at all if there's
    // nowhere to push it, regardless of which thread "wins" arbitration.
    // Bug fix (present since Phase 5, only exposed by Phase 7's SMT
    // timing): head2_ready can legitimately go true (marked done via its
    // own CDB broadcast) cycles before head_ready does -- head and head2
    // finish out of order even though they can only ever *retire* in
    // order. rob.v's own do_commit2 already correctly requires do_commit1
    // (head also retiring this same cycle) before it will actually vacate
    // head2's slot, but every downstream top-level use of
    // t0_rob_commit_req2 (register-file write, RAT clear, store/vector
    // commit below) was NOT gated the same way -- so head2's architectural
    // effects could fire repeatedly, cycle after cycle, *before* head
    // itself ever retires, corrupting precise state observed at exactly
    // head's own retirement (e.g. an ecall_halt reading a register that a
    // program-order-*later* instruction had already prematurely written).
    // Fixing this once here, at the source, rather than patching every
    // downstream use site.
    wire sbuf_ok = !lsq_store_buffer_full;

    wire t0_commit2_store_conflict = t0_rob_head_is_store && t0_rob_head2_is_store;
    wire t0_commit2_vec_conflict   = t0_rob_head_is_vec_dest && t0_rob_head2_is_vec_dest;
    wire t0_head_wants_store       = t0_rob_head_ready  && t0_rob_head_is_store;
    wire t0_head2_wants_store_cand = t0_rob_head2_ready && !t0_commit2_store_conflict && t0_rob_head2_is_store;
    wire t0_head1_sbuf_block = t0_head_wants_store && !sbuf_ok;
    wire t0_head2_sbuf_block = t0_head2_wants_store_cand && !sbuf_ok;
    assign t0_rob_commit_req  = t0_rob_head_ready && !t0_head1_sbuf_block;
    assign t0_rob_commit_req2 = t0_rob_commit_req && t0_rob_head2_ready && !t0_commit2_store_conflict &&
                                 !t0_commit2_vec_conflict && !t0_head2_sbuf_block;

    wire t0_commit_store_is_head1 = t0_rob_commit_req  && t0_rob_head_is_store;
    wire t0_commit_store_is_head2 = t0_rob_commit_req2 && t0_rob_head2_is_store;
    wire t0_wants_store = t0_commit_store_is_head1 || t0_commit_store_is_head2;

    wire t1_commit2_store_conflict = t1_rob_head_is_store && t1_rob_head2_is_store;
    wire t1_commit_store_is_head1_cand = t1_rob_head_ready  && t1_rob_head_is_store;
    wire t1_commit_store_is_head2_cand = t1_rob_head2_ready && !t1_commit2_store_conflict && t1_rob_head2_is_store;
    wire t1_wants_store_cand = t1_commit_store_is_head1_cand || t1_commit_store_is_head2_cand;

    wire t1_blocked_by_t0_or_sbuf = t1_wants_store_cand && (t0_wants_store || !sbuf_ok);
    wire t1_head1_store_blocked = t1_commit_store_is_head1_cand && t1_blocked_by_t0_or_sbuf;
    wire t1_head2_store_blocked = t1_commit_store_is_head2_cand && t1_blocked_by_t0_or_sbuf;

    // Same do_commit1-equivalent gate as t0's fix above (t1_rob_commit_req
    // already folds in the cross-thread store-port block, so requiring it
    // here also correctly withholds head2's commit whenever head1 lost
    // that arbitration).
    assign t1_rob_commit_req  = t1_rob_head_ready && !t1_head1_store_blocked;
    assign t1_rob_commit_req2 = t1_rob_commit_req && t1_rob_head2_ready && !t1_commit2_store_conflict && !t1_head2_store_blocked;

    wire t1_commit_store_is_head1 = t1_rob_commit_req  && t1_rob_head_is_store;
    wire t1_commit_store_is_head2 = t1_rob_commit_req2 && t1_rob_head2_is_store;

    // At most one of these four is ever true, by construction (intra-thread
    // conflicts already prevent a thread's own head+head2 both being
    // stores; the cross-thread arbiter above ensures at most one thread
    // wins the single commit_fire port, gated on the store buffer actually
    // having room).
    wire lsq_commit_fire = t0_commit_store_is_head1 || t0_commit_store_is_head2 ||
                            t1_commit_store_is_head1 || t1_commit_store_is_head2;
    wire commit_lookup_tid = t0_commit_store_is_head1 ? 1'b0 :
                              t0_commit_store_is_head2 ? 1'b0 :
                              t1_commit_store_is_head1 ? 1'b1 :
                              t1_commit_store_is_head2 ? 1'b1 : 1'b0;
    wire [TB-1:0] commit_lookup_tag = t0_commit_store_is_head1 ? t0_rob_head_tag :
                                       t0_commit_store_is_head2 ? t0_rob_head2_tag :
                                       t1_commit_store_is_head1 ? t1_rob_head_tag :
                                       t1_commit_store_is_head2 ? t1_rob_head2_tag : {TB{1'b0}};

    // Vector commit writeback: thread-0-only, so no cross-thread conflict is
    // possible -- mirrors the (single-thread) store-conflict pattern above,
    // targeting vector_register_file.v's single write port.
    wire t0_commit_vec_is_head1 = t0_rob_commit_req  && t0_rob_head_is_vec_dest;
    wire t0_commit_vec_is_head2 = t0_rob_commit_req2 && t0_rob_head2_is_vec_dest;
    assign vec_commit_write_en_w   = t0_commit_vec_is_head1 || t0_commit_vec_is_head2;
    assign vec_commit_write_reg_w  = t0_commit_vec_is_head1 ? t0_rob_head_rd : t0_rob_head2_rd;
    assign vec_commit_write_data_w = t0_commit_vec_is_head1 ? t0_rob_head_vec_value : t0_rob_head2_vec_value;
    assign vec_commit_clear_en_w   = vec_commit_write_en_w;
    assign vec_commit_clear_rd_w   = vec_commit_write_reg_w;
    assign vec_commit_clear_tag_w  = t0_commit_vec_is_head1 ? t0_rob_head_tag : t0_rob_head2_tag;

    // ================================================================
    // ---- Branch-class reservation stations (per thread) -------------
    // ================================================================
    // Lane-0-only allocation on each, same as before -- lane 1 can never
    // be branch-class. Each thread's instance is fully independent, not
    // shared/tagged: branch resolution (resolved/resolved_tag/etc.) is
    // therefore genuinely per-thread and asynchronous to the dispatch
    // round-robin, not muxed by active_thread -- both threads' branches
    // can resolve, and both mispredict, in the same cycle.
    wire t0_branch_rs_full;
    wire t0_branch_resolved;
    wire [63:0] t0_branch_resolved_next_pc;
    wire [TB-1:0] t0_branch_resolved_tag;
    wire t0_branch_resolved_has_result;
    wire t0_branch_req_valid;
    wire [TB-1:0] t0_branch_req_tag;
    wire [63:0] t0_branch_req_value;
    wire [63:0] t0_branch_resolved_pc;
    wire t0_branch_resolved_taken;
    wire t0_branch_resident_is_jalr;

    wire t0_branch_alloc_req = (active_thread == 1'b0) && lane0_fire && (d0_is_branch || d0_is_jal || d0_is_jalr);
    wire branch_src1_ready = d0_is_jal || lane0_src1_ready;
    wire branch_src2_ready = !d0_is_branch || lane0_src2_ready;

    branch_rs #(.TAG_BITS(TB), .MY_TID(0)) t0_branch_rs_i (
        .clk(clk), .reset(reset),
        .alloc_req(t0_branch_alloc_req),
        .alloc_func3(d0_func3),
        .alloc_is_jal(d0_is_jal), .alloc_is_jalr(d0_is_jalr), .alloc_is_branch(d0_is_branch),
        .alloc_pc(pc), .alloc_imm(d0_imm),
        .alloc_src1_ready(branch_src1_ready), .alloc_src1_val(lane0_src1_val), .alloc_src1_tag(rs1_tag),
        .alloc_src2_ready(branch_src2_ready), .alloc_src2_val(lane0_src2_val), .alloc_src2_tag(rs2_tag),
        .alloc_dest_tag(rob_alloc_tag),
        .full(t0_branch_rs_full),
        .cdbA_valid(cdbA_valid), .cdbA_tid(cdbA_tid), .cdbA_tag(cdbA_tag), .cdbA_value(cdbA_value),
        .cdbB_valid(cdbB_valid), .cdbB_tid(cdbB_tid), .cdbB_tag(cdbB_tag), .cdbB_value(cdbB_value),
        .resolved(t0_branch_resolved), .resolved_next_pc(t0_branch_resolved_next_pc),
        .resolved_tag(t0_branch_resolved_tag), .resolved_has_result(t0_branch_resolved_has_result),
        .resolved_pc(t0_branch_resolved_pc), .resolved_taken(t0_branch_resolved_taken),
        .req_valid(t0_branch_req_valid), .req_tag(t0_branch_req_tag), .req_value(t0_branch_req_value),
        .req_grant(t0_branch_grant),
        .resident_is_jalr(t0_branch_resident_is_jalr)
    );

    wire t1_branch_rs_full;
    wire t1_branch_resolved;
    wire [63:0] t1_branch_resolved_next_pc;
    wire [TB-1:0] t1_branch_resolved_tag;
    wire t1_branch_resolved_has_result;
    wire t1_branch_req_valid;
    wire [TB-1:0] t1_branch_req_tag;
    wire [63:0] t1_branch_req_value;
    wire [63:0] t1_branch_resolved_pc;
    wire t1_branch_resolved_taken;
    wire t1_branch_resident_is_jalr;

    wire t1_branch_alloc_req = (active_thread == 1'b1) && lane0_fire && (d0_is_branch || d0_is_jal || d0_is_jalr);

    branch_rs #(.TAG_BITS(TB), .MY_TID(1)) t1_branch_rs_i (
        .clk(clk), .reset(reset),
        .alloc_req(t1_branch_alloc_req),
        .alloc_func3(d0_func3),
        .alloc_is_jal(d0_is_jal), .alloc_is_jalr(d0_is_jalr), .alloc_is_branch(d0_is_branch),
        .alloc_pc(pc), .alloc_imm(d0_imm),
        .alloc_src1_ready(branch_src1_ready), .alloc_src1_val(lane0_src1_val), .alloc_src1_tag(rs1_tag),
        .alloc_src2_ready(branch_src2_ready), .alloc_src2_val(lane0_src2_val), .alloc_src2_tag(rs2_tag),
        .alloc_dest_tag(rob_alloc_tag),
        .full(t1_branch_rs_full),
        .cdbA_valid(cdbA_valid), .cdbA_tid(cdbA_tid), .cdbA_tag(cdbA_tag), .cdbA_value(cdbA_value),
        .cdbB_valid(cdbB_valid), .cdbB_tid(cdbB_tid), .cdbB_tag(cdbB_tag), .cdbB_value(cdbB_value),
        .resolved(t1_branch_resolved), .resolved_next_pc(t1_branch_resolved_next_pc),
        .resolved_tag(t1_branch_resolved_tag), .resolved_has_result(t1_branch_resolved_has_result),
        .resolved_pc(t1_branch_resolved_pc), .resolved_taken(t1_branch_resolved_taken),
        .req_valid(t1_branch_req_valid), .req_tag(t1_branch_req_tag), .req_value(t1_branch_req_value),
        .req_grant(t1_branch_grant),
        .resident_is_jalr(t1_branch_resident_is_jalr)
    );

    wire branch_rs_full = active_thread ? t1_branch_rs_full : t0_branch_rs_full;

    // A resolved conditional branch has no destination register and thus
    // never wins (or even requests) the CDB -- routed through rob.v's
    // separate, unarbitrated mark2 port instead, per thread.
    assign t0_rob_mark2_valid = t0_branch_resolved && !t0_branch_resolved_has_result;
    assign t0_rob_mark2_tag   = t0_branch_resolved_tag;
    assign t1_rob_mark2_valid = t1_branch_resolved && !t1_branch_resolved_has_result;
    assign t1_rob_mark2_tag   = t1_branch_resolved_tag;

    // ---- Branch prediction + speculation (Phase 2, per thread) ------------
    wire t0_jalr_outstanding = t0_branch_resident_is_jalr;
    wire t1_jalr_outstanding = t1_branch_resident_is_jalr;
    wire jalr_outstanding = active_thread ? t1_jalr_outstanding : t0_jalr_outstanding;

    // BHT: single shared instance (see module header) -- predicts for
    // whichever thread is currently dispatching (predict_pc = the muxed
    // `pc`), updated by whichever thread resolves a conditional branch
    // this cycle, thread 0 winning if both resolve the same cycle.
    wire bht_predict_taken;
    wire t0_bht_update_cand = t0_spec_active && t0_branch_resolved;
    wire t1_bht_update_cand = t1_spec_active && t1_branch_resolved;
    wire bht_update_use_t0 = t0_bht_update_cand;
    bht bht_i (
        .clk(clk), .reset(reset),
        .predict_pc(pc), .predict_taken(bht_predict_taken),
        .update_valid(t0_bht_update_cand || t1_bht_update_cand),
        .update_pc(bht_update_use_t0 ? t0_branch_resolved_pc : t1_branch_resolved_pc),
        .update_taken(bht_update_use_t0 ? t0_branch_resolved_taken : t1_branch_resolved_taken)
    );

    reg t0_spec_active;
    reg t0_predicted_taken_reg;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            t0_spec_active <= 1'b0;
        end else if ((active_thread == 1'b0) && lane0_fire && d0_is_branch) begin
            t0_spec_active <= 1'b1;
            t0_predicted_taken_reg <= bht_predict_taken;
        end else if (t0_branch_resolved) begin
            t0_spec_active <= 1'b0;
        end
    end

    reg t1_spec_active;
    reg t1_predicted_taken_reg;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            t1_spec_active <= 1'b0;
        end else if ((active_thread == 1'b1) && lane0_fire && d0_is_branch) begin
            t1_spec_active <= 1'b1;
            t1_predicted_taken_reg <= bht_predict_taken;
        end else if (t1_branch_resolved) begin
            t1_spec_active <= 1'b0;
        end
    end

    wire t0_mispredict = t0_spec_active && t0_branch_resolved && (t0_branch_resolved_taken != t0_predicted_taken_reg);
    wire t1_mispredict = t1_spec_active && t1_branch_resolved && (t1_branch_resolved_taken != t1_predicted_taken_reg);
    // Used only inside the shared dispatch logic's "!mispredict" gate below
    // -- the active thread's own mispredict, discovered this exact cycle,
    // must block its own wrong-path dispatch.
    wire mispredict = active_thread ? t1_mispredict : t0_mispredict;

    wire t0_rat_checkpoint_save = (active_thread == 1'b0) && lane0_fire && d0_is_branch;
    wire t1_rat_checkpoint_save = (active_thread == 1'b1) && lane0_fire && d0_is_branch;

    wire t0_redirect_needed = t0_mispredict || (t0_jalr_outstanding && t0_branch_resolved);
    wire t1_redirect_needed = t1_mispredict || (t1_jalr_outstanding && t1_branch_resolved);

    // ================================================================
    // ---- Dispatch (2-wide, active-thread view) -----------------------
    // ================================================================
    // Unchanged from Phase 3/5/6 -- see module header for why this whole
    // block is reused verbatim, now fed by the active-thread-muxed wires
    // defined above.
    wire lane0_dispatchable = d0_is_alu || d0_is_branch || d0_is_jal || d0_is_jalr || d0_is_mul || d0_is_div ||
                               d0_is_load || d0_is_store || d0_is_vec;
    wire lane0_needed_rs_full = (d0_is_alu && alu_rs_full) ||
                                 ((d0_is_branch || d0_is_jal || d0_is_jalr) && branch_rs_full) ||
                                 (d0_is_mul && mul_rs_full) ||
                                 (d0_is_div && div_rs_full) ||
                                 ((d0_is_load || d0_is_store) && lsq_full) ||
                                 (d0_is_vec && vec_rs_full);
    wire lane0_fire = lane0_dispatchable && (rob_free_count >= 1) && !lane0_needed_rs_full &&
                       !jalr_outstanding && !mispredict && !lane0_vmv_stall;

    wire lane0_breaks_flow = d0_is_jal || d0_is_jalr || (d0_is_branch && bht_predict_taken);
    wire lane1_dispatchable = d1_is_alu || d1_is_mul || d1_is_div || d1_is_load || d1_is_store;

    wire lane1_alu_ok = !d1_is_alu || (d0_is_alu ? alu_rs_has_2_free : !alu_rs_full);
    wire lane1_mul_ok = !d1_is_mul || (d0_is_mul ? mul_rs_has_2_free : !mul_rs_full);
    wire lane1_div_ok = !d1_is_div || (!d0_is_div && !div_rs_full);
    wire lane1_lsq_ok = !(d1_is_load || d1_is_store) ||
                        ((d0_is_load || d0_is_store) ? lsq_has_2_free : !lsq_full);
    wire lane1_resource_ok = lane1_alu_ok && lane1_mul_ok && lane1_div_ok && lane1_lsq_ok;

    wire lane1_fire = ENABLE_DUAL_ISSUE && lane0_fire && !lane0_breaks_flow && lane1_dispatchable &&
                       (rob_free_count >= 2) && lane1_resource_ok;

    // Phase 7 fix: the same-cycle CDB bypass must also check that the
    // broadcast's tid matches the active thread -- rs1_tag/rs2_tag are
    // this thread's own ROB tag, only unique *within* its own ROB, so a
    // tag-only compare against cdbA/cdbB (which can carry either
    // thread's winner) can spuriously bypass-match a same-numbered tag
    // belonging to the OTHER thread's unrelated producer. See alu_rs.v's
    // identical fix for the shared-bank version of this same bug.
    wire cdbA_hits_active = cdbA_valid && (cdbA_tid == active_thread);
    wire cdbB_hits_active = cdbB_valid && (cdbB_tid == active_thread);

    wire lane0_src1_ready = d0_src1_is_zero || d0_src1_is_pc || !rs1_busy ||
                             (cdbA_hits_active && cdbA_tag == rs1_tag) || (cdbB_hits_active && cdbB_tag == rs1_tag) || rob_rs1_done;
    wire [63:0] lane0_src1_val = d0_src1_is_zero ? 64'b0 :
                                  d0_src1_is_pc   ? pc :
                                  !rs1_busy       ? rf_read1 :
                                  (cdbA_hits_active && cdbA_tag == rs1_tag) ? cdbA_value :
                                  (cdbB_hits_active && cdbB_tag == rs1_tag) ? cdbB_value :
                                  rob_rs1_done    ? rob_rs1_value :
                                  64'b0;

    wire lane0_src2_ready = d0_src2_is_imm || !rs2_busy ||
                             (cdbA_hits_active && cdbA_tag == rs2_tag) || (cdbB_hits_active && cdbB_tag == rs2_tag) || rob_rs2_done;
    wire [63:0] lane0_src2_val = d0_src2_is_imm ? d0_imm :
                                  !rs2_busy      ? rf_read2 :
                                  (cdbA_hits_active && cdbA_tag == rs2_tag) ? cdbA_value :
                                  (cdbB_hits_active && cdbB_tag == rs2_tag) ? cdbB_value :
                                  rob_rs2_done   ? rob_rs2_value :
                                  64'b0;

    wire lane1_rs1_intra_hit = d0_reg_write && (d1_rs1 == d0_rd) && (d0_rd != 5'd0);
    wire lane1_rs2_intra_hit = d0_reg_write && (d1_rs2 == d0_rd) && (d0_rd != 5'd0);

    wire lane1_raw_src1_ready = !rs1b_busy_raw ||
                                 (cdbA_hits_active && cdbA_tag == rs1b_tag_raw) || (cdbB_hits_active && cdbB_tag == rs1b_tag_raw) || rob_rs1b_done;
    wire [63:0] lane1_raw_src1_val = !rs1b_busy_raw ? rf1_read1 :
                                      (cdbA_hits_active && cdbA_tag == rs1b_tag_raw) ? cdbA_value :
                                      (cdbB_hits_active && cdbB_tag == rs1b_tag_raw) ? cdbB_value :
                                      rob_rs1b_done  ? rob_rs1b_value :
                                      64'b0;

    wire lane1_src1_ready = d1_src1_is_zero || d1_src1_is_pc ||
                             (lane1_rs1_intra_hit ? 1'b0 : lane1_raw_src1_ready);
    wire [63:0] lane1_src1_val = d1_src1_is_zero ? 64'b0 :
                                  d1_src1_is_pc   ? pc1 :
                                  lane1_rs1_intra_hit ? 64'b0 :
                                  lane1_raw_src1_val;
    wire [TB-1:0] lane1_src1_tag = lane1_rs1_intra_hit ? rob_alloc_tag : rs1b_tag_raw;

    wire lane1_raw_src2_ready = !rs2b_busy_raw ||
                                 (cdbA_hits_active && cdbA_tag == rs2b_tag_raw) || (cdbB_hits_active && cdbB_tag == rs2b_tag_raw) || rob_rs2b_done;
    wire [63:0] lane1_raw_src2_val = !rs2b_busy_raw ? rf1_read2 :
                                      (cdbA_hits_active && cdbA_tag == rs2b_tag_raw) ? cdbA_value :
                                      (cdbB_hits_active && cdbB_tag == rs2b_tag_raw) ? cdbB_value :
                                      rob_rs2b_done  ? rob_rs2b_value :
                                      64'b0;

    wire lane1_src2_ready = d1_src2_is_imm || (lane1_rs2_intra_hit ? 1'b0 : lane1_raw_src2_ready);
    wire [63:0] lane1_src2_val = d1_src2_is_imm ? d1_imm :
                                  lane1_rs2_intra_hit ? 64'b0 :
                                  lane1_raw_src2_val;
    wire [TB-1:0] lane1_src2_tag = lane1_rs2_intra_hit ? rob_alloc_tag : rs2b_tag_raw;

    wire rob_alloc_req  = lane0_fire;
    wire rob_alloc2_req = lane1_fire;
    wire rat_write_en   = lane0_fire && d0_reg_write;
    wire [TB-1:0] rat_new_tag = rob_alloc_tag;
    wire rat_write2_en  = lane1_fire && d1_reg_write;
    wire [TB-1:0] rat_new_tag2 = rob_alloc2_tag;

    // ---- Demux: shared dispatch decisions -> the active thread's own ----
    // ---- RAT/ROB/branch_rs write ports -----------------------------------
    assign t0_rob_alloc_req  = (active_thread == 1'b0) && rob_alloc_req;
    assign t0_rob_alloc2_req = (active_thread == 1'b0) && rob_alloc2_req;
    assign t1_rob_alloc_req  = (active_thread == 1'b1) && rob_alloc_req;
    assign t1_rob_alloc2_req = (active_thread == 1'b1) && rob_alloc2_req;

    assign t0_rat_write_en  = (active_thread == 1'b0) && rat_write_en;
    assign t0_rat_write2_en = (active_thread == 1'b0) && rat_write2_en;
    assign t1_rat_write_en  = (active_thread == 1'b1) && rat_write_en;
    assign t1_rat_write2_en = (active_thread == 1'b1) && rat_write2_en;

    // ---- Per-thread PC: async misprediction/JALR redirect regardless of --
    // ---- whose dispatch turn it is; straight-line advance / JAL / -------
    // ---- predicted-taken redirect only on this thread's own turn --------
    always @(*) begin
        if (t0_redirect_needed)
            t0_next_pc = t0_branch_resolved_next_pc;
        else if (active_thread == 1'b0 && lane0_fire && d0_is_jal)
            t0_next_pc = t0_pc + d0_imm;
        else if (active_thread == 1'b0 && lane0_fire && d0_is_branch && bht_predict_taken)
            t0_next_pc = t0_pc + d0_imm;
        else if (active_thread == 1'b0 && lane0_fire)
            t0_next_pc = lane1_fire ? (t0_pc + 64'd8) : (t0_pc + 64'd4);
        else
            t0_next_pc = t0_pc;
    end

    always @(*) begin
        if (t1_redirect_needed)
            t1_next_pc = t1_branch_resolved_next_pc;
        else if (active_thread == 1'b1 && lane0_fire && d0_is_jal)
            t1_next_pc = t1_pc + d0_imm;
        else if (active_thread == 1'b1 && lane0_fire && d0_is_branch && bht_predict_taken)
            t1_next_pc = t1_pc + d0_imm;
        else if (active_thread == 1'b1 && lane0_fire)
            t1_next_pc = lane1_fire ? (t1_pc + 64'd8) : (t1_pc + 64'd4);
        else
            t1_next_pc = t1_pc;
    end

    // ================================================================
    // ---- Commit (per thread, fully independent + parallel) ---------
    // ================================================================
    // Register-file write / RAT commit-clear are entirely independent of
    // active_thread -- both threads retire from their own ROB every
    // cycle, up to 2/cycle each, exactly as Phase 5 established. Only the
    // shared data-memory store port needed cross-thread arbitration (see
    // above); everything else here is a per-thread mirror of Phase 5's
    // single-thread commit logic.
    wire t0_commit_rf_write_en   = t0_rob_commit_req && t0_rob_head_has_dest && (t0_rob_head_rd != 5'd0);
    wire [4:0] t0_commit_rf_write_reg  = t0_rob_head_rd;
    wire [63:0] t0_commit_rf_write_data = t0_rob_head_value;
    assign t0_rat_commit_clear_en  = t0_rob_commit_req && t0_rob_head_has_dest;
    assign t0_rat_commit_rd        = t0_rob_head_rd;
    assign t0_rat_commit_tag       = t0_rob_head_tag;

    wire t0_commit_rf_write_en2   = t0_rob_commit_req2 && t0_rob_head2_has_dest && (t0_rob_head2_rd != 5'd0);
    wire [4:0] t0_commit_rf_write_reg2  = t0_rob_head2_rd;
    wire [63:0] t0_commit_rf_write_data2 = t0_rob_head2_value;
    assign t0_rat_commit_clear_en2  = t0_rob_commit_req2 && t0_rob_head2_has_dest;
    assign t0_rat_commit_rd2        = t0_rob_head2_rd;
    assign t0_rat_commit_tag2       = t0_rob_head2_tag;

    assign ecall_halt0 = t0_rob_commit_req && t0_rob_head_is_ecall;

    wire t1_commit_rf_write_en   = t1_rob_commit_req && t1_rob_head_has_dest && (t1_rob_head_rd != 5'd0);
    wire [4:0] t1_commit_rf_write_reg  = t1_rob_head_rd;
    wire [63:0] t1_commit_rf_write_data = t1_rob_head_value;
    assign t1_rat_commit_clear_en  = t1_rob_commit_req && t1_rob_head_has_dest;
    assign t1_rat_commit_rd        = t1_rob_head_rd;
    assign t1_rat_commit_tag       = t1_rob_head_tag;

    wire t1_commit_rf_write_en2   = t1_rob_commit_req2 && t1_rob_head2_has_dest && (t1_rob_head2_rd != 5'd0);
    wire [4:0] t1_commit_rf_write_reg2  = t1_rob_head2_rd;
    wire [63:0] t1_commit_rf_write_data2 = t1_rob_head2_value;
    assign t1_rat_commit_clear_en2  = t1_rob_commit_req2 && t1_rob_head2_has_dest;
    assign t1_rat_commit_rd2        = t1_rob_head2_rd;
    assign t1_rat_commit_tag2       = t1_rob_head2_tag;

    // ECALL halt is checked against the head only, per thread -- see
    // Phase 5's identical single-thread rationale.
    assign ecall_halt1 = t1_rob_commit_req && t1_rob_head_is_ecall;
endmodule
