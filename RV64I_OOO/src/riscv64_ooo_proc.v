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
// at a time (matching branch_rs.v's own single-entry design, so a single
// RAT checkpoint -- see rat.v -- is always enough; no checkpoint stack).
// JAL never speculates at all, because it doesn't need to: its target is
// exact at decode (pc+imm, no register dependency), so dispatch redirects
// fetch to it immediately and unconditionally -- there is no "prediction"
// to get wrong. JALR keeps Phase 1's conservative stall-until-resolved
// behavior: its target depends on a register value not known until
// execute, and predicting it would need a separate mechanism (a
// branch-target buffer / return-address stack) that's out of scope here.
//
// On a misprediction: every reservation-station bank (alu_rs, mul_rs,
// div_rs, lsq) and the ROB itself squash any entry younger than the
// mispredicted branch (see each bank's squash_valid/squash_tag port,
// using the same wraparound-safe age()-relative-to-rob_head_tag
// comparison as the CDB arbiter), the RAT is restored from its
// checkpoint, and fetch is redirected to the branch's actual (not
// predicted) resolved target -- see rob.v's squash port and rat.v's
// checkpoint_save/checkpoint_restore for what each side does.
//
// Phase 3 scope, deliberately narrower than "double everything": dispatch
// and rename go to 2-wide (lane 0 = older, lane 1 = younger, fetched from
// pc and pc+4 respectively), but the CDB, execution completion, and
// commit all stay exactly 1-wide -- a single reservation-station bank can
// still only broadcast one result per cycle, and the ROB still only
// retires one entry per cycle. This is a real, deliberate design point
// (2-wide front end feeding a 1-wide back end), not a shortcut: it
// delivers the actual ILP win Phase 3 is about (two independent
// instructions renamed and put in flight per cycle, so reservation
// stations stay fed and stalls shrink) without the much larger
// complexity of a fully doubled execute/commit path (multi-port register
// file, dual ROB retire, doubled CDB arbitration). Three further
// simplifications keep the two-lane interaction tractable:
//   - Lane 1 can never itself be a branch-class instruction (conditional
//     branch, JAL, or JALR) -- only lane 0 can. This avoids ever needing
//     to reason about two branches (and two possible redirects) landing
//     in the same fetch group. branch_rs.v accordingly still has just
//     one, lane-0-only allocation port.
//   - Both lanes are fetched every cycle from their straight-line
//     addresses (pc, pc+4) regardless of what lane 0 turns out to be. If
//     lane 0 dispatches as something that breaks that straight-line
//     assumption (JAL, JALR, or a conditional branch predicted taken),
//     lane 1's speculatively-fetched instruction is simply never
//     dispatched that cycle (discarded, not squashed -- it never
//     allocated anything) and gets correctly re-fetched from the real
//     target the following cycle.
//   - Lane 1's own operands need one extra piece of logic beyond
//     everything Phase 1/2 already built: if lane 1 depends on lane 0's
//     destination register (an intra-group RAW hazard -- both
//     instructions rename in the very same cycle, so the RAT's raw read
//     for lane 1 doesn't yet reflect lane 0's not-yet-registered rename),
//     dispatch overrides lane 1's operand source to point directly at
//     lane 0's own newly-allocated ROB tag, the same way it would if that
//     tag had come from the RAT. rat.v's dual read ports (rs1b/rs2b)
//     give the *raw* (pre-lane-0-rename) view; this module layers the
//     intra-group bypass on top, rather than rat.v knowing about lanes at
//     all.
//
// Reused unchanged from the single-cycle RV64I core: program_counter.v,
// instruction_fetch.v (its RVC halfword-awareness is simply unused -- PC
// always advances by exactly 4), register_file.v (written only at
// commit). Both are instantiated twice (once per fetch/read lane) since
// each is a deterministic, read-mostly structure with a single shared
// write path -- two instances fed identical clock/reset/write signals
// always hold identical state, so this is just extra read ports on what
// is logically one array, the same way a real dual-read-port memory
// would be built.
module riscv64_ooo_proc #(
    parameter IMEM_FILE = "instructions.mem",
    parameter DMEM_FILE = "data.mem",
    parameter IMEM_WORDS = 8192,
    parameter DMEM_WORDS = 4096,
    parameter ROB_DEPTH = 8,
    parameter ALU_RS_DEPTH = 4,
    parameter MUL_RS_DEPTH = 2,
    parameter LSQ_DEPTH = 4,
    // Phase 4 benchmarking knob: forces lane 1 to never fire, turning
    // this same RTL into a single-issue machine (everything else
    // identical -- OoO execution, prediction/speculation, ROB/RS sizing)
    // for a direct, apples-to-apples dispatch-width comparison rather
    // than maintaining a separate single-issue copy that could drift.
    parameter ENABLE_DUAL_ISSUE = 1
    // BRANCH_RS and DIV_RS are fixed at exactly 1 entry -- see their
    // headers for why that's not a knob worth exposing.
)(
    input clk,
    input reset,
    output wire [63:0] pc_out,
    output wire ecall_halt
);
    localparam TB = $clog2(ROB_DEPTH);

    // ---- Fetch (2-wide) ---------------------------------------------------
    wire [63:0] pc;
    wire [63:0] pc1 = pc + 64'd4; // lane 1's straight-line fetch address
    reg  [63:0] next_pc;
    wire [31:0] instruction0, instruction1;

    program_counter pc_module (.clk(clk), .reset(reset), .pc_in(next_pc), .pc_out(pc));
    instruction_fetch #(.IMEM_FILE(IMEM_FILE), .IMEM_WORDS(IMEM_WORDS)) if0 (
        .clk(clk), .pc(pc), .instruction(instruction0)
    );
    instruction_fetch #(.IMEM_FILE(IMEM_FILE), .IMEM_WORDS(IMEM_WORDS)) if1 (
        .clk(clk), .pc(pc1), .instruction(instruction1)
    );
    assign pc_out = pc;

    // ---- Decode (2-wide) ----------------------------------------------------
    wire [4:0] d0_rs1, d0_rs2, d0_rd;
    wire [2:0] d0_func3;
    wire [63:0] d0_imm;
    wire d0_is_alu, d0_is_muldiv, d0_is_branch, d0_is_jal, d0_is_jalr, d0_is_load, d0_is_store, d0_is_ecall;
    wire [3:0] d0_alu_op;
    wire d0_word_op;
    wire [2:0] d0_muldiv_op;
    wire d0_reg_write, d0_src2_is_imm, d0_src1_is_zero, d0_src1_is_pc;

    decode_ooo dec0 (
        .instruction(instruction0),
        .rs1(d0_rs1), .rs2(d0_rs2), .rd(d0_rd), .func3(d0_func3), .imm(d0_imm),
        .is_alu(d0_is_alu), .is_muldiv(d0_is_muldiv), .is_branch(d0_is_branch),
        .is_jal(d0_is_jal), .is_jalr(d0_is_jalr), .is_load(d0_is_load), .is_store(d0_is_store),
        .is_ecall(d0_is_ecall),
        .alu_op(d0_alu_op), .word_op(d0_word_op), .muldiv_op(d0_muldiv_op),
        .reg_write(d0_reg_write), .src2_is_imm(d0_src2_is_imm),
        .src1_is_zero(d0_src1_is_zero), .src1_is_pc(d0_src1_is_pc)
    );

    wire [4:0] d1_rs1, d1_rs2, d1_rd;
    wire [2:0] d1_func3;
    wire [63:0] d1_imm;
    wire d1_is_alu, d1_is_muldiv, d1_is_branch, d1_is_jal, d1_is_jalr, d1_is_load, d1_is_store, d1_is_ecall;
    wire [3:0] d1_alu_op;
    wire d1_word_op;
    wire [2:0] d1_muldiv_op;
    wire d1_reg_write, d1_src2_is_imm, d1_src1_is_zero, d1_src1_is_pc;

    decode_ooo dec1 (
        .instruction(instruction1),
        .rs1(d1_rs1), .rs2(d1_rs2), .rd(d1_rd), .func3(d1_func3), .imm(d1_imm),
        .is_alu(d1_is_alu), .is_muldiv(d1_is_muldiv), .is_branch(d1_is_branch),
        .is_jal(d1_is_jal), .is_jalr(d1_is_jalr), .is_load(d1_is_load), .is_store(d1_is_store),
        .is_ecall(d1_is_ecall),
        .alu_op(d1_alu_op), .word_op(d1_word_op), .muldiv_op(d1_muldiv_op),
        .reg_write(d1_reg_write), .src2_is_imm(d1_src2_is_imm),
        .src1_is_zero(d1_src1_is_zero), .src1_is_pc(d1_src1_is_pc)
    );

    wire d0_is_mul = d0_is_muldiv && !d0_muldiv_op[2];
    wire d0_is_div = d0_is_muldiv && d0_muldiv_op[2];
    wire d1_is_mul = d1_is_muldiv && !d1_muldiv_op[2];
    wire d1_is_div = d1_is_muldiv && d1_muldiv_op[2];

    // ---- Register renaming (RAT) + architectural register file ----------
    wire rs1_busy, rs2_busy;
    wire [TB-1:0] rs1_tag, rs2_tag;
    wire rs1b_busy_raw, rs2b_busy_raw;
    wire [TB-1:0] rs1b_tag_raw, rs2b_tag_raw;
    wire rat_write_en;
    wire [TB-1:0] rat_new_tag;
    wire rat_write2_en;
    wire [TB-1:0] rat_new_tag2;
    wire rat_commit_clear_en;
    wire [4:0] rat_commit_rd;
    wire [TB-1:0] rat_commit_tag;

    rat #(.TAG_BITS(TB)) rat_i (
        .clk(clk), .reset(reset),
        .rs1(d0_rs1), .rs2(d0_rs2),
        .rs1_busy(rs1_busy), .rs1_tag(rs1_tag),
        .rs2_busy(rs2_busy), .rs2_tag(rs2_tag),
        .rs1b(d1_rs1), .rs2b(d1_rs2),
        .rs1b_busy(rs1b_busy_raw), .rs1b_tag(rs1b_tag_raw),
        .rs2b_busy(rs2b_busy_raw), .rs2b_tag(rs2b_tag_raw),
        .write_en(rat_write_en), .rd(d0_rd), .new_tag(rat_new_tag),
        .write2_en(rat_write2_en), .rd2(d1_rd), .new_tag2(rat_new_tag2),
        .commit_clear_en(rat_commit_clear_en), .commit_rd(rat_commit_rd), .commit_tag(rat_commit_tag),
        .checkpoint_save(rat_checkpoint_save), .checkpoint_restore(mispredict)
    );

    wire [63:0] rf_read1, rf_read2;
    wire [63:0] rf1_read1, rf1_read2;
    wire commit_rf_write_en;
    wire [4:0] commit_rf_write_reg;
    wire [63:0] commit_rf_write_data;
    register_file regfile0 (
        .clk(clk), .reset(reset),
        .reg_write(commit_rf_write_en),
        .read_reg1(d0_rs1), .read_reg2(d0_rs2), .write_reg(commit_rf_write_reg),
        .write_data(commit_rf_write_data),
        .read_data1(rf_read1), .read_data2(rf_read2)
    );
    register_file regfile1 (
        .clk(clk), .reset(reset),
        .reg_write(commit_rf_write_en),
        .read_reg1(d1_rs1), .read_reg2(d1_rs2), .write_reg(commit_rf_write_reg),
        .write_data(commit_rf_write_data),
        .read_data1(rf1_read1), .read_data2(rf1_read2)
    );

    // ---- Reorder buffer (dual allocation) ---------------------------------
    wire rob_alloc_req, rob_alloc2_req;
    wire [TB-1:0] rob_alloc_tag, rob_alloc2_tag;
    wire [TB:0] rob_free_count;
    wire rob_mark_valid;
    wire [TB-1:0] rob_mark_tag;
    wire [63:0] rob_mark_value;
    wire rob_mark2_valid;
    wire [TB-1:0] rob_mark2_tag;
    wire rob_head_ready;
    wire [TB-1:0] rob_head_tag;
    wire rob_head_has_dest;
    wire [4:0] rob_head_rd;
    wire [63:0] rob_head_value;
    wire rob_head_is_store, rob_head_is_ecall;
    wire rob_commit_req;
    wire rob_rs1_done, rob_rs2_done, rob_rs1b_done, rob_rs2b_done;
    wire [63:0] rob_rs1_value, rob_rs2_value, rob_rs1b_value, rob_rs2b_value;
    wire [LSQ_DEPTH-1:0] lsq_store_ready;
    wire [LSQ_DEPTH*TB-1:0] lsq_store_ready_tag_flat;

    rob #(.DEPTH(ROB_DEPTH), .EXTRA_MARK_N(LSQ_DEPTH)) rob_i (
        .clk(clk), .reset(reset),
        .alloc_req(rob_alloc_req), .alloc_has_dest(d0_reg_write), .alloc_rd(d0_rd),
        .alloc_is_store(d0_is_store), .alloc_is_ecall(d0_is_ecall),
        .alloc_tag(rob_alloc_tag),
        .alloc2_req(rob_alloc2_req), .alloc2_has_dest(d1_reg_write), .alloc2_rd(d1_rd),
        .alloc2_is_store(d1_is_store), .alloc2_is_ecall(d1_is_ecall),
        .alloc2_tag(rob_alloc2_tag), .free_count(rob_free_count),
        .mark_valid(rob_mark_valid), .mark_tag(rob_mark_tag), .mark_value(rob_mark_value),
        .mark2_valid(rob_mark2_valid), .mark2_tag(rob_mark2_tag),
        .extra_mark_valid(lsq_store_ready), .extra_mark_tag_flat(lsq_store_ready_tag_flat),
        .squash_valid(mispredict), .squash_tag(branch_resolved_tag),
        .lookup1_tag(rs1_tag), .lookup1_done(rob_rs1_done), .lookup1_value(rob_rs1_value),
        .lookup2_tag(rs2_tag), .lookup2_done(rob_rs2_done), .lookup2_value(rob_rs2_value),
        .lookup3_tag(rs1b_tag_raw), .lookup3_done(rob_rs1b_done), .lookup3_value(rob_rs1b_value),
        .lookup4_tag(rs2b_tag_raw), .lookup4_done(rob_rs2b_done), .lookup4_value(rob_rs2b_value),
        .head_ready(rob_head_ready), .head_tag(rob_head_tag), .head_has_dest(rob_head_has_dest),
        .head_rd(rob_head_rd), .head_value(rob_head_value),
        .head_is_store(rob_head_is_store), .head_is_ecall(rob_head_is_ecall),
        .commit_req(rob_commit_req)
    );

    // ---- CDB arbiter ----------------------------------------------------
    // Oldest-ROB-tag-first: "older" is distance-from-the-*current* ROB
    // head, computed with wraparound-correct unsigned subtraction (tag -
    // head, mod ROB_DEPTH) -- NOT raw numeric tag comparison, which would
    // be wrong across a wraparound (a just-allocated, youngest entry can
    // have a numerically smaller tag than an older one still in flight).
    // This policy is more than "a" reasonable fairness choice: the ROB
    // head, when it's itself a requester, always has age 0 and therefore
    // always wins immediately, which is exactly the property that keeps
    // in-order commit from ever being needlessly delayed by arbitration.
    // Still exactly 1-wide (one broadcast per cycle) regardless of Phase
    // 3's 2-wide dispatch -- see module header.
    function [TB-1:0] age;
        input [TB-1:0] tag;
        begin
            age = tag - rob_head_tag;
        end
    endfunction

    wire cdb_valid;
    wire [TB-1:0] cdb_tag;
    wire [63:0] cdb_value;

    // ---- ALU reservation-station bank + functional unit ------------------
    wire alu_rs_full, alu_rs_has_2_free;
    wire alu_req_valid;
    wire [TB-1:0] alu_req_tag;
    wire [63:0] alu_req_value;
    wire alu_grant;

    alu_rs #(.DEPTH(ALU_RS_DEPTH), .TAG_BITS(TB)) alu_rs_i (
        .clk(clk), .reset(reset),
        .alloc_req(lane0_fire && d0_is_alu),
        .alloc_op(d0_alu_op), .alloc_word_op(d0_word_op),
        .alloc_src1_ready(lane0_src1_ready), .alloc_src1_val(lane0_src1_val), .alloc_src1_tag(rs1_tag),
        .alloc_src2_ready(lane0_src2_ready), .alloc_src2_val(lane0_src2_val), .alloc_src2_tag(rs2_tag),
        .alloc_dest_tag(rob_alloc_tag),
        .full(alu_rs_full),
        .alloc2_req(lane1_fire && d1_is_alu),
        .alloc2_op(d1_alu_op), .alloc2_word_op(d1_word_op),
        .alloc2_src1_ready(lane1_src1_ready), .alloc2_src1_val(lane1_src1_val), .alloc2_src1_tag(lane1_src1_tag),
        .alloc2_src2_ready(lane1_src2_ready), .alloc2_src2_val(lane1_src2_val), .alloc2_src2_tag(lane1_src2_tag),
        .alloc2_dest_tag(rob_alloc2_tag),
        .has_2_free(alu_rs_has_2_free),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_value(cdb_value),
        .req_valid(alu_req_valid), .req_tag(alu_req_tag), .req_value(alu_req_value),
        .req_grant(alu_grant),
        .rob_head_tag(rob_head_tag), .squash_valid(mispredict), .squash_tag(branch_resolved_tag)
    );

    // ---- Multiply reservation-station bank + functional unit -------------
    wire mul_rs_full, mul_rs_has_2_free;
    wire mul_req_valid;
    wire [TB-1:0] mul_req_tag;
    wire [63:0] mul_req_value;
    wire mul_grant;

    mul_rs #(.DEPTH(MUL_RS_DEPTH), .TAG_BITS(TB)) mul_rs_i (
        .clk(clk), .reset(reset),
        .alloc_req(lane0_fire && d0_is_mul),
        .alloc_op(d0_muldiv_op), .alloc_word_op(d0_word_op),
        .alloc_src1_ready(lane0_src1_ready), .alloc_src1_val(lane0_src1_val), .alloc_src1_tag(rs1_tag),
        .alloc_src2_ready(lane0_src2_ready), .alloc_src2_val(lane0_src2_val), .alloc_src2_tag(rs2_tag),
        .alloc_dest_tag(rob_alloc_tag),
        .full(mul_rs_full),
        .alloc2_req(lane1_fire && d1_is_mul),
        .alloc2_op(d1_muldiv_op), .alloc2_word_op(d1_word_op),
        .alloc2_src1_ready(lane1_src1_ready), .alloc2_src1_val(lane1_src1_val), .alloc2_src1_tag(lane1_src1_tag),
        .alloc2_src2_ready(lane1_src2_ready), .alloc2_src2_val(lane1_src2_val), .alloc2_src2_tag(lane1_src2_tag),
        .alloc2_dest_tag(rob_alloc2_tag),
        .has_2_free(mul_rs_has_2_free),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_value(cdb_value),
        .req_valid(mul_req_valid), .req_tag(mul_req_tag), .req_value(mul_req_value),
        .req_grant(mul_grant),
        .rob_head_tag(rob_head_tag), .squash_valid(mispredict), .squash_tag(branch_resolved_tag)
    );

    // ---- Divide reservation station + functional unit (multi-cycle) ------
    // Still exactly 1 entry -- unlike alu_rs/mul_rs/lsq, this bank cannot
    // accept both lanes in the same cycle even in principle. If both lanes
    // happen to be divides, lane 1 simply doesn't dispatch this cycle
    // (see lane1_div_ok below); otherwise, whichever lane actually wants
    // it (lane 0 or lane 1, never both) feeds this single allocation port
    // via the mux below.
    wire div_rs_full;
    wire div_req_valid;
    wire [TB-1:0] div_req_tag;
    wire [63:0] div_req_value;
    wire div_grant;

    wire div_from_lane0 = lane0_fire && d0_is_div;
    wire div_from_lane1 = lane1_fire && d1_is_div;

    div_rs #(.TAG_BITS(TB)) div_rs_i (
        .clk(clk), .reset(reset),
        .alloc_req(div_from_lane0 || div_from_lane1),
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
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_value(cdb_value),
        .req_valid(div_req_valid), .req_tag(div_req_tag), .req_value(div_req_value),
        .req_grant(div_grant),
        .rob_head_tag(rob_head_tag), .squash_valid(mispredict), .squash_tag(branch_resolved_tag)
    );

    // ---- Load-Store Queue + data memory -----------------------------------
    // Loads present a real destination register and a real rs1 (base
    // address register); rs2's raw instruction field for a load isn't a
    // real source register at all (I-type's bits[24:20] are part of the
    // immediate, not a register field, same category of non-register
    // field as LUI/AUIPC's src1) -- presented as trivially data-ready so
    // dispatch never renames/waits on it. Stores use a real rs1 (base)
    // and rs2 (data), same as any other 2-operand RS entry.
    wire lsq_full, lsq_has_2_free;
    wire lsq_req_valid;
    wire [TB-1:0] lsq_req_tag;
    wire [63:0] lsq_req_value;
    wire lsq_grant;
    wire lsq_mem_read_req;
    wire [63:0] lsq_mem_read_addr;
    wire [2:0] lsq_mem_read_func3;
    wire [63:0] dmem_read_data;
    wire lsq_commit_match;
    wire [63:0] lsq_commit_addr, lsq_commit_data;
    wire [2:0] lsq_commit_func3;

    wire lane0_data_ready = d0_is_load || lane0_src2_ready;
    wire lane1_data_ready = d1_is_load || lane1_src2_ready;

    lsq #(.DEPTH(LSQ_DEPTH), .TAG_BITS(TB)) lsq_i (
        .clk(clk), .reset(reset),
        .alloc_req(lane0_fire && (d0_is_load || d0_is_store)),
        .alloc_is_store(d0_is_store), .alloc_func3(d0_func3), .alloc_imm(d0_imm),
        .alloc_base_ready(lane0_src1_ready), .alloc_base_val(lane0_src1_val), .alloc_base_tag(rs1_tag),
        .alloc_data_ready(lane0_data_ready), .alloc_data_val(lane0_src2_val), .alloc_data_tag(rs2_tag),
        .alloc_dest_tag(rob_alloc_tag),
        .full(lsq_full),
        .alloc2_req(lane1_fire && (d1_is_load || d1_is_store)),
        .alloc2_is_store(d1_is_store), .alloc2_func3(d1_func3), .alloc2_imm(d1_imm),
        .alloc2_base_ready(lane1_src1_ready), .alloc2_base_val(lane1_src1_val), .alloc2_base_tag(lane1_src1_tag),
        .alloc2_data_ready(lane1_data_ready), .alloc2_data_val(lane1_src2_val), .alloc2_data_tag(lane1_src2_tag),
        .alloc2_dest_tag(rob_alloc2_tag),
        .has_2_free(lsq_has_2_free),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_value(cdb_value),
        .rob_head_tag(rob_head_tag),
        .req_valid(lsq_req_valid), .req_tag(lsq_req_tag), .req_value(lsq_req_value),
        .req_grant(lsq_grant),
        .mem_port_busy(dmem_write_en),
        .mem_read_req(lsq_mem_read_req), .mem_read_addr(lsq_mem_read_addr), .mem_read_func3(lsq_mem_read_func3),
        .mem_read_data(dmem_read_data),
        .store_ready(lsq_store_ready), .store_ready_tag_flat(lsq_store_ready_tag_flat),
        .commit_match(lsq_commit_match), .commit_addr(lsq_commit_addr), .commit_data(lsq_commit_data),
        .commit_func3(lsq_commit_func3),
        .commit_fire(dmem_write_en),
        .squash_valid(mispredict), .squash_tag(branch_resolved_tag)
    );

    // A store's ROB entry becoming commit-eligible (rob_head_ready &&
    // rob_head_is_store) is exactly the signal to perform its memory
    // write -- rob_commit_req is already wired as rob_head_ready below,
    // so this fires the same cycle the head actually retires.
    wire dmem_write_en = rob_head_ready && rob_head_is_store;

    data_memory #(.DMEM_FILE(DMEM_FILE), .DMEM_WORDS(DMEM_WORDS)) dmem_i (
        .clk(clk),
        .mem_read(lsq_mem_read_req), .mem_write(dmem_write_en),
        .func3(dmem_write_en ? lsq_commit_func3 : lsq_mem_read_func3),
        .mem_addr(dmem_write_en ? lsq_commit_addr : lsq_mem_read_addr),
        .write_data(lsq_commit_data),
        .read_data(dmem_read_data),
        .vmem_read(1'b0), .vmem_write(1'b0), .vmem_addr(64'b0), .vmem_write_data(128'b0)
    );

    // ---- Branch-class reservation station (branches/JAL/JALR) -----------
    // Lane-0-only allocation port -- lane 1 can never be branch-class
    // (Phase 3 scoping, see module header), so no mux/dual-port is needed
    // here the way div_rs above needs one.
    wire branch_rs_full;
    wire branch_resolved;
    wire [63:0] branch_resolved_next_pc;
    wire [TB-1:0] branch_resolved_tag;
    wire branch_resolved_has_result;
    wire branch_req_valid;
    wire [TB-1:0] branch_req_tag;
    wire [63:0] branch_req_value;
    wire branch_grant;
    wire [63:0] branch_resolved_pc;
    wire branch_resolved_taken;
    wire branch_resident_is_jalr;

    // JAL needs no register operands at all (target/return-value depend
    // only on pc+imm); JALR needs src1 only; conditional branches need
    // both. Presented as "trivially ready" for whichever operand(s) this
    // instruction doesn't actually need, the same pattern decode_ooo.v /
    // dispatch already use for LUI/AUIPC's non-register src1.
    wire branch_src1_ready = d0_is_jal || lane0_src1_ready;
    wire branch_src2_ready = !d0_is_branch || lane0_src2_ready;

    branch_rs #(.TAG_BITS(TB)) branch_rs_i (
        .clk(clk), .reset(reset),
        .alloc_req(lane0_fire && (d0_is_branch || d0_is_jal || d0_is_jalr)),
        .alloc_func3(d0_func3),
        .alloc_is_jal(d0_is_jal), .alloc_is_jalr(d0_is_jalr), .alloc_is_branch(d0_is_branch),
        .alloc_pc(pc), .alloc_imm(d0_imm),
        .alloc_src1_ready(branch_src1_ready), .alloc_src1_val(lane0_src1_val), .alloc_src1_tag(rs1_tag),
        .alloc_src2_ready(branch_src2_ready), .alloc_src2_val(lane0_src2_val), .alloc_src2_tag(rs2_tag),
        .alloc_dest_tag(rob_alloc_tag),
        .full(branch_rs_full),
        .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_value(cdb_value),
        .resolved(branch_resolved), .resolved_next_pc(branch_resolved_next_pc),
        .resolved_tag(branch_resolved_tag), .resolved_has_result(branch_resolved_has_result),
        .resolved_pc(branch_resolved_pc), .resolved_taken(branch_resolved_taken),
        .req_valid(branch_req_valid), .req_tag(branch_req_tag), .req_value(branch_req_value),
        .req_grant(branch_grant),
        .resident_is_jalr(branch_resident_is_jalr)
    );

    // A resolved conditional branch has no destination register and thus
    // never wins (or even requests) the CDB -- without this, its ROB
    // entry would never be marked done and commit would stall forever
    // the instant it reached the head. Routed through rob.v's separate,
    // unarbitrated mark2 port (see rob.v's header for why this needs to
    // be genuinely separate from the CDB-arbitrated mark port, not
    // merged behind it) since it can fire the same cycle the CDB is
    // independently broadcasting something else's value entirely.
    assign rob_mark2_valid = branch_resolved && !branch_resolved_has_result;
    assign rob_mark2_tag   = branch_resolved_tag;

    // Oldest-tag-first across all five requesters. Reduced pairwise: pick
    // the oldest of {alu, branch}, then the oldest of {that, mul}, then
    // the oldest of {that, div}, then the oldest of {that, lsq (loads
    // only -- stores never request)}. Ties (impossible in practice --
    // tags are always distinct) fall to whichever comparison runs first,
    // same as the original 2-way version.
    wire alu_vs_branch_wins_alu = alu_req_valid && (!branch_req_valid || (age(alu_req_tag) <= age(branch_req_tag)));
    wire [TB-1:0] ab_tag   = alu_vs_branch_wins_alu ? alu_req_tag   : branch_req_tag;
    wire [63:0]   ab_value = alu_vs_branch_wins_alu ? alu_req_value : branch_req_value;
    wire ab_valid = alu_req_valid || branch_req_valid;

    wire ab_wins = ab_valid && (!mul_req_valid || (age(ab_tag) <= age(mul_req_tag)));
    wire [TB-1:0] abm_tag   = ab_wins ? ab_tag   : mul_req_tag;
    wire [63:0]   abm_value = ab_wins ? ab_value : mul_req_value;
    wire abm_valid = ab_valid || mul_req_valid;

    wire abm_wins = abm_valid && (!div_req_valid || (age(abm_tag) <= age(div_req_tag)));
    wire [TB-1:0] abmd_tag   = abm_wins ? abm_tag   : div_req_tag;
    wire [63:0]   abmd_value = abm_wins ? abm_value : div_req_value;
    wire abmd_valid = abm_valid || div_req_valid;

    wire abmd_wins = abmd_valid && (!lsq_req_valid || (age(abmd_tag) <= age(lsq_req_tag)));

    assign alu_grant    = alu_vs_branch_wins_alu && ab_wins && abm_wins && abmd_wins;
    assign branch_grant = !alu_vs_branch_wins_alu && ab_wins && abm_wins && abmd_wins && branch_req_valid;
    assign mul_grant    = mul_req_valid && !ab_wins && abm_wins && abmd_wins;
    assign div_grant    = div_req_valid && !abm_wins && abmd_wins;
    assign lsq_grant     = lsq_req_valid && !abmd_wins;

    assign cdb_valid = abmd_valid || lsq_req_valid;
    assign cdb_tag   = abmd_wins ? abmd_tag   : lsq_req_tag;
    assign cdb_value = abmd_wins ? abmd_value : lsq_req_value;

    assign rob_mark_valid = cdb_valid;
    assign rob_mark_tag   = cdb_tag;
    assign rob_mark_value = cdb_value;

    // ---- Branch prediction + speculation (Phase 2) ------------------------
    // JALR still fully stalls dispatch until it resolves (see module
    // header) -- derived combinationally from branch_rs's own resident
    // state rather than a separate tracked register, since branch_rs.full
    // already only ever holds one branch-class instruction at a time.
    wire jalr_outstanding = branch_resident_is_jalr;

    // BHT: predicts conditional branches only, and only ever for lane 0
    // (the only lane that can be branch-class). Read combinationally for
    // whatever lane 0 currently is (used both to steer fetch when a
    // conditional branch itself dispatches, and latched into
    // predicted_taken_reg for later comparison against the actual
    // outcome). Updated once per resolved conditional branch, correct or
    // not -- gated on spec_active, which by construction is only ever set
    // for conditional branches, never JAL/JALR.
    wire bht_predict_taken;
    bht bht_i (
        .clk(clk), .reset(reset),
        .predict_pc(pc), .predict_taken(bht_predict_taken),
        .update_valid(spec_active && branch_resolved),
        .update_pc(branch_resolved_pc), .update_taken(branch_resolved_taken)
    );

    // spec_active: exactly one conditional branch may be speculatively
    // outstanding at a time (branch_rs's own single-entry sizing already
    // guarantees no second branch-class instruction can dispatch in the
    // meantime, so this never needs to track more than one). Cleared the
    // cycle it resolves, whether the prediction was right or wrong.
    reg spec_active;
    reg predicted_taken_reg;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            spec_active <= 1'b0;
        end else if (lane0_fire && d0_is_branch) begin
            spec_active <= 1'b1;
            predicted_taken_reg <= bht_predict_taken;
        end else if (branch_resolved) begin
            spec_active <= 1'b0;
        end
    end

    // Misprediction: the just-resolved conditional branch's actual
    // outcome differs from what was predicted at its own dispatch time.
    // squash_valid/squash_tag on every RS bank and the ROB, and
    // checkpoint_restore on the RAT, are all driven directly from this
    // (and branch_resolved_tag) -- see each module's own port for what it
    // does with it.
    wire mispredict = spec_active && branch_resolved && (branch_resolved_taken != predicted_taken_reg);

    // Checkpoint the RAT the same cycle a conditional branch (lane 0
    // only) dispatches -- captures "what the RAT looked like right before
    // any of this branch's younger, speculative instructions (including a
    // same-cycle lane 1, if one also dispatches) could rename anything",
    // which is exactly the state a misprediction needs to roll back to.
    // Safe regardless of lane 1's own activity this cycle: checkpoint_save
    // reads pre-edge busy[]/tag[] values (see rat.v), so it captures state
    // from *before* this cycle's write_en/write2_en take effect no matter
    // what else dispatches alongside the branch.
    wire rat_checkpoint_save = lane0_fire && d0_is_branch;

    // Fetch only needs an explicit redirect for two cases: a
    // misprediction (roll back to the branch's actual target), or a
    // JALR resolving (fetch was fully stalled the whole time waiting
    // specifically for it, same as Phase 1's unconditional redirect). A
    // *correctly* predicted conditional branch needs no redirect at all
    // -- fetch already followed the predicted path on its own, and
    // JAL's redirect already happened immediately at its own dispatch
    // (see the next_pc mux below), not here.
    wire redirect_needed = mispredict || (jalr_outstanding && branch_resolved);

    // ---- Dispatch (2-wide) -------------------------------------------------
    // Lane 0: identical structure to Phase 1/2's single-issue dispatch,
    // just renamed with a d0_ prefix and using rob_free_count instead of
    // !rob_full (equivalent for a single allocation).
    wire lane0_dispatchable = d0_is_alu || d0_is_branch || d0_is_jal || d0_is_jalr || d0_is_mul || d0_is_div ||
                               d0_is_load || d0_is_store;
    wire lane0_needed_rs_full = (d0_is_alu && alu_rs_full) ||
                                 ((d0_is_branch || d0_is_jal || d0_is_jalr) && branch_rs_full) ||
                                 (d0_is_mul && mul_rs_full) ||
                                 (d0_is_div && div_rs_full) ||
                                 ((d0_is_load || d0_is_store) && lsq_full);
    // !mispredict: the instruction currently being decoded was fetched
    // along the (just-discovered-wrong) speculative path, so it must
    // never be allowed to dispatch/rename/allocate -- this is what
    // actually flushes the "in-flight this exact cycle" instruction, on
    // top of the squash ports flushing everything already dispatched.
    wire lane0_fire = lane0_dispatchable && (rob_free_count >= 1) && !lane0_needed_rs_full &&
                       !jalr_outstanding && !mispredict;

    // Lane 1: only considered at all if lane 0 also fires this cycle (no
    // skipping lane 0 -- dispatch stays in-order), is never branch-class
    // (see module header), and never dispatches the cycle lane 0 itself
    // breaks the straight-line fetch assumption lane 1 was speculatively
    // fetched under (JAL, JALR, or a taken-predicted conditional branch --
    // lane 1's *content* in that case came from the wrong address
    // entirely, not just a resource conflict).
    wire lane0_breaks_flow = d0_is_jal || d0_is_jalr || (d0_is_branch && bht_predict_taken);
    wire lane1_dispatchable = d1_is_alu || d1_is_mul || d1_is_div || d1_is_load || d1_is_store;

    // Per-bank: can lane 1 actually get a slot, given lane 0 may be
    // claiming one in the very same bank this same cycle? A multi-entry
    // bank (alu_rs/mul_rs/lsq) needs has_2_free if lane 0 also wants it,
    // or just its normal single-free check otherwise. div_rs is a single
    // physical slot -- if lane 0 also wants it, lane 1 categorically
    // cannot, regardless of div_rs_full's value.
    wire lane1_alu_ok = !d1_is_alu || (d0_is_alu ? alu_rs_has_2_free : !alu_rs_full);
    wire lane1_mul_ok = !d1_is_mul || (d0_is_mul ? mul_rs_has_2_free : !mul_rs_full);
    wire lane1_div_ok = !d1_is_div || (!d0_is_div && !div_rs_full);
    wire lane1_lsq_ok = !(d1_is_load || d1_is_store) ||
                        ((d0_is_load || d0_is_store) ? lsq_has_2_free : !lsq_full);
    wire lane1_resource_ok = lane1_alu_ok && lane1_mul_ok && lane1_div_ok && lane1_lsq_ok;

    wire lane1_fire = ENABLE_DUAL_ISSUE && lane0_fire && !lane0_breaks_flow && lane1_dispatchable &&
                       (rob_free_count >= 2) && lane1_resource_ok;

    // src1/src2 readiness at dispatch: a register is ready if it's not
    // currently renamed (RAT says not busy -> read the committed value
    // straight from register_file.v), OR -- the CDB-bypass case -- it IS
    // renamed but this exact cycle's CDB broadcast happens to be carrying
    // its producer's result. Without this bypass, a same-cycle "producer
    // broadcasts tag X" + "consumer dispatches waiting on tag X" would
    // permanently miss each other: the CDB is a one-shot pulse, not a
    // latched bus, and the consumer's RS entry doesn't even exist yet at
    // the moment of the broadcast to snoop it normally. Found by tracing
    // this exact scenario by hand before it could become a real deadlock
    // in simulation.
    //
    // A third case, found the same way (hand-tracing a stuck backward-
    // branch loop): a producer's tag can broadcast on the CDB, then the
    // consumer dispatches one or more cycles *later* -- past the one-shot
    // CDB pulse, but still before the producer's ROB entry actually
    // commits and clears the RAT's busy/tag mapping. In that gap, rs_busy
    // is still (correctly) 1 but the tag will never broadcast again, so a
    // consumer that only checks the same-cycle CDB bypass parks itself
    // waiting on an event that already happened -- a real deadlock, not a
    // corner case (any producer/consumer pair separated by enough other
    // dispatched instructions hits this). Fixed by also checking the
    // ROB's own done_arr/value_arr for the tag directly: once an entry is
    // marked done, its value is valid there continuously until commit,
    // unlike the CDB's one-shot broadcast.
    wire lane0_src1_ready = d0_src1_is_zero || d0_src1_is_pc || !rs1_busy || (cdb_valid && cdb_tag == rs1_tag) || rob_rs1_done;
    wire [63:0] lane0_src1_val = d0_src1_is_zero ? 64'b0 :
                                  d0_src1_is_pc   ? pc :
                                  !rs1_busy       ? rf_read1 :
                                  (cdb_valid && cdb_tag == rs1_tag) ? cdb_value :
                                  rob_rs1_done    ? rob_rs1_value :
                                  64'b0; // don't-care: tag (rs1_tag) is used instead

    wire lane0_src2_ready = d0_src2_is_imm || !rs2_busy || (cdb_valid && cdb_tag == rs2_tag) || rob_rs2_done;
    wire [63:0] lane0_src2_val = d0_src2_is_imm ? d0_imm :
                                  !rs2_busy      ? rf_read2 :
                                  (cdb_valid && cdb_tag == rs2_tag) ? cdb_value :
                                  rob_rs2_done   ? rob_rs2_value :
                                  64'b0;

    // Lane 1's operands need one more case beyond lane 0's three: an
    // intra-group RAW hazard on lane 0's own destination register. Both
    // instructions rename in the very same cycle, so rat.v's raw rs1b/rs2b
    // read (rs1b_busy_raw/rs1b_tag_raw etc.) reflects the RAT as it stood
    // *before* lane 0's own rename this cycle -- it cannot see lane 0's
    // not-yet-registered write. If lane 1 reads the same register lane 0
    // writes, its true dependency is on lane 0's own (this-cycle,
    // newly-allocated) ROB tag directly, not whatever the raw RAT read
    // says -- and it is unconditionally *not* ready yet, since lane 0
    // itself has not executed anything at its own dispatch time (its
    // result only becomes known later, via the normal CDB broadcast path,
    // exactly like any other RAW producer).
    wire lane1_rs1_intra_hit = d0_reg_write && (d1_rs1 == d0_rd) && (d0_rd != 5'd0);
    wire lane1_rs2_intra_hit = d0_reg_write && (d1_rs2 == d0_rd) && (d0_rd != 5'd0);

    wire lane1_raw_src1_ready = !rs1b_busy_raw || (cdb_valid && cdb_tag == rs1b_tag_raw) || rob_rs1b_done;
    wire [63:0] lane1_raw_src1_val = !rs1b_busy_raw ? rf1_read1 :
                                      (cdb_valid && cdb_tag == rs1b_tag_raw) ? cdb_value :
                                      rob_rs1b_done  ? rob_rs1b_value :
                                      64'b0;

    wire lane1_src1_ready = d1_src1_is_zero || d1_src1_is_pc ||
                             (lane1_rs1_intra_hit ? 1'b0 : lane1_raw_src1_ready);
    wire [63:0] lane1_src1_val = d1_src1_is_zero ? 64'b0 :
                                  d1_src1_is_pc   ? pc1 :
                                  lane1_rs1_intra_hit ? 64'b0 :
                                  lane1_raw_src1_val;
    wire [TB-1:0] lane1_src1_tag = lane1_rs1_intra_hit ? rob_alloc_tag : rs1b_tag_raw;

    wire lane1_raw_src2_ready = !rs2b_busy_raw || (cdb_valid && cdb_tag == rs2b_tag_raw) || rob_rs2b_done;
    wire [63:0] lane1_raw_src2_val = !rs2b_busy_raw ? rf1_read2 :
                                      (cdb_valid && cdb_tag == rs2b_tag_raw) ? cdb_value :
                                      rob_rs2b_done  ? rob_rs2b_value :
                                      64'b0;

    wire lane1_src2_ready = d1_src2_is_imm || (lane1_rs2_intra_hit ? 1'b0 : lane1_raw_src2_ready);
    wire [63:0] lane1_src2_val = d1_src2_is_imm ? d1_imm :
                                  lane1_rs2_intra_hit ? 64'b0 :
                                  lane1_raw_src2_val;
    wire [TB-1:0] lane1_src2_tag = lane1_rs2_intra_hit ? rob_alloc_tag : rs2b_tag_raw;

    assign rob_alloc_req  = lane0_fire;
    assign rob_alloc2_req = lane1_fire;
    assign rat_write_en   = lane0_fire && d0_reg_write; // rat.v itself skips rd==0
    assign rat_new_tag    = rob_alloc_tag;
    assign rat_write2_en  = lane1_fire && d1_reg_write;
    assign rat_new_tag2   = rob_alloc2_tag;

    // JAL's target is exact and register-independent, so dispatch
    // redirects fetch to it immediately and unconditionally -- no
    // prediction, no possibility of misprediction, nothing to roll back
    // (lane 1 never dispatches this cycle either way -- lane0_breaks_flow
    // forces lane1_fire to 0). A conditional branch, when it dispatches,
    // steers fetch down the *predicted* path if taken (still speculative
    // -- may yet be squashed later, see redirect_needed above); if
    // predicted not-taken, falls through to the general case below, which
    // correctly advances by 8 or 4 depending on whether lane 1 also fired.
    // JALR falls through the same way -- lane1_fire is already forced 0
    // for it via lane0_breaks_flow, so the general case's "+4" is exactly
    // Phase 1/2's original stall-in-place-until-resolved behavior.
    always @(*) begin
        if (redirect_needed)
            next_pc = branch_resolved_next_pc;
        else if (lane0_fire && d0_is_jal)
            next_pc = pc + d0_imm;
        else if (lane0_fire && d0_is_branch && bht_predict_taken)
            next_pc = pc + d0_imm;
        else if (lane0_fire)
            next_pc = lane1_fire ? (pc + 64'd8) : (pc + 64'd4);
        else
            next_pc = pc;
    end

    // ---- Commit (in-order retirement, still 1-wide -- see module header) --
    assign rob_commit_req = rob_head_ready;
    assign commit_rf_write_en   = rob_head_ready && rob_head_has_dest && (rob_head_rd != 5'd0);
    assign commit_rf_write_reg  = rob_head_rd;
    assign commit_rf_write_data = rob_head_value;
    assign rat_commit_clear_en  = rob_head_ready && rob_head_has_dest;
    assign rat_commit_rd        = rob_head_rd;
    assign rat_commit_tag       = rob_head_tag;

    assign ecall_halt = rob_head_ready && rob_head_is_ecall;
endmodule
