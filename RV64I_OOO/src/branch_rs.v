`timescale 1ns / 1ps
// Branch-class reservation station: conditional branches, JAL, JALR.
// Exactly 1 entry, not "1-2" -- correct *by construction* in Phase 1,
// since a branch-class instruction blocks dispatch of anything younger
// until it resolves, so a second branch can never be dispatched while
// the first is outstanding; a second slot would be permanently dead
// capacity. No speculation: the redirect this module computes is always
// architecturally correct (nothing younger was ever fetched on a wrong
// path to begin with), so there's no misprediction/squash logic here --
// that's what Phase 2 (branch prediction + speculation) adds on top of
// this same structure.
//
// JAL needs no register operands at all (target = pc+imm, return value =
// pc+4) -- the caller presents it as trivially ready regardless of the
// raw rs1/rs2 instruction fields, the same way decode_ooo.v/dispatch
// treat LUI/AUIPC's non-register src1. JALR needs src1 only (its "src2"
// is the immediate, passed separately). Conditional branches need both
// src1 and src2 (the two comparison operands) and produce no result.
//
// The entry stays occupied (blocking new branch-class dispatch) until
// its result, if any, is actually broadcast and granted on the CDB --
// not just "resolved" -- specifically so a JAL/JALR's pending return-
// address broadcast can never be silently lost by a new branch reusing
// this single slot before the old one's value has actually gone out.
module branch_rs #(
    parameter TAG_BITS = 3,
    // Phase 7 (SMT): unlike the shared banks (alu_rs etc.), this module
    // stays single-thread-scoped -- one full instance per thread (see
    // riscv64_ooo_proc.v's header) -- so it needs no per-entry tid
    // storage. It DOES need to know its OWN fixed thread identity,
    // though: it now snoops the SAME shared cdbA/cdbB buses those shared
    // banks broadcast on, and (just like them) ROB tags are only unique
    // *within* a thread, so a numeric tag match alone isn't enough to
    // know a broadcast is really this instance's own operand rather than
    // the other thread's unrelated, coincidentally-same-numbered one.
    // MY_TID is a compile-time constant (0 for the thread-0 instance, 1
    // for thread-1's), not a per-entry runtime signal, since a given
    // instance's thread never changes.
    parameter MY_TID = 0
)(
    input clk,
    input reset,

    input alloc_req,
    input [2:0] alloc_func3,
    input alloc_is_jal,
    input alloc_is_jalr,
    input alloc_is_branch,
    input [63:0] alloc_pc,
    input [63:0] alloc_imm,
    input alloc_src1_ready,
    input [63:0] alloc_src1_val,
    input [TAG_BITS-1:0] alloc_src1_tag,
    input alloc_src2_ready,
    input [63:0] alloc_src2_val,
    input [TAG_BITS-1:0] alloc_src2_tag,
    input [TAG_BITS-1:0] alloc_dest_tag,
    output full,

    // Two independent CDB snoop buses -- see alu_rs.v's identical port.
    // cdbA_tid/cdbB_tid are compared against MY_TID (above), not stored
    // per-entry, since this instance only ever holds its own thread's
    // entries.
    input cdbA_valid,
    input cdbA_tid,
    input [TAG_BITS-1:0] cdbA_tag,
    input [63:0] cdbA_value,
    input cdbB_valid,
    input cdbB_tid,
    input [TAG_BITS-1:0] cdbB_tag,
    input [63:0] cdbB_value,

    // Pulses the cycle this entry actually vacates: for a conditional
    // branch, the same cycle its operands become ready (no CDB wait --
    // no result to deliver); for JAL/JALR, the cycle its return-address
    // broadcast is granted.
    output resolved,
    output [63:0] resolved_next_pc,
    output [TAG_BITS-1:0] resolved_tag,
    output resolved_has_result, // 0 for a conditional branch (feeds rob.v's
                                  // mark2 port instead of the CDB -- see
                                  // riscv64_ooo_proc.v), 1 for JAL/JALR
    output [63:0] resolved_pc,   // Phase 2: BHT update indexing
    output resolved_taken,       // Phase 2: actual outcome, for misprediction
                                  // detection and BHT update (meaningful only
                                  // when the resolved entry is a conditional
                                  // branch -- riscv64_ooo_proc.v only consults
                                  // it during a speculative window, which by
                                  // construction only exists for those)

    output req_valid,
    output [TAG_BITS-1:0] req_tag,
    output [63:0] req_value,
    input req_grant,

    // Phase 2: whether the currently-resident entry (if any) is a JALR --
    // used by riscv64_ooo_proc.v to keep JALR on the old Phase 1
    // stall-until-resolved path (target genuinely unknown until execute;
    // no BTB in this scope to predict it) while conditional branches and
    // JAL both get new, non-stalling behavior.
    output resident_is_jalr
);
    localparam BEQ  = 3'b000;
    localparam BNE  = 3'b001;
    localparam BLT  = 3'b100;
    localparam BGE  = 3'b101;
    localparam BLTU = 3'b110;
    localparam BGEU = 3'b111;

    reg busy;
    reg [2:0] func3;
    reg is_jal, is_jalr, is_branch;
    reg [63:0] r_pc, r_imm;
    reg s1_ready; reg [63:0] s1_val; reg [TAG_BITS-1:0] s1_tag;
    reg s2_ready; reg [63:0] s2_val; reg [TAG_BITS-1:0] s2_tag;
    reg [TAG_BITS-1:0] dest_tag;

    assign full = busy;

    wire needs_s1 = !is_jal;
    wire needs_s2 = is_branch;
    wire ready = busy && (!needs_s1 || s1_ready) && (!needs_s2 || s2_ready);
    wire has_result = is_jal || is_jalr;

    reg branch_taken;
    always @(*) begin
        case (func3)
            BEQ:  branch_taken = (s1_val == s2_val);
            BNE:  branch_taken = (s1_val != s2_val);
            BLT:  branch_taken = ($signed(s1_val) < $signed(s2_val));
            BGE:  branch_taken = ($signed(s1_val) >= $signed(s2_val));
            BLTU: branch_taken = (s1_val < s2_val);
            BGEU: branch_taken = (s1_val >= s2_val);
            default: branch_taken = 1'b0;
        endcase
    end

    wire [63:0] target = is_jal    ? (r_pc + r_imm) :
                          is_jalr  ? ((s1_val + r_imm) & ~64'b1) :
                          (is_branch && branch_taken) ? (r_pc + r_imm) :
                          (r_pc + 64'd4);

    assign req_valid = ready && has_result;
    assign req_tag   = dest_tag;
    assign req_value = r_pc + 64'd4; // return address -- always pc+4 (this
                                       // scoped core has no compressed
                                       // instructions, so no pc+2 case)

    wire vacate_now = ready && (!has_result || req_grant);
    assign resolved = vacate_now;
    assign resolved_next_pc = target;
    assign resolved_tag = dest_tag;
    assign resolved_has_result = has_result;
    assign resolved_pc = r_pc;
    assign resolved_taken = branch_taken;
    assign resident_is_jalr = busy && is_jalr;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            busy <= 1'b0;
        end else begin
            if (busy && !s1_ready && cdbA_valid && cdbA_tid == MY_TID && s1_tag == cdbA_tag) begin
                s1_ready <= 1'b1; s1_val <= cdbA_value;
            end else if (busy && !s1_ready && cdbB_valid && cdbB_tid == MY_TID && s1_tag == cdbB_tag) begin
                s1_ready <= 1'b1; s1_val <= cdbB_value;
            end
            if (busy && !s2_ready && cdbA_valid && cdbA_tid == MY_TID && s2_tag == cdbA_tag) begin
                s2_ready <= 1'b1; s2_val <= cdbA_value;
            end else if (busy && !s2_ready && cdbB_valid && cdbB_tid == MY_TID && s2_tag == cdbB_tag) begin
                s2_ready <= 1'b1; s2_val <= cdbB_value;
            end

            if (alloc_req && !busy) begin
                busy      <= 1'b1;
                func3     <= alloc_func3;
                is_jal    <= alloc_is_jal;
                is_jalr   <= alloc_is_jalr;
                is_branch <= alloc_is_branch;
                r_pc      <= alloc_pc;
                r_imm     <= alloc_imm;
                s1_ready  <= alloc_src1_ready;
                s1_val    <= alloc_src1_val;
                s1_tag    <= alloc_src1_tag;
                s2_ready  <= alloc_src2_ready;
                s2_val    <= alloc_src2_val;
                s2_tag    <= alloc_src2_tag;
                dest_tag  <= alloc_dest_tag;
            end

            if (vacate_now) begin
                busy <= 1'b0;
            end
        end
    end
endmodule
