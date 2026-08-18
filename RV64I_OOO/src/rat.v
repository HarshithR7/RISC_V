`timescale 1ns / 1ps
// Register Alias Table (Tomasulo's "register status"): maps each
// architectural register to either "value is in the real register file"
// (busy=0) or "will come from ROB tag T" (busy=1, tag=T). x0 is excluded
// entirely -- it's never renamed, matching register_file.v's own
// hardwired-zero handling, so an instruction with rd=x0 (used purely for
// side effects, e.g. `addi x0, x1, 5` in build_tests64.py's
// t_alu_rtype()) never creates a phantom producer anything could wait on.
//
// Dispatch ordering requirement on the CALLER, not enforced by this
// module: read rs1_tag/rs2_tag (this instruction's own operand sources)
// *before* asserting write_en for this same instruction's rd. Writing
// rd's new tag first would let an instruction like `addi x3, x3, 1`
// capture its own not-yet-computed tag as its own source operand -- a
// self-dependency that can never resolve. Since reads are combinational
// and the write only takes effect at the next clock edge, simply reading
// rs1_tag/rs2_tag and write_en/new_tag in the same dispatch cycle (before
// any registered update happens) is naturally safe either way; this
// comment states the requirement explicitly because it's the single
// easiest thing to get backwards in a first draft.
//
// Commit-vs-dispatch same-cycle interaction (real, not hypothetical --
// see tb_rob_rat.v's dedicated test): a WAW pair I1 (tag 5, rd=x3) then
// I2 (tag 8, rd=x3) is safe across *different* cycles by construction
// (commit_clear_en only fires if the RAT's *current* tag for that
// register still matches the committing tag -- see below). But if I1's
// commit and a brand-new, even-younger instruction J's dispatch-rename of
// the *same* register x3 land in the *same* cycle, both commit_clear_en
// and write_en target rat[x3] at once. write_en must win: J is younger
// than I1 in program order, so its rename is the correct final state.
// That's exactly what the statement order below produces (Verilog's
// last-non-blocking-assignment-wins rule for one always-block invocation
// resolves same-target conflicts in program order, not scheduling order,
// as long as they're written in that order) -- commit_clear_en's block
// is written first, write_en's second, so write_en's update to
// busy/tag for that register is what actually lands.
module rat #(
    parameter TAG_BITS = 3
)(
    input clk,
    input reset,

    input [4:0] rs1, rs2,
    output rs1_busy,
    output [TAG_BITS-1:0] rs1_tag,
    output rs2_busy,
    output [TAG_BITS-1:0] rs2_tag,

    // Phase 3: a second, independent read port for lane 1 (the younger
    // of a 2-wide dispatch group)'s own rs1/rs2, reading the exact same
    // underlying busy[]/tag[] this same cycle. This is the *raw* RAT
    // view only -- it does not know about lane 0's own rename, which is
    // happening this same cycle via write_en/rd/new_tag below and so
    // isn't reflected here yet (registered, takes effect next edge).
    // riscv64_ooo_proc.v's dispatch logic is responsible for the
    // intra-group bypass (lane 1 depending on lane 0's own destination
    // register) on top of this raw read -- see its header for why that
    // lives at the top level rather than inside this module.
    input [4:0] rs1b, rs2b,
    output rs1b_busy,
    output [TAG_BITS-1:0] rs1b_tag,
    output rs2b_busy,
    output [TAG_BITS-1:0] rs2b_tag,

    input write_en,          // skipped internally when rd == 0
    input [4:0] rd,
    input [TAG_BITS-1:0] new_tag,

    // Phase 3: lane 1's own rename, applied the same cycle as write_en
    // above but *after* it in program order (see the always block) --
    // last-non-blocking-assignment-wins means write2_en's update to a
    // shared target register is what actually lands, which is correct:
    // lane 1 is younger than lane 0, so if both write the same register
    // this cycle (an intra-group WAW), lane 1's rename must be the one
    // that sticks, exactly the same reasoning as the existing
    // commit_clear_en-vs-write_en ordering below.
    input write2_en,         // skipped internally when rd2 == 0
    input [4:0] rd2,
    input [TAG_BITS-1:0] new_tag2,

    input commit_clear_en,   // skipped internally when commit_rd == 0
    input [4:0] commit_rd,
    input [TAG_BITS-1:0] commit_tag,

    // Widened-commit support: a second, independent commit-clear for the
    // ROB's head+1 entry (see rob.v's dual-commit header). Same
    // conditional-clear semantics as commit_clear_en -- only fires if the
    // RAT's *current* tag for that register still matches commit_tag2.
    // No special ordering is needed relative to commit_clear_en even
    // though both can fire the same cycle: head and head+1 always carry
    // *different* ROB tags, so at most one of the two conditional clears
    // can ever actually match a given register's current tag -- there's
    // no real WAW race to resolve here, unlike write_en-vs-write2_en's
    // genuine same-cycle-rename race below.
    input commit_clear_en2,
    input [4:0] commit_rd2,
    input [TAG_BITS-1:0] commit_tag2,

    // Phase 2 speculation support: a single checkpoint (matching
    // branch_rs's own single-outstanding-branch scoping -- see
    // riscv64_ooo_proc.v's header). checkpoint_save snapshots live
    // busy[]/tag[] into a shadow copy the cycle a conditional branch
    // dispatches. From then until either a checkpoint_restore or the
    // next checkpoint_save, the shadow mirrors every commit_clear_en
    // event the live array also sees (every commit during a speculative
    // window is guaranteed older than the outstanding branch, since
    // commit is strictly in-order and the branch itself hasn't committed
    // yet -- so mirroring is always correct, never speculative). Without
    // this mirroring, a long speculative window with several older
    // commits in it would leave the checkpoint stale enough that, after
    // a restore, a dispatching consumer could see rs_busy=1 for an
    // already-committed register whose ROB slot has since been reused by
    // a wraparound-reallocated newer instruction -- silently wrong,
    // rather than just slow. checkpoint_restore then overwrites live
    // busy[]/tag[] from the (kept-current) shadow, discarding every
    // rename caused by the now-squashed speculative instructions.
    input checkpoint_save,
    input checkpoint_restore
);
    reg busy [1:31];
    reg [TAG_BITS-1:0] tag [1:31];
    reg busy_cp [1:31];
    reg [TAG_BITS-1:0] tag_cp [1:31];

    assign rs1_busy = (rs1 != 5'd0) && busy[rs1];
    assign rs1_tag  = (rs1 != 5'd0) ? tag[rs1] : {TAG_BITS{1'b0}};
    assign rs2_busy = (rs2 != 5'd0) && busy[rs2];
    assign rs2_tag  = (rs2 != 5'd0) ? tag[rs2] : {TAG_BITS{1'b0}};

    assign rs1b_busy = (rs1b != 5'd0) && busy[rs1b];
    assign rs1b_tag  = (rs1b != 5'd0) ? tag[rs1b] : {TAG_BITS{1'b0}};
    assign rs2b_busy = (rs2b != 5'd0) && busy[rs2b];
    assign rs2b_tag  = (rs2b != 5'd0) ? tag[rs2b] : {TAG_BITS{1'b0}};

    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 1; i < 32; i = i + 1) begin
                busy[i] <= 1'b0;
                busy_cp[i] <= 1'b0;
            end
        end else begin
            // Mirror this cycle's commit-clear(s) into the shadow first
            // (see header -- always safe, whether or not a checkpoint is
            // currently "active"; harmless no-op otherwise).
            if (commit_clear_en && commit_rd != 5'd0 && tag_cp[commit_rd] == commit_tag) begin
                busy_cp[commit_rd] <= 1'b0;
            end
            if (commit_clear_en2 && commit_rd2 != 5'd0 && tag_cp[commit_rd2] == commit_tag2) begin
                busy_cp[commit_rd2] <= 1'b0;
            end

            // Order matters -- see module header. Conditional clear
            // (only if no younger producer has since remapped this
            // register) first, then this cycle's new rename, so the
            // rename wins if both target the same register.
            if (commit_clear_en && commit_rd != 5'd0 && tag[commit_rd] == commit_tag) begin
                busy[commit_rd] <= 1'b0;
            end
            if (commit_clear_en2 && commit_rd2 != 5'd0 && tag[commit_rd2] == commit_tag2) begin
                busy[commit_rd2] <= 1'b0;
            end
            if (write_en && rd != 5'd0) begin
                busy[rd] <= 1'b1;
                tag[rd]  <= new_tag;
            end
            if (write2_en && rd2 != 5'd0) begin
                busy[rd2] <= 1'b1;
                tag[rd2]  <= new_tag2;
            end

            if (checkpoint_save) begin
                for (i = 1; i < 32; i = i + 1) begin
                    busy_cp[i] <= busy[i];
                    tag_cp[i]  <= tag[i];
                end
            end

            // Last, so it wins over any same-cycle write_en/commit_clear_en
            // above (last-non-blocking-assignment-wins, program order) --
            // a misprediction must produce a fully consistent rolled-back
            // RAT this cycle, taking priority over anything else touching
            // busy[]/tag[] the same cycle.
            if (checkpoint_restore) begin
                for (i = 1; i < 32; i = i + 1) begin
                    busy[i] <= busy_cp[i];
                    tag[i]  <= tag_cp[i];
                end
            end
        end
    end
endmodule
