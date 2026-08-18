`timescale 1ns / 1ps
// Multiply reservation-station bank + the (combinational, 1-cycle)
// multiplier itself -- same structure as alu_rs.v (lowest-free-index
// allocation, oldest-ROB-tag-first issue -- see alu_rs.v's header for
// why issue is age-ordered, not fixed-index), sized to 2 entries per the
// original Phase 1 sizing plan. Handles MUL/MULH/MULHSU/MULHU
// and MULW (the only *W multiply RISC-V defines; MULHW/MULHSUW/MULHUW
// don't exist). Logic lifted verbatim from RV64I/src/execute.v's
// 128-bit-multiply case statements. DIV/DIVU/REM/REMU (muldiv_op[2]==1)
// go to div_fu/div_rs instead -- decode_ooo.v's header documents this
// split (muldiv_op[2]==0 -> here, ==1 -> the divider).
module mul_rs #(
    parameter DEPTH = 2,
    parameter TAG_BITS = 3
)(
    input clk,
    input reset,

    input alloc_req,
    input [2:0] alloc_op,        // muldiv_op: 000=MUL 001=MULH 010=MULHSU 011=MULHU
    input alloc_word_op,
    input alloc_src1_ready,
    input [63:0] alloc_src1_val,
    input [TAG_BITS-1:0] alloc_src1_tag,
    input alloc_src2_ready,
    input [63:0] alloc_src2_val,
    input [TAG_BITS-1:0] alloc_src2_tag,
    input [TAG_BITS-1:0] alloc_dest_tag,
    output full,

    // Phase 3: lane-1 allocation port, same convention as alu_rs.v.
    input alloc2_req,
    input [2:0] alloc2_op,
    input alloc2_word_op,
    input alloc2_src1_ready,
    input [63:0] alloc2_src1_val,
    input [TAG_BITS-1:0] alloc2_src1_tag,
    input alloc2_src2_ready,
    input [63:0] alloc2_src2_val,
    input [TAG_BITS-1:0] alloc2_src2_tag,
    input [TAG_BITS-1:0] alloc2_dest_tag,
    output has_2_free,

    input cdb_valid,
    input [TAG_BITS-1:0] cdb_tag,
    input [63:0] cdb_value,

    output req_valid,
    output [TAG_BITS-1:0] req_tag,
    output [63:0] req_value,
    input req_grant,

    // Phase 2 misprediction squash -- same convention as alu_rs.v.
    input [TAG_BITS-1:0] rob_head_tag,
    input squash_valid,
    input [TAG_BITS-1:0] squash_tag
);
    localparam MUL    = 3'b000;
    localparam MULH   = 3'b001;
    localparam MULHSU = 3'b010;
    localparam MULHU  = 3'b011;

    reg busy       [0:DEPTH-1];
    reg [2:0] op    [0:DEPTH-1];
    reg word_op_arr[0:DEPTH-1];
    reg s1_ready   [0:DEPTH-1];
    reg [63:0] s1_val[0:DEPTH-1];
    reg [TAG_BITS-1:0] s1_tag[0:DEPTH-1];
    reg s2_ready   [0:DEPTH-1];
    reg [63:0] s2_val[0:DEPTH-1];
    reg [TAG_BITS-1:0] s2_tag[0:DEPTH-1];
    reg [TAG_BITS-1:0] dest_tag[0:DEPTH-1];

    // ---- Allocation: first free slot for lane 0, second-lowest (distinct
    // from lane 0's own pick) for lane 1 -- same pattern as alu_rs.v ----
    integer fi;
    reg [DEPTH-1:0] free_mask;
    reg have_free;
    reg [31:0] free_idx;
    reg have_free2;
    reg [31:0] free_idx2;
    always @(*) begin
        have_free = 1'b0;
        free_idx = 0;
        for (fi = 0; fi < DEPTH; fi = fi + 1)
            free_mask[fi] = !busy[fi];
        for (fi = DEPTH - 1; fi >= 0; fi = fi - 1)
            if (free_mask[fi]) begin
                have_free = 1'b1;
                free_idx = fi;
            end

        // Only reserve free_idx away from lane 1 if lane 0 is actually
        // going to consume it (alloc_req, not just have_free) -- see
        // alu_rs.v's identical fix for the deadlock this caused when
        // omitted.
        have_free2 = 1'b0;
        free_idx2 = 0;
        for (fi = DEPTH - 1; fi >= 0; fi = fi - 1)
            if (free_mask[fi] && !(alloc_req && have_free && fi == free_idx)) begin
                have_free2 = 1'b1;
                free_idx2 = fi;
            end
    end
    assign full = !have_free;
    assign has_2_free = have_free2;

    wire do_alloc1 = alloc_req  && have_free;
    wire do_alloc2 = alloc2_req && have_free2;

    // ---- Issue: oldest ready busy entry (age-ordered, see header) ----
    integer ri;
    reg have_ready;
    reg [31:0] ready_idx;
    reg [TAG_BITS-1:0] ready_best_age;
    always @(*) begin
        have_ready = 1'b0;
        ready_idx = 0;
        ready_best_age = 0;
        for (ri = 0; ri < DEPTH; ri = ri + 1)
            if (busy[ri] && s1_ready[ri] && s2_ready[ri] &&
                (!have_ready || age(dest_tag[ri]) < ready_best_age)) begin
                have_ready = 1'b1;
                ready_idx = ri;
                ready_best_age = age(dest_tag[ri]);
            end
    end

    reg signed [127:0] op1_s128, op1_z128, op2_s128, op2_z128;
    reg signed [127:0] mul_ss, mul_su, mul_uu;
    always @(*) begin
        op1_s128 = {{64{s1_val[ready_idx][63]}}, s1_val[ready_idx]};
        op1_z128 = {64'b0, s1_val[ready_idx]};
        op2_s128 = {{64{s2_val[ready_idx][63]}}, s2_val[ready_idx]};
        op2_z128 = {64'b0, s2_val[ready_idx]};
        mul_ss = op1_s128 * op2_s128;
        mul_su = op1_s128 * op2_z128;
        mul_uu = op1_z128 * op2_z128;
    end

    reg [63:0] mul_full_result, mul_word_result_ext;
    always @(*) begin
        case (op[ready_idx])
            MUL:     mul_full_result = mul_ss[63:0];
            MULH:    mul_full_result = mul_ss[127:64];
            MULHSU:  mul_full_result = mul_su[127:64];
            MULHU:   mul_full_result = mul_uu[127:64];
            default: mul_full_result = 64'b0;
        endcase
        // MULW: low 32 bits of a truncated product only depend on the low
        // 32 bits of the operands, so mul_ss[31:0] already equals the
        // 32-bit truncated product -- no separate multiply needed, same
        // reasoning as RV64I/src/execute.v's muldiv_word_result.
        mul_word_result_ext = {{32{mul_ss[31]}}, mul_ss[31:0]};
    end

    assign req_valid = have_ready;
    assign req_tag   = dest_tag[ready_idx];
    assign req_value = word_op_arr[ready_idx] ? mul_word_result_ext : mul_full_result;

    function [TAG_BITS-1:0] age;
        input [TAG_BITS-1:0] t;
        begin
            age = t - rob_head_tag;
        end
    endfunction

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (fi = 0; fi < DEPTH; fi = fi + 1)
                busy[fi] <= 1'b0;
        end else begin
            if (cdb_valid) begin
                for (fi = 0; fi < DEPTH; fi = fi + 1) begin
                    if (busy[fi] && !s1_ready[fi] && s1_tag[fi] == cdb_tag) begin
                        s1_ready[fi] <= 1'b1;
                        s1_val[fi]   <= cdb_value;
                    end
                    if (busy[fi] && !s2_ready[fi] && s2_tag[fi] == cdb_tag) begin
                        s2_ready[fi] <= 1'b1;
                        s2_val[fi]   <= cdb_value;
                    end
                end
            end

            if (do_alloc1) begin
                busy[free_idx]     <= 1'b1;
                op[free_idx]       <= alloc_op;
                word_op_arr[free_idx] <= alloc_word_op;
                s1_ready[free_idx] <= alloc_src1_ready;
                s1_val[free_idx]   <= alloc_src1_val;
                s1_tag[free_idx]   <= alloc_src1_tag;
                s2_ready[free_idx] <= alloc_src2_ready;
                s2_val[free_idx]   <= alloc_src2_val;
                s2_tag[free_idx]   <= alloc_src2_tag;
                dest_tag[free_idx] <= alloc_dest_tag;
            end
            if (do_alloc2) begin
                busy[free_idx2]     <= 1'b1;
                op[free_idx2]       <= alloc2_op;
                word_op_arr[free_idx2] <= alloc2_word_op;
                s1_ready[free_idx2] <= alloc2_src1_ready;
                s1_val[free_idx2]   <= alloc2_src1_val;
                s1_tag[free_idx2]   <= alloc2_src1_tag;
                s2_ready[free_idx2] <= alloc2_src2_ready;
                s2_val[free_idx2]   <= alloc2_src2_val;
                s2_tag[free_idx2]   <= alloc2_src2_tag;
                dest_tag[free_idx2] <= alloc2_dest_tag;
            end

            if (req_valid && req_grant) begin
                busy[ready_idx] <= 1'b0;
            end

            if (squash_valid) begin
                for (fi = 0; fi < DEPTH; fi = fi + 1)
                    if (busy[fi] && age(dest_tag[fi]) > age(squash_tag))
                        busy[fi] <= 1'b0;
            end
        end
    end
endmodule
