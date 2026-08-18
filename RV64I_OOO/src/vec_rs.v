`timescale 1ns / 1ps
// Vector reservation-station bank + the (combinational) vector ALU
// itself -- wraps RV64I/src/vector_alu.v's actual elementwise datapath
// unchanged, the same "reuse the real execution logic, don't re-derive
// it" instinct as alu_rs.v reusing RV64I/src/execute.v's ALU case
// statements. Single-entry-class allocation like branch_rs.v (only ever
// fed from lane 0, single-issue for vector instructions -- see
// riscv64_ooo_proc.v's header for the Phase 6 scoping this reflects),
// sized to DEPTH entries with the same fixed-lowest-free-index
// allocation / oldest-ROB-tag-first issue pattern as alu_rs.v/mul_rs.v.
//
// Broadcasts on a dedicated vec_mark port on rob.v, not the scalar CDB:
// vector operands in this scoped core only ever come from other vector
// instructions (no .vx/.vi forms, which would read a scalar register as
// an operand -- out of scope here), so there's no cross-domain
// dependency needing unified arbitration with the scalar banks. With
// only one vector-producing bank in this scope, there's also nothing
// else to arbitrate against -- req_grant is simply tied to req_valid at
// the top level.
module vec_rs #(
    parameter DEPTH = 2,
    parameter TAG_BITS = 3,
    parameter VLEN = 128
)(
    input clk,
    input reset,

    input alloc_req,
    input [4:0] alloc_op,
    input alloc_src1_ready,
    input [VLEN-1:0] alloc_src1_val,
    input [TAG_BITS-1:0] alloc_src1_tag,
    input alloc_src2_ready,
    input [VLEN-1:0] alloc_src2_val,
    input [TAG_BITS-1:0] alloc_src2_tag,
    input [TAG_BITS-1:0] alloc_dest_tag,
    output full,

    // Snoop: an operand can only be produced by another (older) vector
    // instruction's own vec_mark broadcast -- see module header.
    input vec_cdb_valid,
    input [TAG_BITS-1:0] vec_cdb_tag,
    input [VLEN-1:0] vec_cdb_value,

    output req_valid,
    output [TAG_BITS-1:0] req_tag,
    output [VLEN-1:0] req_value,
    input req_grant,

    input [TAG_BITS-1:0] rob_head_tag,
    input squash_valid,
    input [TAG_BITS-1:0] squash_tag
);
    reg busy       [0:DEPTH-1];
    reg [4:0] op    [0:DEPTH-1];
    reg s1_ready   [0:DEPTH-1];
    reg [VLEN-1:0] s1_val[0:DEPTH-1];
    reg [TAG_BITS-1:0] s1_tag[0:DEPTH-1];
    reg s2_ready   [0:DEPTH-1];
    reg [VLEN-1:0] s2_val[0:DEPTH-1];
    reg [TAG_BITS-1:0] s2_tag[0:DEPTH-1];
    reg [TAG_BITS-1:0] dest_tag[0:DEPTH-1];

    integer fi;
    reg [DEPTH-1:0] free_mask;
    reg have_free;
    reg [31:0] free_idx;
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
    end
    assign full = !have_free;

    function [TAG_BITS-1:0] age;
        input [TAG_BITS-1:0] t;
        begin
            age = t - rob_head_tag;
        end
    endfunction

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

    wire [VLEN-1:0] valu_result;
    wire [VLEN/32-1:0] valu_cmp_bits; // unused (no compares in this scoped integration)
    vector_alu #(.LANES(VLEN/32), .VLEN(VLEN)) valu_i (
        .vs2_data(s1_val[ready_idx]),  // caller wires this to vs2's read value
        .operand2(s2_val[ready_idx]),  // caller wires this to vs1's read value (.vv form)
        .v_op(op[ready_idx]),
        .vd_result(valu_result),
        .cmp_bits(valu_cmp_bits)
    );

    assign req_valid = have_ready;
    assign req_tag   = dest_tag[ready_idx];
    assign req_value = valu_result;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (fi = 0; fi < DEPTH; fi = fi + 1)
                busy[fi] <= 1'b0;
        end else begin
            if (vec_cdb_valid) begin
                for (fi = 0; fi < DEPTH; fi = fi + 1) begin
                    if (busy[fi] && !s1_ready[fi] && s1_tag[fi] == vec_cdb_tag) begin
                        s1_ready[fi] <= 1'b1;
                        s1_val[fi]   <= vec_cdb_value;
                    end
                    if (busy[fi] && !s2_ready[fi] && s2_tag[fi] == vec_cdb_tag) begin
                        s2_ready[fi] <= 1'b1;
                        s2_val[fi]   <= vec_cdb_value;
                    end
                end
            end

            if (alloc_req && have_free) begin
                busy[free_idx]     <= 1'b1;
                op[free_idx]       <= alloc_op;
                s1_ready[free_idx] <= alloc_src1_ready;
                s1_val[free_idx]   <= alloc_src1_val;
                s1_tag[free_idx]   <= alloc_src1_tag;
                s2_ready[free_idx] <= alloc_src2_ready;
                s2_val[free_idx]   <= alloc_src2_val;
                s2_tag[free_idx]   <= alloc_src2_tag;
                dest_tag[free_idx] <= alloc_dest_tag;
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
