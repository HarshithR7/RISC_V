`timescale 1ns / 1ps

// Synthesis-only variant of riscv64_proc.v: fpu.v/fp_register_file.v are
// removed and their outputs tied to constants. fpu.v's F/D arithmetic is
// built on Verilog's `real` type ($bitstoreal/$realtobits) -- a
// simulation-only construct with no synthesizable hardware mapping in any
// real tool (Yosys included) -- so a from-scratch synthesizable FPU would
// be a separate, substantial undertaking, not something this file
// attempts. This file exists purely to get real area/frequency numbers
// for the synthesizable RV64IMAC + RVV integer/vector datapath; it is not
// part of the verified, simulated core (that's src/riscv64_proc.v,
// unchanged) and must never be used for simulation or correctness
// verification -- it silently produces wrong results for any F/D
// instruction (fp_write_back_data/fpu_iresult always read as 0).
module riscv64_processor_synth #(
    parameter IMEM_FILE  = "instructions.mem",
    parameter DMEM_FILE  = "data.mem",
    parameter IMEM_WORDS = 8192,  // halfwords (RV64C: instructions are 2-byte aligned)
    parameter DMEM_WORDS = 4096
)(
    input clk,
    input reset,
    output wire [63:0] alu_result,
    output wire [63:0] output_mem_read,
    output wire [63:0] pc_out,
    output wire ecall_halt
);
    wire [63:0] pc, next_pc;
    wire [31:0] raw_fetch32;
    wire [31:0] instruction;    // always the expanded/real 32-bit instruction
    wire is_compressed;
    wire [63:0] read_data1, read_data2;
    wire [63:0] write_back_data;
    wire [63:0] imm;
    wire [4:0] rs1, rs2, rd;
    wire reg_write, mem_to_reg;

    wire alu_src, mem_read, mem_write, word_op;
    wire [3:0] alu_op;
    wire [2:0] func3;
    wire [6:0] func7;
    wire branch_taken;
    wire is_muldiv;
    wire [2:0] muldiv_op;
    wire is_amo, is_lr, is_sc;
    wire [4:0] amo_funct5;

    wire is_fp_op, rs1_is_fp, rs2_is_fp, rd_is_fp, fp_mem_op, word_int_src, fp_reg_write, is_double;
    wire [4:0] fp_op, rs3;
    wire [63:0] fp_read_data1, fp_read_data2, fp_read_data3;
    wire [63:0] fp_write_back_data;
    wire [63:0] fpu_fresult;
    wire [63:0] fpu_iresult;

    wire is_vector_op, is_vsetvli, vreg_write, is_vle, is_vse;
    wire is_vcompare, is_vreduce, v_vm;
    wire [4:0] v_op;
    wire [1:0] v_form;
    wire [4:0] v_imm;
    wire [127:0] vreg_read_data1, vreg_read_data2, v0_data, v_write_back_data, valu_result;
    wire [3:0] valu_cmp_bits;
    wire [127:0] vmem_read_data;

    wire [63:0] mem_addr;

    assign pc_out = pc;
    assign ecall_halt = (instruction[6:0] == 7'b1110011); // ECALL/EBREAK (SYSTEM opcode)

    program_counter pc_module (
        .clk(clk), .reset(reset),
        .pc_in(next_pc), .pc_out(pc)
    );

    instruction_fetch #(
        .IMEM_FILE(IMEM_FILE), .IMEM_WORDS(IMEM_WORDS)
    ) if_module (
        .clk(clk), .pc(pc), .instruction(raw_fetch32)
    );

    compressed_decoder cdec_module (
        .fetch32(raw_fetch32),
        .instruction(instruction),
        .is_compressed(is_compressed)
    );

    decode_control_unit id_cu_module (
        .instruction(instruction),
        .rs1(rs1), .rs2(rs2), .rd(rd), .imm(imm),
        .alu_src(alu_src), .reg_write(reg_write),
        .mem_read(mem_read), .mem_write(mem_write), .mem_to_reg(mem_to_reg),
        .alu_op(alu_op), .word_op(word_op),
        .is_muldiv(is_muldiv), .muldiv_op(muldiv_op),
        .is_amo(is_amo), .is_lr(is_lr), .is_sc(is_sc), .amo_funct5(amo_funct5),
        .is_fp_op(is_fp_op), .fp_op(fp_op),
        .rs1_is_fp(rs1_is_fp), .rs2_is_fp(rs2_is_fp), .rd_is_fp(rd_is_fp), .rs3(rs3),
        .fp_mem_op(fp_mem_op), .word_int_src(word_int_src), .fp_reg_write(fp_reg_write),
        .is_double(is_double),
        .is_vector_op(is_vector_op), .is_vsetvli(is_vsetvli),
        .v_op(v_op), .v_form(v_form), .v_imm(v_imm), .vreg_write(vreg_write),
        .is_vle(is_vle), .is_vse(is_vse),
        .v_vm(v_vm), .is_vcompare(is_vcompare), .is_vreduce(is_vreduce),
        .func3(func3), .func7(func7)
    );

    // fp_register_file/fpu intentionally not instantiated here -- see the
    // file header. Tied to constants so the rest of the module (which
    // still references these wires in its write-back muxes) elaborates
    // and synthesizes; F/D instructions are not functionally correct in
    // this variant.
    assign fp_read_data1 = 64'b0;
    assign fp_read_data2 = 64'b0;
    assign fp_read_data3 = 64'b0;
    assign fpu_fresult = 64'b0;
    assign fpu_iresult = 64'b0;

    register_file regfile (
        .clk(clk), .reset(reset),
        .reg_write(reg_write),
        .read_reg1(rs1), .read_reg2(rs2), .write_reg(rd),
        .write_data(write_back_data),
        .read_data1(read_data1), .read_data2(read_data2)
    );

    // ---- RVV (scoped: VLEN=128, SEW=32, LMUL=1 only) ------------------
    // vector_register_file's "read_reg1" doubles as vs2 (arithmetic) and
    // vs3 (vse32.v's store-data register, which per spec occupies the
    // same instruction field position vd/rd would for a load) -- these
    // are mutually exclusive control paths, so the mux is safe.
    wire [4:0] vreg_addr1 = is_vse ? rd : rs2;
    vector_register_file vregfile (
        .clk(clk), .reset(reset),
        .vreg_write(vreg_write),
        .read_reg1(vreg_addr1), .read_reg2(rs1), .write_reg(rd),
        .write_data(v_write_back_data),
        .read_data1(vreg_read_data1), .read_data2(vreg_read_data2),
        .v0_data(v0_data)
    );

    // operand2 for the vector ALU: vs1's vector data (.vv), rs1's scalar
    // integer value broadcast into all 4 lanes (.vx), or the instruction's
    // 5-bit immediate, sign-extended then broadcast (.vi).
    wire [31:0] v_imm_sext = {{27{v_imm[4]}}, v_imm};
    reg [127:0] v_operand2;
    always @(*) begin
        case (v_form)
            2'd0: v_operand2 = vreg_read_data2;                          // .vv
            2'd1: v_operand2 = {4{read_data1[31:0]}};                    // .vx
            2'd2: v_operand2 = {4{v_imm_sext}};                          // .vi
            default: v_operand2 = 128'b0;
        endcase
    end

    vector_alu valu_module (
        .vs2_data(vreg_read_data1), .operand2(v_operand2), .v_op(v_op),
        .vd_result(valu_result), .cmp_bits(valu_cmp_bits)
    );

    // vl: the only piece of "CSR-like" state this core has. vsetvli sets
    // it to min(AVL, VLMAX=4), or VLMAX itself when rs1=x0 (the spec's
    // "set vl to the maximum" idiom, which reading x0 as a value of 0
    // would otherwise misinterpret as "set vl to 0").
    reg [2:0] vl_reg;
    wire vsetvli_use_vlmax = (rs1 == 5'd0);
    wire [2:0] vsetvli_new_vl = vsetvli_use_vlmax ? 3'd4 :
                                 (read_data1 > 64'd4) ? 3'd4 : read_data1[2:0];
    always @(posedge clk or posedge reset) begin
        if (reset) vl_reg <= 3'd4;
        else if (is_vsetvli) vl_reg <= vsetvli_new_vl;
    end

    // vle32.v: zero-fill (tail-agnostic) any lane at or beyond vl.
    wire [127:0] vle_lane_mask = {{32{vl_reg > 3}}, {32{vl_reg > 2}}, {32{vl_reg > 1}}, {32{vl_reg > 0}}};
    wire [127:0] vle_result = vmem_read_data & vle_lane_mask;

    // vse32.v: only lanes below vl are new data; lanes at/beyond vl keep
    // whatever is already in memory there (read-modify-write via the same
    // always-on vmem_read_data port, since data_memory's write commits the
    // full 128 bits every time).
    wire [127:0] vse_write_data = (vreg_read_data1 & vle_lane_mask) | (vmem_read_data & ~vle_lane_mask);

    // ---- RVV Tier 2: masking (v0.t), compares, reductions ------------
    // A lane is "active" for an arithmetic/compare instruction when it's
    // below vl AND (unmasked, or v0's corresponding bit is set). Inactive
    // lanes are mask-agnostic here (forced to 0) -- spec-legal, and the
    // same simplification already used for vle32.v/vse32.v's tail
    // handling above (avoiding a 3rd vector-register read port that a
    // mask-undisturbed policy would need for vd's old value).
    wire [3:0] tail_mask = {vl_reg > 3, vl_reg > 2, vl_reg > 1, vl_reg > 0};
    wire [3:0] full_lane_mask = v_vm ? tail_mask : (tail_mask & v0_data[3:0]);
    wire [127:0] lane_mask_128 = {{32{full_lane_mask[3]}}, {32{full_lane_mask[2]}},
                                   {32{full_lane_mask[1]}}, {32{full_lane_mask[0]}}};
    wire [127:0] varith_result = valu_result & lane_mask_128;
    // Compare results are 1 bit/lane, packed into vd's low 4 bits (a mask
    // register); the rest of vd is agnostic (zeroed).
    wire [127:0] vcmp_result = {124'b0, (valu_cmp_bits & full_lane_mask)};

    // Reductions (vredsum/etc): fold vs2's active lanes (vreg_read_data1)
    // into a scalar, seeded by vs1's element 0 (vreg_read_data2[31:0]),
    // using the same op semantics vector_alu's per-lane path uses --
    // implemented directly here since a fold across lanes is a genuinely
    // different execution shape than vector_alu's parallel-lanes model.
    reg [31:0] vreduce_acc;
    integer r_i;
    always @(*) begin
        vreduce_acc = vreg_read_data2[31:0];
        for (r_i = 0; r_i < 4; r_i = r_i + 1) begin
            if (full_lane_mask[r_i]) begin
                case (v_op)
                    4'd0:  vreduce_acc = vreduce_acc + vreg_read_data1[r_i*32 +: 32];
                    4'd2:  vreduce_acc = vreduce_acc & vreg_read_data1[r_i*32 +: 32];
                    4'd3:  vreduce_acc = vreduce_acc | vreg_read_data1[r_i*32 +: 32];
                    4'd4:  vreduce_acc = vreduce_acc ^ vreg_read_data1[r_i*32 +: 32];
                    4'd12: vreduce_acc = ($signed(vreduce_acc) < $signed(vreg_read_data1[r_i*32 +: 32])) ? vreduce_acc : vreg_read_data1[r_i*32 +: 32];
                    4'd13: vreduce_acc = (vreduce_acc < vreg_read_data1[r_i*32 +: 32]) ? vreduce_acc : vreg_read_data1[r_i*32 +: 32];
                    4'd14: vreduce_acc = ($signed(vreduce_acc) > $signed(vreg_read_data1[r_i*32 +: 32])) ? vreduce_acc : vreg_read_data1[r_i*32 +: 32];
                    4'd15: vreduce_acc = (vreduce_acc > vreg_read_data1[r_i*32 +: 32]) ? vreduce_acc : vreg_read_data1[r_i*32 +: 32];
                    default: ;
                endcase
            end
        end
    end

    execute ex_module (
        .opcode(instruction[6:0]), .func3(func3),
        .rs1_data(read_data1), .rs2_data(read_data2),
        .imm(imm), .pc(pc), .rd(rd),
        .alu_src(alu_src), .alu_op(alu_op), .word_op(word_op),
        .is_muldiv(is_muldiv), .muldiv_op(muldiv_op),
        .is_compressed(is_compressed),
        .alu_result(alu_result), .mem_addr(mem_addr),
        .next_pc(next_pc), .branch_taken(branch_taken)
    );

    // ---- A extension: LR/SC reservation ------------------------------
    // Single-hart, single-cycle core: only one instruction is ever "in
    // flight", so ordinary AMO read-modify-write is atomic by construction
    // (see the amo_write_data block below). LR/SC still need real state,
    // though, since SC's success depends on whether *any* store happened
    // between the LR and this SC. Invalidate conservatively on any store
    // (the spec permits invalidating more often than strictly required).
    reg reservation_valid;
    reg [63:0] reservation_addr;
    wire sc_success = is_sc && reservation_valid && (reservation_addr == mem_addr);
    wire dmem_write_enable = mem_write || (is_sc && sc_success);

    always @(posedge clk or posedge reset) begin
        if (reset)
            reservation_valid <= 1'b0;
        else if (is_lr) begin
            reservation_valid <= 1'b1;
            reservation_addr  <= mem_addr;
        end
        else if (is_sc || mem_write)
            reservation_valid <= 1'b0; // SC always drops the reservation, per spec; so does any other store
    end

    // ---- A extension: AMO read-modify-write --------------------------
    // output_mem_read (from data_memory below) already holds the correctly
    // sign/zero-width-extended *old* value for this address (data_memory's
    // existing func3-driven LW/LD read path -- AMO's func3 is 010/011,
    // identical encoding to LW/LD). Only MIN/MAX-family comparisons need
    // an explicit 32-bit-vs-64-bit reinterpretation; ADD/XOR/AND/OR/SWAP
    // are invariant in their low 32 bits regardless, since data_memory's
    // write path only ever commits write_data[31:0] for a .W access.
    localparam AMO_FUNCT5_SWAP  = 5'b00001;
    localparam AMO_FUNCT5_ADD   = 5'b00000;
    localparam AMO_FUNCT5_XOR   = 5'b00100;
    localparam AMO_FUNCT5_AND   = 5'b01100;
    localparam AMO_FUNCT5_OR    = 5'b01000;
    localparam AMO_FUNCT5_MIN   = 5'b10000;
    localparam AMO_FUNCT5_MAX   = 5'b10100;
    localparam AMO_FUNCT5_MINU  = 5'b11000;
    localparam AMO_FUNCT5_MAXU  = 5'b11100;

    wire is_word_amo = (func3 == 3'b010);
    wire [63:0] amo_old_s = is_word_amo ? {{32{output_mem_read[31]}}, output_mem_read[31:0]} : output_mem_read;
    wire [63:0] amo_old_u = is_word_amo ? {32'b0, output_mem_read[31:0]} : output_mem_read;
    wire [63:0] amo_rs2_s = is_word_amo ? {{32{read_data2[31]}}, read_data2[31:0]} : read_data2;
    wire [63:0] amo_rs2_u = is_word_amo ? {32'b0, read_data2[31:0]} : read_data2;

    reg [63:0] amo_write_data;
    always @(*) begin
        case (amo_funct5)
            AMO_FUNCT5_SWAP: amo_write_data = read_data2;
            AMO_FUNCT5_ADD:  amo_write_data = output_mem_read + read_data2;
            AMO_FUNCT5_XOR:  amo_write_data = output_mem_read ^ read_data2;
            AMO_FUNCT5_AND:  amo_write_data = output_mem_read & read_data2;
            AMO_FUNCT5_OR:   amo_write_data = output_mem_read | read_data2;
            AMO_FUNCT5_MIN:  amo_write_data = ($signed(amo_old_s) < $signed(amo_rs2_s)) ? output_mem_read : read_data2;
            AMO_FUNCT5_MAX:  amo_write_data = ($signed(amo_old_s) > $signed(amo_rs2_s)) ? output_mem_read : read_data2;
            AMO_FUNCT5_MINU: amo_write_data = (amo_old_u < amo_rs2_u) ? output_mem_read : read_data2;
            AMO_FUNCT5_MAXU: amo_write_data = (amo_old_u > amo_rs2_u) ? output_mem_read : read_data2;
            default:         amo_write_data = read_data2;
        endcase
    end

    // FSD stores the full 64-bit double; FSW only the low 32 bits (upper
    // bits of dmem_write_data are irrelevant there since data_memory's
    // own SW-style write only ever commits write_data[31:0]).
    wire [63:0] dmem_write_data = is_amo ? amo_write_data :
                                   fp_mem_op ? (is_double ? fp_read_data2 : {32'b0, fp_read_data2[31:0]}) :
                                   read_data2;

    // FLW must zero-extend its 32-bit read (it's a bit pattern, not a
    // signed integer) -- data_memory's own func3=010 path sign-extends
    // (that's LW's normal behavior), so override to 110 (LWU's
    // zero-extending word read) specifically for that case. FLD needs no
    // such override: a 64-bit read has no sign/zero-extension question at
    // all. FSW/FSD keep their own func3 (010/011) unmodified: extension
    // is irrelevant for a write either way.
    wire [2:0] dmem_func3 = (fp_mem_op && mem_read && !is_double) ? 3'b110 : func3;

    data_memory #(
        .DMEM_FILE(DMEM_FILE), .DMEM_WORDS(DMEM_WORDS)
    ) dmem (
        .clk(clk), .mem_read(mem_read), .mem_write(dmem_write_enable),
        .func3(dmem_func3), .mem_addr(mem_addr),
        .write_data(dmem_write_data), .read_data(output_mem_read),
        .vmem_read(is_vle), .vmem_write(is_vse), .vmem_addr(mem_addr),
        .vmem_write_data(vse_write_data), .vmem_read_data(vmem_read_data)
    );

    // Integer register write-back mux. SC synthesizes its own result
    // (0=success, 1=failure); FP compares/classify/move-to-int/convert-
    // to-int write an integer register too (rd_is_fp=0 while is_fp_op=1);
    // vsetvli writes the newly-configured vl.
    assign write_back_data = is_sc ? (sc_success ? 64'd0 : 64'd1) :
                              is_vsetvli ? {61'b0, vsetvli_new_vl} :
                              (is_fp_op && !rd_is_fp) ? fpu_iresult :
                              mem_to_reg ? output_mem_read : alu_result;

    // FP register write-back mux. `fpu_fresult` already comes out of
    // fpu.v correctly formatted (NaN-boxed float32, or a full float64) --
    // no re-boxing needed here. FLW/FLD need the same distinction applied
    // to the raw memory read instead.
    assign fp_write_back_data = fp_mem_op ? (is_double ? output_mem_read : {32'hFFFFFFFF, output_mem_read[31:0]}) :
                                 fpu_fresult;

    // Vector register write-back: vle32.v's (tail-zeroed) load result, a
    // folded reduction scalar (element 0 only), a packed compare mask, or
    // the vector ALU's masked/tail-zeroed elementwise result.
    assign v_write_back_data = is_vle ? vle_result :
                                is_vreduce ? {96'b0, vreduce_acc} :
                                is_vcompare ? vcmp_result :
                                varith_result;

endmodule
