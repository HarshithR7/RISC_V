`timescale 1ns / 1ps

module execute (
    input wire [6:0] opcode,
    input wire [2:0] func3,
    input wire [63:0] rs1_data,
    input wire [63:0] rs2_data,
    input wire [63:0] imm,
    input wire [63:0] pc,
    input wire [4:0] rd,
    input wire alu_src,
    input wire [3:0] alu_op,
    input wire word_op,           // *W / *IW: compute on lower 32 bits, sign-extend result
    input wire is_muldiv,         // M extension: MUL/MULH*/DIV*/REM* (and *W forms)
    input wire [2:0] muldiv_op,
    input wire is_compressed,     // C extension: this instruction was 16 bits, not 32
    output reg [63:0] alu_result,
    output reg [63:0] mem_addr,
    output reg [63:0] next_pc,
    output reg branch_taken,
    output reg [4:0] rd_out
);
    localparam LOAD     = 7'b0000011;
    localparam S_TYPE   = 7'b0100011;
    localparam AMO      = 7'b0101111;
    localparam LOAD_FP  = 7'b0000111;
    localparam STORE_FP = 7'b0100111;
    localparam B_TYPE   = 7'b1100011;
    localparam JAL      = 7'b1101111;
    localparam JALR     = 7'b1100111;
    localparam LUI_TYPE = 7'b0110111;
    localparam AUIPC    = 7'b0010111;

    localparam ALU_ADD  = 4'b0010;
    localparam ALU_SUB  = 4'b1010;
    localparam ALU_AND  = 4'b0100;
    localparam ALU_OR   = 4'b0101;
    localparam ALU_XOR  = 4'b0011;
    localparam ALU_SLL  = 4'b0110;
    localparam ALU_SRL  = 4'b0111;
    localparam ALU_SRA  = 4'b1000;
    localparam ALU_SLT  = 4'b1011;
    localparam ALU_SLTU = 4'b1100;

    localparam BEQ  = 3'b000;
    localparam BNE  = 3'b001;
    localparam BLT  = 3'b100;
    localparam BGE  = 3'b101;
    localparam BLTU = 3'b110;
    localparam BGEU = 3'b111;

    localparam MULDIV_MUL    = 3'b000;
    localparam MULDIV_MULH   = 3'b001;
    localparam MULDIV_MULHSU = 3'b010;
    localparam MULDIV_MULHU  = 3'b011;
    localparam MULDIV_DIV    = 3'b100;
    localparam MULDIV_DIVU   = 3'b101;
    localparam MULDIV_REM    = 3'b110;
    localparam MULDIV_REMU   = 3'b111;

    localparam [63:0] MIN_INT64  = 64'h8000000000000000;
    localparam [63:0] ALLONES64  = 64'hFFFFFFFFFFFFFFFF;
    localparam [31:0] MIN_INT32  = 32'h80000000;
    localparam [31:0] ALLONES32  = 32'hFFFFFFFF;

    reg [63:0] operand_1, operand_2;
    reg [63:0] full_result;
    reg [31:0] word_result;

    // ---- M extension: multiply --------------------------------------
    // Sign/zero-extend to 128 bits per the desired interpretation, then a
    // single signed 128x128 multiply gives the right bit pattern for all
    // three signedness combinations (MUL/MULH, MULHSU, MULHU).
    reg signed [127:0] op1_s128, op1_z128, op2_s128, op2_z128;
    reg signed [127:0] mul_ss, mul_su, mul_uu;
    always @(*) begin
        op1_s128 = {{64{operand_1[63]}}, operand_1};
        op1_z128 = {64'b0, operand_1};
        op2_s128 = {{64{operand_2[63]}}, operand_2};
        op2_z128 = {64'b0, operand_2};
        mul_ss = op1_s128 * op2_s128; // signed x signed   (MUL, MULH)
        mul_su = op1_s128 * op2_z128; // signed x unsigned (MULHSU)
        mul_uu = op1_z128 * op2_z128; // unsigned x unsigned (MULHU)
    end

    // ---- M extension: divide/remainder, full 64-bit width ------------
    // Division-by-zero and signed-overflow (MIN_INT / -1) results are
    // architecturally defined (not exceptions) per the RISC-V spec.
    reg [63:0] div_result, divu_result, rem_result, remu_result;
    always @(*) begin
        if (operand_2 == 64'b0)
            div_result = ALLONES64;
        else if (operand_1 == MIN_INT64 && operand_2 == ALLONES64)
            div_result = MIN_INT64;
        else
            div_result = $signed(operand_1) / $signed(operand_2);

        divu_result = (operand_2 == 64'b0) ? ALLONES64 : (operand_1 / operand_2);

        if (operand_2 == 64'b0)
            rem_result = operand_1;
        else if (operand_1 == MIN_INT64 && operand_2 == ALLONES64)
            rem_result = 64'b0;
        else
            rem_result = $signed(operand_1) % $signed(operand_2);

        remu_result = (operand_2 == 64'b0) ? operand_1 : (operand_1 % operand_2);
    end

    // ---- M extension: divide/remainder, 32-bit ("W") width -----------
    wire [31:0] op1_w = operand_1[31:0];
    wire [31:0] op2_w = operand_2[31:0];
    reg [31:0] divw_result, divuw_result, remw_result, remuw_result;
    always @(*) begin
        if (op2_w == 32'b0)
            divw_result = ALLONES32;
        else if (op1_w == MIN_INT32 && op2_w == ALLONES32)
            divw_result = MIN_INT32;
        else
            divw_result = $signed(op1_w) / $signed(op2_w);

        divuw_result = (op2_w == 32'b0) ? ALLONES32 : (op1_w / op2_w);

        if (op2_w == 32'b0)
            remw_result = op1_w;
        else if (op1_w == MIN_INT32 && op2_w == ALLONES32)
            remw_result = 32'b0;
        else
            remw_result = $signed(op1_w) % $signed(op2_w);

        remuw_result = (op2_w == 32'b0) ? op1_w : (op1_w % op2_w);
    end

    reg [63:0] muldiv_full_result;
    reg [31:0] muldiv_word_result;
    always @(*) begin
        case (muldiv_op)
            MULDIV_MUL:    muldiv_full_result = mul_ss[63:0];
            MULDIV_MULH:   muldiv_full_result = mul_ss[127:64];
            MULDIV_MULHSU: muldiv_full_result = mul_su[127:64];
            MULDIV_MULHU:  muldiv_full_result = mul_uu[127:64];
            MULDIV_DIV:    muldiv_full_result = div_result;
            MULDIV_DIVU:   muldiv_full_result = divu_result;
            MULDIV_REM:    muldiv_full_result = rem_result;
            MULDIV_REMU:   muldiv_full_result = remu_result;
            default:       muldiv_full_result = 64'b0;
        endcase
        case (muldiv_op)
            // The low 32 bits of a truncated product only depend on the low
            // 32 bits of the operands, so mul_ss[31:0] already equals
            // (op1_w * op2_w) truncated to 32 bits -- no separate multiply
            // needed for MULW.
            MULDIV_MUL:  muldiv_word_result = mul_ss[31:0];
            MULDIV_DIV:  muldiv_word_result = divw_result;
            MULDIV_DIVU: muldiv_word_result = divuw_result;
            MULDIV_REM:  muldiv_word_result = remw_result;
            MULDIV_REMU: muldiv_word_result = remuw_result;
            default:     muldiv_word_result = 32'b0;
        endcase
    end

    always @(*) begin
        operand_1 = (opcode == LUI_TYPE) ? 64'b0 :
                    (opcode == AUIPC)    ? pc    : rs1_data;
        operand_2 = alu_src ? imm : rs2_data;

        branch_taken = 1'b0;
        next_pc      = pc + (is_compressed ? 64'd2 : 64'd4);
        rd_out       = rd;
        mem_addr     = 64'b0;
        alu_result   = 64'b0;

        case (opcode)
            LOAD, S_TYPE, AMO, LOAD_FP, STORE_FP: mem_addr = rs1_data + imm; // imm is 0 for AMO -- address is rs1 alone

            B_TYPE: begin
                case (func3)
                    BEQ:  branch_taken = (rs1_data == rs2_data);
                    BNE:  branch_taken = (rs1_data != rs2_data);
                    BLT:  branch_taken = ($signed(rs1_data) < $signed(rs2_data));
                    BGE:  branch_taken = ($signed(rs1_data) >= $signed(rs2_data));
                    BLTU: branch_taken = (rs1_data < rs2_data);
                    BGEU: branch_taken = (rs1_data >= rs2_data);
                    default: branch_taken = 1'b0;
                endcase
                if (branch_taken) next_pc = pc + imm;
            end

            JAL:  next_pc = pc + imm;
            JALR: next_pc = (rs1_data + imm) & ~64'b1;

            default: ; // R_TYPE / I_TYPE / OP_32 / OP_IMM_32 / LUI / AUIPC
        endcase

        // 64-bit ALU result
        case (alu_op)
            ALU_ADD:  full_result = operand_1 + operand_2;
            ALU_SUB:  full_result = operand_1 - operand_2;
            ALU_AND:  full_result = operand_1 & operand_2;
            ALU_OR:   full_result = operand_1 | operand_2;
            ALU_XOR:  full_result = operand_1 ^ operand_2;
            ALU_SLL:  full_result = operand_1 << operand_2[5:0];
            ALU_SRL:  full_result = operand_1 >> operand_2[5:0];
            ALU_SRA:  full_result = $signed(operand_1) >>> operand_2[5:0];
            ALU_SLT:  full_result = ($signed(operand_1) < $signed(operand_2)) ? 64'd1 : 64'd0;
            ALU_SLTU: full_result = (operand_1 < operand_2) ? 64'd1 : 64'd0;
            default:  full_result = 64'b0;
        endcase

        // 32-bit ("W") ALU result, used for ADDW/SUBW/SLLW/SRLW/SRAW/*IW
        case (alu_op)
            ALU_ADD:  word_result = operand_1[31:0] + operand_2[31:0];
            ALU_SUB:  word_result = operand_1[31:0] - operand_2[31:0];
            ALU_SLL:  word_result = operand_1[31:0] << operand_2[4:0];
            ALU_SRL:  word_result = operand_1[31:0] >> operand_2[4:0];
            ALU_SRA:  word_result = $signed(operand_1[31:0]) >>> operand_2[4:0];
            default:  word_result = 32'b0;
        endcase

        if (is_muldiv)
            alu_result = word_op ? {{32{muldiv_word_result[31]}}, muldiv_word_result} : muldiv_full_result;
        else
            alu_result = word_op ? {{32{word_result[31]}}, word_result} : full_result;

        // JAL/JALR write back the return address (the following instruction),
        // not the ALU result -- +2 if this JAL/JALR was itself compressed
        // (C.J has no return-address writeback since it targets x0, but
        // C.JALR/C.JR expand to a real JALR here and must get this right).
        if (opcode == JAL || opcode == JALR)
            alu_result = pc + (is_compressed ? 64'd2 : 64'd4);
    end
endmodule
