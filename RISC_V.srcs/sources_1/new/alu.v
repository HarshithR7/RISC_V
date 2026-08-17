`timescale 1ns / 1ps

module execute1 (
    input wire [6:0] opcode,               // Opcode from instruction
    input wire [2:0] func3,                // Selects branch/load/store width variant
    input wire [31:0] rs1_data,            // Data from rs1 (source register 1)
    input wire [31:0] rs2_data,            // Data from rs2 (source register 2) or immediate
    input wire [31:0] imm,                 // Immediate value
    input wire [31:0] pc,                  // Current program counter (PC)
    input wire [4:0] rd,                   // Destination register
    input wire alu_src,                    // ALU source signal: 1 for immediate, 0 for register
    input wire [3:0] alu_op,               // ALU operation code (from control unit)
    output reg [31:0] alu_result,          // Result from ALU / write-back value
    output reg [31:0] mem_addr,            // Memory address for LOAD/S-type instruction
    output reg [31:0] next_pc,             // Next program counter
    output reg branch_taken,               // Indicates if branch was taken
    output reg [4:0] rd_out                // Destination register (for R, I, LOAD types)
);

    localparam R_TYPE   = 7'b0110011;
    localparam I_TYPE   = 7'b0010011;
    localparam LOAD     = 7'b0000011;
    localparam S_TYPE   = 7'b0100011;
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

    // Branch func3 encodings (RV32I)
    localparam BEQ  = 3'b000;
    localparam BNE  = 3'b001;
    localparam BLT  = 3'b100;
    localparam BGE  = 3'b101;
    localparam BLTU = 3'b110;
    localparam BGEU = 3'b111;

    reg [31:0] alu_operand_1;              // ALU first operand (rs1, 0 for LUI, pc for AUIPC)
    reg [31:0] alu_operand_2;              // ALU second operand (rs2 or immediate)

    always @(*) begin
        // Operand selection
        alu_operand_1 = (opcode == LUI_TYPE) ? 32'b0 :
                         (opcode == AUIPC)    ? pc    : rs1_data;
        alu_operand_2 = alu_src ? imm : rs2_data;

        // Defaults
        branch_taken = 1'b0;
        next_pc      = pc + 4;
        rd_out       = rd;
        mem_addr     = 32'h0;
        alu_result   = 32'h0;

        case (opcode)
            LOAD, S_TYPE: begin
                mem_addr = rs1_data + imm;         // Compute memory address
            end

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

                if (branch_taken)
                    next_pc = pc + imm;            // Update PC if branch is taken
            end

            JAL: begin
                next_pc = pc + imm;                // Compute target PC for jump
            end

            JALR: begin
                next_pc = (rs1_data + imm) & ~32'b1;   // Target, LSB cleared per spec
            end

            default: ; // R_TYPE / I_TYPE / LUI_TYPE / AUIPC fall through to the ALU case below
        endcase

        // ALU result (also used as the default register write-back value)
        case (alu_op)
            ALU_ADD:  alu_result = alu_operand_1 + alu_operand_2;
            ALU_SUB:  alu_result = alu_operand_1 - alu_operand_2;
            ALU_AND:  alu_result = alu_operand_1 & alu_operand_2;
            ALU_OR:   alu_result = alu_operand_1 | alu_operand_2;
            ALU_XOR:  alu_result = alu_operand_1 ^ alu_operand_2;
            ALU_SLL:  alu_result = alu_operand_1 << alu_operand_2[4:0];
            ALU_SRL:  alu_result = alu_operand_1 >> alu_operand_2[4:0];
            ALU_SRA:  alu_result = $signed(alu_operand_1) >>> alu_operand_2[4:0];
            ALU_SLT:  alu_result = ($signed(alu_operand_1) < $signed(alu_operand_2)) ? 32'd1 : 32'd0;
            ALU_SLTU: alu_result = (alu_operand_1 < alu_operand_2) ? 32'd1 : 32'd0;
            default:  alu_result = 32'b0;
        endcase

        // JAL/JALR write back the return address (PC + 4), not the ALU result
        if (opcode == JAL || opcode == JALR)
            alu_result = pc + 4;
    end

endmodule
