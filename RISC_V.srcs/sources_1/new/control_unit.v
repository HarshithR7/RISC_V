`timescale 1ns / 1ps

module ControlUnit(
    //input [31:0] instruction,           // 32-bit instruction input
    input [6:0] opcode,          // Opcode (7 bits)
             // Destination register (5 bits)
    input [2:0] func3,           // Function code 3 (3 bits)
    input [6:0] func7,           // Function code 7 (7 bits, R-type)
                  // Function code 7 (7 bits, R-type)
    
    output reg reg_dst,
    output reg branch,
    output reg mem_read,
    output reg mem_to_reg,
    output reg [3:0] alu_op,
    output reg mem_write,
    //output reg alu_src,
    output reg reg_write,
    output reg jump,
    output reg [2:0] branch_type,
    output reg is_shift
    //output reg [31:0] imm
);

//assign opcode = instruction[6:0];
//assign func3 = instruction[14:12];
//assign func7 = instruction[31:25];


    // Opcode definitions for RV32I ISA
    localparam R_TYPE    = 7'b0110011;
    localparam I_TYPE    = 7'b0010011;
    localparam S_TYPE    = 7'b0100011;
    localparam B_TYPE    = 7'b1100011;
    localparam LUI_TYPE  = 7'b0110111;
    localparam AUIPC     = 7'b0010111;
    localparam JAL       = 7'b1101111;
    localparam JALR      = 7'b1100111;
    localparam LOAD      = 7'b0000011;
    
    // ALU Operation codes
    localparam ALU_ADD   = 4'b0010;
    localparam ALU_SUB   = 4'b1010;
    localparam ALU_AND   = 4'b0100;
    localparam ALU_OR    = 4'b0101;
    localparam ALU_XOR   = 4'b0011;
    localparam ALU_SLL   = 4'b0110;
    localparam ALU_SRL   = 4'b0111;
    localparam ALU_SRA   = 4'b1000;
    localparam ALU_SLT   = 4'b1011;
    localparam ALU_SLTU  = 4'b1100;
    
    initial begin
    $monitor("Time: %0t | reg_write: %b | mem_to_reg: %b", 
             $time,  reg_write, mem_to_reg);
end


    always @(*) begin
        // Default control signals
        reg_dst     = 0;
        branch      = 0;
        mem_read    = 0;
        mem_to_reg  = 0;
        alu_op      = ALU_ADD;
        mem_write   = 0;
      //  alu_src     = 0;
        reg_write   = 0;
        jump        = 0;
        is_shift    = 0;
        branch_type = 3'b0;

        case (opcode)
            // R-Type Instructions
            R_TYPE: begin
                reg_dst   = 1;
                reg_write = 1;
                
                case ({func7, func3})
                    10'b0000000_000: alu_op = ALU_ADD;   // ADD
                    10'b0100000_000: alu_op = ALU_SUB;   // SUB
                    10'b0000000_111: alu_op = ALU_AND;   // AND
                    10'b0000000_110: alu_op = ALU_OR;    // OR
                    10'b0000000_100: alu_op = ALU_XOR;   // XOR
                    10'b0000000_001: begin
                        alu_op = ALU_SLL;                 // SLL
                        is_shift = 1;
                    end
                    10'b0000000_101: begin
                        alu_op = ALU_SRL;                 // SRL
                        is_shift = 1;
                    end
                    10'b0100000_101: begin
                        alu_op = ALU_SRA;                 // SRA
                        is_shift = 1;
                    end
                    10'b0000000_010: alu_op = ALU_SLT;   // SLT
                    10'b0000000_011: alu_op = ALU_SLTU;  // SLTU
                endcase
            end

            // I-Type Instructions (ALU operations)
            I_TYPE: begin
                //alu_src   = 1;
                reg_write = 1;
                 // I-type: imm[11:0]
              
                case (func3)
                    3'b000: alu_op = ALU_ADD;   // ADDI
                    3'b010: alu_op = ALU_SLT;   // SLTI
                    3'b011: alu_op = ALU_SLTU;  // SLTIU
                    3'b100: alu_op = ALU_XOR;   // XORI
                    3'b110: alu_op = ALU_OR;    // ORI
                    3'b111: alu_op = ALU_AND;   // ANDI
                    3'b001: begin
                        if (func7 == 7'b0000000) begin
                            alu_op = ALU_SLL;   // SLLI
                            is_shift = 1;
                        end
                    end
                    3'b101: begin
                        if (func7 == 7'b0000000) begin
                            alu_op = ALU_SRL;   // SRLI
                            is_shift = 1;
                        end
                        else if (func7 == 7'b0100000) begin
                            alu_op = ALU_SRA;   // SRAI
                            is_shift = 1;
                        end
                    end
                endcase
            end

            // Load Instructions
            LOAD: begin
                //alu_src   = 1;
                mem_to_reg = 1;
                reg_write = 1;
                mem_read  = 1;
                alu_op    = ALU_ADD;   // Address calculation
            end

            // Store Instructions (S-Type)
            S_TYPE: begin
                //alu_src   = 1;
                mem_write = 1;
            
                alu_op    = ALU_ADD;   // Address calculation
            end

            // Branch Instructions (B-Type)
            B_TYPE: begin
                branch    = 1;
                alu_op    = ALU_SUB; 
                  // Branch comparison (e.g., BEQ, BNE)
                branch_type = func3;
                
            end

            // JAL (J-Type)
            JAL: begin
                jump      = 1;
                reg_write = 1;
               
             //   alu_op    = ALU_ADD;   // PC + immediate
            end

            // JALR (I-Type)
            JALR: begin
                jump      = 1;
                reg_write = 1;
                //alu_src   = 1;
                alu_op    = ALU_ADD;   // PC + immediate
            end

            // LUI (U-Type)
            LUI_TYPE: begin
                reg_write = 1;
                //alu_src   = 1;
                alu_op    = ALU_ADD;   // rd = 0 + imm (operand-1 forced to 0 in execute stage)
            end

            // AUIPC (U-Type)
            AUIPC: begin
                reg_write = 1;
                //alu_src   = 1;
                
                alu_op    = ALU_ADD;   // PC + immediate
            end

           default: begin
        reg_dst = 1'b0;
        branch = 1'b0;
        mem_read = 1'b0;
        mem_to_reg = 1'b0;
        alu_op = 4'b0;
        mem_write = 1'b0;
        //alu_src = 1'b0;
        reg_write = 1'b0;
        jump = 1'b0;
        is_shift = 1'b0;
        branch_type = 1'b0;
    end
    
        endcase
        // Final debug statement for each instruction processed
        $display("Final Control Signals - Opcode=%b | RegWrite=%b | MemRead=%b | MemWrite=%b | Jump=%b | Branch=%b", 
                 opcode, reg_write, mem_read, mem_write, jump, branch);
        $display("---------------------------------------------");
    end
endmodule
