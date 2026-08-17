`timescale 1ns / 1ps

module instruction_decode(
    input [31:0] instruction,     // 32-bit instruction input
    
    // Common fields
    output wire [6:0] opcode,     // Opcode (7 bits)
    output wire [4:0] rs1,        // Source register 1 (5 bits)
    output wire [4:0] rs2,        // Source register 2 (5 bits, R & S type)
    output wire [4:0] rd,         // Destination register (5 bits)
    output wire [2:0] func3,      // Function code 3 (3 bits)
    output wire [6:0] func7,      // Function code 7 (7 bits, R-type)
    output reg [31:0] imm,        // Immediate value
    output reg alu_src
);
   // reg alu_src;
    // Extract common fields using continuous assignments
    assign opcode = instruction[6:0];         // Bits 0-6
    assign rs1    = instruction[19:15];       // Bits 15-19
    assign rs2    = instruction[24:20];       // Bits 20-24
    assign rd     = instruction[11:7];        // Bits 7-11
    assign func3  = instruction[14:12];       // Bits 12-14
    assign func7  = instruction[31:25];       // Bits 25-31 (R-type only)
    
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
    
    // Immediate generation and ALU-source selection
    always @(*) begin
        alu_src = 0; // default: register operand (R_TYPE, B_TYPE)

        case (opcode)
            LOAD: begin
                imm = {{20{instruction[31]}}, instruction[31:20]};
                alu_src = 1;
            end
            I_TYPE: begin
                imm = {{20{instruction[31]}}, instruction[31:20]};
                alu_src = 1;
            end
            JALR: begin
                imm = {{20{instruction[31]}}, instruction[31:20]};
                alu_src = 1;
            end
            S_TYPE: begin
                imm = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
                alu_src = 1;
            end
            B_TYPE:
                imm = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            JAL:
                imm = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
            LUI_TYPE, AUIPC: begin
                imm = {instruction[31:12], 12'b0};
                alu_src = 1;
            end
            default:
                imm = 32'b0;
        endcase
    end

endmodule
