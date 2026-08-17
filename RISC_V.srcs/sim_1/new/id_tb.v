`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/14/2025 09:29:30 AM
// Design Name: 
// Module Name: id_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 


module id_tb;
    // Signals
    reg [31:0] instruction;
    wire [6:0] opcode;
    wire [4:0] rs1, rs2, rd;
    wire [2:0] func3;
    wire [6:0] func7;
    wire [31:0] imm;
 

    // Instantiate the instruction decode module
    instruction_decode dut (
        .instruction(instruction),
        .opcode(opcode),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .func3(func3),
        .func7(func7),
        .imm(imm)
    );

    // Test stimulus
    initial begin
        // Initialize
        instruction = 32'b0;
        #10;

        // Test R-type instruction (add x5, x6, x7)
        instruction = 32'b0000000_00111_00110_000_00101_0110011;
        #10;

        // Test I-type instruction (addi x15, x1, 42)
        instruction = 32'b000000101010_00001_000_01111_0010011;
        #10;

        // Test S-type instruction (sw x9, 16(x2))
        instruction = 32'b0000001_01001_00010_010_10000_0100011;
        #10;

        // Test B-type instruction (beq x3, x4, 8)
        instruction = 32'b0000000_00100_00011_000_01000_1100011;
        #10;

        // Test JAL instruction (jal x1, 1024)
        instruction = 32'b01000000000000000000_00001_1101111;
        #10;

        // Test JALR instruction (jalr x1, x2, 4)
        instruction = 32'b000000000100_00010_000_00001_1100111;
        #10;

        // Test LOAD instruction (lw x10, 8(x3))
        instruction = 32'b000000001000_00011_010_01010_0000011;
        #10;

        // End simulation
        #10 $finish;
    end

    // Monitor changes
    initial begin
        $monitor("Time=%0t\nInstruction=%32b\nOpcode=%7b rs1=%5d rs2=%5d rd=%5d\nfunc3=%3b func7=%7b\nimm=%8h\n",
                 $time,
                 instruction,
                 opcode,
                 rs1, rs2, rd,
                 func3, func7,
                 imm);
    end

endmodule
