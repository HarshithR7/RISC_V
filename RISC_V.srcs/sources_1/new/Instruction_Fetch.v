`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/06/2025 10:56:42 AM
// Design Name: 
// Module Name: instruction_memory
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
//////////////////////////////////////////////////////////////////////////////////

module instruction_fetch #(
    parameter IMEM_FILE = "instructions.mem",
    parameter IMEM_WORDS = 4096
)(
    input clk,
    input [31:0] pc,
    output wire [31:0] instruction
);
    reg [31:0] memory [0:IMEM_WORDS-1];

    initial begin
        // Load instructions from file or initialize directly
        $readmemh(IMEM_FILE, memory);
        $display("Instruction memory loaded successfully from %s.", IMEM_FILE);
    end

    // Asynchronous (combinational) read: in a single-cycle datapath the
    // instruction for the current PC must be available within the same
    // cycle, feeding decode/execute combinationally so next_pc is ready
    // before the clock edge that advances the PC. A registered read here
    // would make `instruction` lag `pc` by a full cycle, since next_pc
    // (computed from `instruction`) and `pc` update on the very same edge.
    assign instruction = memory[pc[$clog2(IMEM_WORDS)+1:2]];

endmodule
