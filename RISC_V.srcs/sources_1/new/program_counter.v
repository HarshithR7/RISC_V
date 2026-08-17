`timescale 1ns / 1ps

module Program_counter(
    input clk,              // Clock signal
    input reset,            // Reset signal
    input [31:0] pc_in,
    output reg [31:0] pc_out     // Current PC
);
    // Initialize PC
    initial begin
        pc_out = 32'h0;
    end

    always @(posedge clk or posedge reset) begin
 
        if (reset) begin
            // Reset PC to 0
            pc_out <= 32'b0;
        end
        
            else begin
                // Otherwise, increment PC by 4 (next instruction)
                pc_out <= pc_in;
            end
        end
endmodule