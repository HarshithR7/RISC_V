`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: register_rw
// 32 x 32-bit register file. x0 is hardwired to zero. Reads are asynchronous;
// writes are synchronous (same clock edge that advances the PC), matching the
// single-cycle datapath's write-back timing.
//////////////////////////////////////////////////////////////////////////////////

module register_rw(
    input clk,
    input reset,
    input reg_write,          // Write enable signal
    input [4:0] read_reg1,    // Address of source register 1
    input [4:0] read_reg2,    // Address of source register 2
    input [4:0] write_reg,    // Address of destination register
    input [31:0] write_data,  // Data to write to destination register
    output reg [31:0] read_data1, // Data from source register 1
    output reg [31:0] read_data2  // Data from source register 2
);
    reg [31:0] registers [0:31]; // Shared register memory

    integer i;

    // Synchronous write, x0 never written
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 32'b0;
        end
        else if (reg_write && write_reg != 5'd0) begin
            registers[write_reg] <= write_data;
        end
    end

    // Asynchronous read, x0 hardwired to zero
    always @(*) begin
        read_data1 = (read_reg1 == 5'd0) ? 32'b0 : registers[read_reg1];
        read_data2 = (read_reg2 == 5'd0) ? 32'b0 : registers[read_reg2];
    end

endmodule
