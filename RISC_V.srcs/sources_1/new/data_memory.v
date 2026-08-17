`timescale 1ns / 1ps

module data_memory #(
    parameter DMEM_FILE = "data.mem",
    parameter DMEM_WORDS = 4096
)(
    input clk,
    input mem_read,
    input mem_write,
    input [2:0] func3,
    input [31:0] mem_addr,
    input [31:0] write_data,
    output reg [31:0] read_data
);

    localparam LB  = 3'b000;
    localparam LH  = 3'b001;
    localparam LW  = 3'b010;
    localparam LBU = 3'b100;
    localparam LHU = 3'b101;

    localparam SB = 3'b000;
    localparam SH = 3'b001;
    localparam SW = 3'b010;

    reg [31:0] memory [0:DMEM_WORDS-1];
    wire [$clog2(DMEM_WORDS)-1:0] word_addr = mem_addr[$clog2(DMEM_WORDS)+1:2];
    wire [1:0] byte_offset = mem_addr[1:0];

    // Initialize memory with data.mem contents
    initial begin
        for (integer i = 0; i < DMEM_WORDS; i = i + 1)
            memory[i] = 32'b0;

        $readmemh(DMEM_FILE, memory);
        $display("Memory initialized from %s.", DMEM_FILE);
    end

    // Synchronous write, byte/halfword/word aware
    reg [31:0] word_before;
    reg [31:0] word_after;
    always @(posedge clk) begin
        if (mem_write) begin
            word_before = memory[word_addr];
            word_after  = word_before;
            case (func3)
                SB: case (byte_offset)
                        2'b00: word_after[7:0]   = write_data[7:0];
                        2'b01: word_after[15:8]  = write_data[7:0];
                        2'b10: word_after[23:16] = write_data[7:0];
                        2'b11: word_after[31:24] = write_data[7:0];
                    endcase
                SH: case (byte_offset[1])
                        1'b0: word_after[15:0]  = write_data[15:0];
                        1'b1: word_after[31:16] = write_data[15:0];
                    endcase
                default: word_after = write_data; // SW
            endcase
            memory[word_addr] <= word_after;
            $display("Time=%0t | MEM WRITE | Addr=%h | func3=%b | Data=%h -> Word=%h",
                     $time, mem_addr, func3, write_data, word_after);
        end
    end

    // Asynchronous read, byte/halfword/word aware with sign/zero extension
    reg  [31:0] word;
    reg  [7:0]  byte_val;
    reg  [15:0] half_val;
    always @(*) begin
        word     = memory[word_addr];
        byte_val = word[byte_offset*8 +: 8];
        half_val = word[{byte_offset[1],4'b0} +: 16];
        if (mem_read) begin
            case (func3)
                LB:  read_data = {{24{byte_val[7]}}, byte_val};
                LH:  read_data = {{16{half_val[15]}}, half_val};
                LBU: read_data = {24'b0, byte_val};
                LHU: read_data = {16'b0, half_val};
                default: read_data = word; // LW
            endcase
            $strobe("Time=%0t | MEM READ  | Addr=%h | func3=%b | ReadData=%h",
                     $time, mem_addr, func3, read_data);
        end else begin
            read_data = 32'b0;
        end
    end

endmodule
