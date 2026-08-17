`timescale 1ns / 1ps
// Proof-of-concept for "configurable vector width": explicitly
// instantiates riscv64_processor with LANES=8 (VLEN=256) instead of the
// default LANES=4 (VLEN=128) every other test in this repo uses. Not
// part of the regular regression suite -- a one-off demonstration that
// the LANES parameter genuinely changes hardware behavior (an 8-element
// vadd.vv actually computes 8 elements in one instruction), not just that
// the parameter exists and does nothing.
module tb_core64_lanes8 #(
    parameter IMEM_FILE = "test.mem",
    parameter DMEM_FILE = "test_data.mem",
    parameter TEST_NAME = "test",
    parameter integer MAX_CYCLES = 20000
)();
    localparam [63:0] PASS_CODE = 64'hFFFF0000;

    reg clk, reset;
    wire [63:0] alu_result, output_mem_read, pc_out;
    wire ecall_halt;
    integer cycles;

    riscv64_processor #(.IMEM_FILE(IMEM_FILE), .DMEM_FILE(DMEM_FILE), .LANES(8)) uut (
        .clk(clk), .reset(reset),
        .alu_result(alu_result), .output_mem_read(output_mem_read),
        .pc_out(pc_out), .ecall_halt(ecall_halt)
    );

    initial begin clk = 0; forever #50 clk = ~clk; end
    initial begin reset = 1; cycles = 0; #100; reset = 0; end

    always @(posedge clk) begin
        if (!reset) begin
            cycles = cycles + 1;
            if (ecall_halt) begin
                #1;
                if (uut.regfile.registers[31] === PASS_CODE)
                    $display("[PASS] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, pc_out);
                else
                    $display("[FAIL] %0s | check #%0d | %0d cycles | PC=%h", TEST_NAME, uut.regfile.registers[31], cycles, pc_out);
                $finish;
            end
            if (cycles >= MAX_CYCLES) begin
                $display("[TIMEOUT] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, pc_out);
                $finish;
            end
        end
    end
endmodule
