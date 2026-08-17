`timescale 1ns / 1ps
// Generic self-checking testbench core. Each test program is expected to end
// with:
//   li x31, 0xFFFF0000 ; ecall          on success
//   li x31, <check_id>  ; ecall         on the first failed check (check_id != 0)
// so a single generic harness can run every program and report PASS/FAIL/TIMEOUT
// without per-test Verilog.
module tb_core #(
    parameter IMEM_FILE = "test.mem",
    parameter DMEM_FILE = "test_data.mem",
    parameter TEST_NAME = "test",
    parameter integer MAX_CYCLES = 20000
)();
    localparam [31:0] PASS_CODE = 32'hFFFF0000;

    reg clk, reset;
    wire [31:0] alu_result, output_mem_read, pc_out;
    wire ecall_halt;
    integer cycles;

    riscv_processor #(.IMEM_FILE(IMEM_FILE), .DMEM_FILE(DMEM_FILE)) uut (
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
