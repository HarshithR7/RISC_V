`timescale 1ns/1ps
// Phase 4 benchmark testbench: same PASS/FAIL-via-x31 convention as
// tb_core_ooo.v, but exposes ENABLE_DUAL_ISSUE so bench_ooo.py can
// compile and run the *same* benchmark program against the *same* RTL
// twice -- once dual-issue, once single-issue -- for a direct,
// apples-to-apples dispatch-width comparison (see
// riscv64_ooo_proc.v's header for why that's more trustworthy than
// maintaining two separate core copies).
module tb_bench_ooo #(
    parameter IMEM_FILE = "bench.mem",
    parameter DMEM_FILE = "bench_data.mem",
    parameter TEST_NAME = "bench",
    parameter ENABLE_DUAL_ISSUE = 1,
    parameter integer MAX_CYCLES = 50000
)();
    localparam [63:0] PASS_CODE = 64'hFFFF0000;

    reg clk, reset;
    wire [63:0] pc_out;
    wire ecall_halt;
    integer cycles;

    riscv64_ooo_proc #(.IMEM_FILE(IMEM_FILE), .DMEM_FILE(DMEM_FILE), .ENABLE_DUAL_ISSUE(ENABLE_DUAL_ISSUE)) uut (
        .clk(clk), .reset(reset),
        .pc_out(pc_out), .ecall_halt(ecall_halt)
    );

    initial begin clk = 0; forever #50 clk = ~clk; end
    initial begin reset = 1; cycles = 0; #100; reset = 0; end

    always @(posedge clk) begin
        if (!reset) begin
            cycles = cycles + 1;
            if (ecall_halt) begin
                #1;
                if (uut.regfile0.registers[31] === PASS_CODE)
                    $display("[PASS] %0s | dual_issue=%0d | %0d cycles", TEST_NAME, ENABLE_DUAL_ISSUE, cycles);
                else
                    $display("[FAIL] %0s | dual_issue=%0d | check #%0d | %0d cycles",
                              TEST_NAME, ENABLE_DUAL_ISSUE, uut.regfile0.registers[31], cycles);
                $finish;
            end
            if (cycles >= MAX_CYCLES) begin
                $display("[TIMEOUT] %0s | dual_issue=%0d | %0d cycles", TEST_NAME, ENABLE_DUAL_ISSUE, cycles);
                $finish;
            end
        end
    end
endmodule
