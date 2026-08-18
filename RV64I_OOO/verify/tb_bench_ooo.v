`timescale 1ns/1ps
// Phase 4 benchmark testbench: same PASS/FAIL-via-x31 convention as
// tb_core_ooo.v, but exposes ENABLE_DUAL_ISSUE so bench_ooo.py can
// compile and run the *same* benchmark program against the *same* RTL
// twice -- once dual-issue, once single-issue -- for a direct,
// apples-to-apples dispatch-width comparison (see
// riscv64_ooo_proc.v's header for why that's more trustworthy than
// maintaining two separate core copies).
module tb_bench_ooo #(
    parameter IMEM_FILE0 = "bench.mem",
    parameter IMEM_FILE1 = "idle_thread.mem",
    parameter DMEM_FILE = "bench_data.mem",
    parameter TEST_NAME = "bench",
    parameter ENABLE_DUAL_ISSUE = 1,
    parameter integer MAX_CYCLES = 50000
)();
    localparam [63:0] PASS_CODE = 64'hFFFF0000;

    reg clk, reset;
    wire [63:0] pc_out0, pc_out1;
    wire ecall_halt0, ecall_halt1;
    integer cycles;
    reg done0, done1;
    reg halt0_seen, halt1_seen;

    riscv64_ooo_proc #(.IMEM_FILE0(IMEM_FILE0), .IMEM_FILE1(IMEM_FILE1), .DMEM_FILE(DMEM_FILE), .ENABLE_DUAL_ISSUE(ENABLE_DUAL_ISSUE)) uut (
        .clk(clk), .reset(reset),
        .pc_out0(pc_out0), .pc_out1(pc_out1),
        .ecall_halt0(ecall_halt0), .ecall_halt1(ecall_halt1)
    );

    initial begin clk = 0; forever #50 clk = ~clk; end
    initial begin reset = 1; cycles = 0; done0 = 0; done1 = 0; #100; reset = 0; end

    // Same settling-delay convention as tb_core_ooo.v -- see its header
    // comment for why the condition itself, not just the later register
    // reads, must wait for this edge's NBA updates to settle.
    always @(posedge clk) begin
        if (!reset) begin
            cycles = cycles + 1;
            #1;
            halt0_seen = ecall_halt0;
            halt1_seen = ecall_halt1;

            if (!done0 && halt0_seen) begin
                done0 = 1;
                if (uut.t0_regfile0.registers[31] === PASS_CODE)
                    $display("[PASS] %0s | dual_issue=%0d | %0d cycles", TEST_NAME, ENABLE_DUAL_ISSUE, cycles);
                else
                    $display("[FAIL] %0s | dual_issue=%0d | check #%0d | %0d cycles",
                              TEST_NAME, ENABLE_DUAL_ISSUE, uut.t0_regfile0.registers[31], cycles);
            end
            if (!done1 && halt1_seen) begin
                done1 = 1;
            end
            if (done0 && done1) begin
                $finish;
            end
            if (cycles >= MAX_CYCLES) begin
                $display("[TIMEOUT] %0s | dual_issue=%0d | %0d cycles", TEST_NAME, ENABLE_DUAL_ISSUE, cycles);
                $finish;
            end
        end
    end
endmodule
