`timescale 1ns / 1ps
// Dual-core system testbench: same PASS/FAIL-via-x31 convention as
// tb_core_ooo.v, applied to all 4 hardware threads (2 cores x 2 SMT
// threads each). Simulation ends once all 4 have halted (or the shared
// MAX_CYCLES timeout elapses). This is what actually exercises real
// cross-core MESI coherency through the full pipeline -- see
// build_dual_core_tests.py's producer/consumer test for the point of it.
module tb_dual_core #(
    parameter C0_IMEM_FILE0 = "core0_t0.mem",
    parameter C0_IMEM_FILE1 = "idle_thread.mem",
    parameter C1_IMEM_FILE0 = "core1_t0.mem",
    parameter C1_IMEM_FILE1 = "idle_thread.mem",
    parameter DMEM_FILE = "test_data.mem",
    parameter TEST_NAME = "dual",
    parameter integer MAX_CYCLES = 20000
)();
    localparam [63:0] PASS_CODE = 64'hFFFF0000;

    reg clk, reset;
    wire [63:0] c0t0_pc, c0t1_pc, c1t0_pc, c1t1_pc;
    wire c0t0_halt, c0t1_halt, c1t0_halt, c1t1_halt;
    integer cycles;
    reg d00, d01, d10, d11;
    reg h00, h01, h10, h11;

    dual_core_riscv64_ooo #(
        .C0_IMEM_FILE0(C0_IMEM_FILE0), .C0_IMEM_FILE1(C0_IMEM_FILE1),
        .C1_IMEM_FILE0(C1_IMEM_FILE0), .C1_IMEM_FILE1(C1_IMEM_FILE1),
        .DMEM_FILE(DMEM_FILE)
    ) uut (
        .clk(clk), .reset(reset),
        .c0t0_pc_out(c0t0_pc), .c0t1_pc_out(c0t1_pc),
        .c0t0_ecall_halt(c0t0_halt), .c0t1_ecall_halt(c0t1_halt),
        .c1t0_pc_out(c1t0_pc), .c1t1_pc_out(c1t1_pc),
        .c1t0_ecall_halt(c1t0_halt), .c1t1_ecall_halt(c1t1_halt)
    );

    initial begin clk = 0; forever #50 clk = ~clk; end
    initial begin reset = 1; cycles = 0; d00=0; d01=0; d10=0; d11=0; #100; reset = 0; end

    // Same settling-delay convention as tb_core_ooo.v.
    always @(posedge clk) begin
        if (!reset) begin
            cycles = cycles + 1;
            #1;
            h00 = c0t0_halt; h01 = c0t1_halt; h10 = c1t0_halt; h11 = c1t1_halt;

            if (!d00 && h00) begin
                d00 = 1;
                if (uut.core0.t0_regfile0.registers[31] === PASS_CODE)
                    $display("[PASS-C0T0] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, c0t0_pc);
                else
                    $display("[FAIL-C0T0] %0s | check #%0d | %0d cycles | PC=%h", TEST_NAME, uut.core0.t0_regfile0.registers[31], cycles, c0t0_pc);
            end
            if (!d01 && h01) begin
                d01 = 1;
                if (uut.core0.t1_regfile0.registers[31] === PASS_CODE)
                    $display("[PASS-C0T1] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, c0t1_pc);
                else
                    $display("[FAIL-C0T1] %0s | check #%0d | %0d cycles | PC=%h", TEST_NAME, uut.core0.t1_regfile0.registers[31], cycles, c0t1_pc);
            end
            if (!d10 && h10) begin
                d10 = 1;
                if (uut.core1.t0_regfile0.registers[31] === PASS_CODE)
                    $display("[PASS-C1T0] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, c1t0_pc);
                else
                    $display("[FAIL-C1T0] %0s | check #%0d | %0d cycles | PC=%h", TEST_NAME, uut.core1.t0_regfile0.registers[31], cycles, c1t0_pc);
            end
            if (!d11 && h11) begin
                d11 = 1;
                if (uut.core1.t1_regfile0.registers[31] === PASS_CODE)
                    $display("[PASS-C1T1] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, c1t1_pc);
                else
                    $display("[FAIL-C1T1] %0s | check #%0d | %0d cycles | PC=%h", TEST_NAME, uut.core1.t1_regfile0.registers[31], cycles, c1t1_pc);
            end

            if (d00 && d01 && d10 && d11) $finish;

            if (cycles >= MAX_CYCLES) begin
                if (!d00) $display("[TIMEOUT-C0T0] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, c0t0_pc);
                if (!d01) $display("[TIMEOUT-C0T1] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, c0t1_pc);
                if (!d10) $display("[TIMEOUT-C1T0] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, c1t0_pc);
                if (!d11) $display("[TIMEOUT-C1T1] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, c1t1_pc);
                $finish;
            end
        end
    end
endmodule
