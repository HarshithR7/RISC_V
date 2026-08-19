`timescale 1ns / 1ps
// Generic self-checking testbench for the out-of-order core. Same
// convention as the single-cycle RV64I suite's tb_core64.v: a program ends
// with `li x31, 0xFFFF0000 ; ecall` on success. ecall_halt0/ecall_halt1
// fire when ECALL retires at each thread's own ROB head (see
// riscv64_ooo_proc.v), i.e. only once every older in-flight instruction on
// that thread -- including a still-executing multi-cycle multiply or
// divide -- has actually committed, so the x31 checks below are guaranteed
// to observe final, precise per-thread architectural state, not a
// snapshot mid-flight.
//
// Phase 7 (SMT): the core now always runs two threads. Most existing
// tests only care about thread 0's result; thread 1 defaults to a trivial
// "li x31, 0xFFFF0000; ecall" idle program (IMEM_FILE1's default) so it
// halts almost immediately and reports its own [PASS-T1] without any
// special-casing needed here -- genuine SMT tests instead give both
// IMEM_FILE0 and IMEM_FILE1 real, independent programs and check both
// [PASS-T0]/[PASS-T1] lines. Simulation ends once BOTH threads have
// halted (or the shared MAX_CYCLES timeout elapses).
module tb_core_ooo #(
    parameter IMEM_FILE0 = "test.mem",
    parameter IMEM_FILE1 = "idle_thread.mem",
    parameter DMEM_FILE = "test_data.mem",
    parameter TEST_NAME = "test",
    parameter integer MAX_CYCLES = 20000
)();
    localparam [63:0] PASS_CODE = 64'hFFFF0000;

    reg clk, reset;
    wire [63:0] pc_out0, pc_out1;
    wire ecall_halt0, ecall_halt1;
    integer cycles;
    reg done0, done1;

    // Phase 8: riscv64_ooo_proc.v no longer owns a backing memory directly
    // (see its header) -- riscv64_ooo_proc_solo.v pairs it with a private
    // l2_cache.v so single-core tests stay self-contained. Hierarchical
    // references below go one level deeper (uut.core.*) than Phase 1-7's
    // uut.* accordingly.
    riscv64_ooo_proc_solo #(.IMEM_FILE0(IMEM_FILE0), .IMEM_FILE1(IMEM_FILE1), .DMEM_FILE(DMEM_FILE)) uut (
        .clk(clk), .reset(reset),
        .pc_out0(pc_out0), .pc_out1(pc_out1),
        .ecall_halt0(ecall_halt0), .ecall_halt1(ecall_halt1)
    );

    initial begin clk = 0; forever #50 clk = ~clk; end
    initial begin reset = 1; cycles = 0; done0 = 0; done1 = 0; #100; reset = 0; end

    // ecall_halt0/ecall_halt1 are combinational wires derived from this
    // same edge's NBA-updated ROB state (head_ptr/valid_arr/done_arr
    // inside rob.v) -- evaluating them directly in this block's `if`
    // condition, with no settling delay, races that NBA update: whether
    // this read observes the pre- or post-edge value is scheduler-
    // dependent, not guaranteed. Settling with `#1` *before* forming the
    // conditions (not only before the later register reads, which was
    // this testbench's original -- and, it turns out, still racy --
    // convention) makes every read in this block observe a single,
    // consistent, fully-settled post-edge snapshot.
    reg halt0_seen, halt1_seen;
    always @(posedge clk) begin
        if (!reset) begin
            cycles = cycles + 1;
            #1;
            halt0_seen = ecall_halt0;
            halt1_seen = ecall_halt1;

            if (!done0 && halt0_seen) begin
                done0 = 1;
                // Unconditional register dump (x5-x10) for thread 0, same
                // convention as Phase 1-6's single-thread [REGS] line --
                // see run_branch_free's header in build_tests_ooo.py.
                $display("[REGS] x5=%h x6=%h x7=%h x8=%h x9=%h x10=%h",
                          uut.core.t0_regfile0.data_mem[5], uut.core.t0_regfile0.data_mem[6],
                          uut.core.t0_regfile0.data_mem[7], uut.core.t0_regfile0.data_mem[8],
                          uut.core.t0_regfile0.data_mem[9], uut.core.t0_regfile0.data_mem[10]);
                // Vector register dump (v1-v4): thread-0-only (Phase 6/7
                // scope), same permanent verification-hook convention as
                // before (no vector store/extract instruction in scope).
                $display("[VREGS] v1=%h v2=%h v3=%h v4=%h",
                          uut.core.vregfile_i.registers[1], uut.core.vregfile_i.registers[2],
                          uut.core.vregfile_i.registers[3], uut.core.vregfile_i.registers[4]);
                if (uut.core.t0_regfile0.data_mem[31] === PASS_CODE)
                    $display("[PASS-T0] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, pc_out0);
                else
                    $display("[FAIL-T0] %0s | check #%0d | %0d cycles | PC=%h", TEST_NAME, uut.core.t0_regfile0.data_mem[31], cycles, pc_out0);
            end

            if (!done1 && halt1_seen) begin
                done1 = 1;
                $display("[REGS1] x5=%h x6=%h x7=%h x8=%h x9=%h x10=%h",
                          uut.core.t1_regfile0.data_mem[5], uut.core.t1_regfile0.data_mem[6],
                          uut.core.t1_regfile0.data_mem[7], uut.core.t1_regfile0.data_mem[8],
                          uut.core.t1_regfile0.data_mem[9], uut.core.t1_regfile0.data_mem[10]);
                if (uut.core.t1_regfile0.data_mem[31] === PASS_CODE)
                    $display("[PASS-T1] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, pc_out1);
                else
                    $display("[FAIL-T1] %0s | check #%0d | %0d cycles | PC=%h", TEST_NAME, uut.core.t1_regfile0.data_mem[31], cycles, pc_out1);
            end

            if (done0 && done1) begin
                $finish;
            end

            if (cycles >= MAX_CYCLES) begin
                if (!done0) $display("[TIMEOUT-T0] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, pc_out0);
                if (!done1) $display("[TIMEOUT-T1] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, pc_out1);
                $finish;
            end
        end
    end
endmodule
