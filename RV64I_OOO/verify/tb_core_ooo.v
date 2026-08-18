`timescale 1ns / 1ps
// Generic self-checking testbench for the out-of-order core. Same
// convention as the single-cycle RV64I suite's tb_core64.v: program ends
// with `li x31, 0xFFFF0000 ; ecall` on success. ecall_halt fires when
// ECALL retires at the ROB head (see riscv64_ooo_proc.v), i.e. only once
// every older in-flight instruction -- including a still-executing
// multi-cycle multiply or divide -- has actually committed, so the
// x31 check below is guaranteed to observe final, precise architectural
// state, not a snapshot mid-flight.
module tb_core_ooo #(
    parameter IMEM_FILE = "test.mem",
    parameter DMEM_FILE = "test_data.mem",
    parameter TEST_NAME = "test",
    parameter integer MAX_CYCLES = 20000
)();
    localparam [63:0] PASS_CODE = 64'hFFFF0000;

    reg clk, reset;
    wire [63:0] pc_out;
    wire ecall_halt;
    integer cycles;

    riscv64_ooo_proc #(.IMEM_FILE(IMEM_FILE), .DMEM_FILE(DMEM_FILE)) uut (
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
                // Unconditional register dump (x5-x10), independent of
                // the x31/PASS_CODE convention below: step 3's earliest
                // tests (before branch support exists in step 4) can't
                // use the usual bne-based check_eq self-check, since that
                // needs a branch instruction this core doesn't dispatch
                // yet -- those tests instead write their result into a
                // known register and end with an unconditional `ecall`,
                // and the Python driver greps this line for the expected
                // value instead of relying on in-program branching.
                $display("[REGS] x5=%h x6=%h x7=%h x8=%h x9=%h x10=%h",
                          uut.regfile0.registers[5], uut.regfile0.registers[6],
                          uut.regfile0.registers[7], uut.regfile0.registers[8],
                          uut.regfile0.registers[9], uut.regfile0.registers[10]);
                if (uut.regfile0.registers[31] === PASS_CODE)
                    $display("[PASS] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, pc_out);
                else
                    $display("[FAIL] %0s | check #%0d | %0d cycles | PC=%h", TEST_NAME, uut.regfile0.registers[31], cycles, pc_out);
                $finish;
            end
            if (cycles >= MAX_CYCLES) begin
                $display("[TIMEOUT] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, pc_out);
                $finish;
            end
        end
    end
endmodule
