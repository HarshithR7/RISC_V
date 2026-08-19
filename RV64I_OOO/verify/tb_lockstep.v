`timescale 1ns / 1ps
// Lockstep DMR testbench: same PASS/FAIL-via-x31 convention as
// tb_core_ooo.v (checked against core A, the primary), plus checking
// lockstep_fault itself -- this suite's actual point.
//
// FAULT_INJECT_CYCLE (0 = disabled) flips one bit of core B's own
// architectural x7 register once, at that exact cycle count, simulating
// a soft-error single-event upset in one of the two redundant copies --
// a real, hierarchical procedural assignment in this testbench, not an
// RTL fault-injection port (a synthesizable "please corrupt yourself"
// input would be a strange thing for a real redundant core to have;
// this is simulation-only, exactly like the hierarchical-reference
// debug technique this project already uses throughout its own
// development, just applied as a permanent test mechanism instead of a
// one-off).
//
// A single, one-time hierarchical assignment (not `force`/`release`):
// a soft error is a single-event upset, not a sustained drive, so a
// plain procedural write to the target reg -- issued once, well clear
// of any clock edge -- is the semantically correct model, and it sidesteps
// two real problems found while getting this right. First, `force` on
// an *indexed word of a variable array* (`registers[5]`, a Verilog
// memory) is unsupported by Icarus's code generator ("cannot %force/vec4
// to the word of a variable array") -- it only supports forcing whole
// scalars/nets, not a memory word. Second, and more fundamentally, even
// where `force` does work (e.g. on a wire), `force` then `release` a
// mere `#1` later collapses the corruption back to the driven value
// before the *next* clock edge ever arrives -- so the registered
// lockstep comparator, which only samples at posedge clk, never
// actually observes a divergence; this is what silently defeated an
// earlier attempt that forced the PC wire. A one-time write to the
// architectural register itself avoids both: it's plain assignment
// (not force) so it compiles, and it persists in the register file
// (a real reg) until the next real write, so it's still corrupted at
// the next posedge sample.
//
// Targets x7, this test program's loop *bound* (`li x7, N`, set once
// before the loop and never rewritten), not x6 (the loop counter) or x5
// (the accumulator). x5 was tried first and found not to move the PC at
// all: the branch never inspects it. x6 was tried next and also found
// not to move the PC, for a subtler reason: x6 is redefined every single
// iteration (`addi x6, x6, 1`), so at dispatch time its RAT entry is
// almost always still busy (pointing at the just-issued producer's ROB
// tag) -- meaning each iteration's consumers overwhelmingly capture x6's
// operand via CDB/tag forwarding, not via a fresh read of the committed
// architectural register file, so corrupting the *committed copy* of a
// tightly RAW-chained register is largely invisible to instructions
// already in flight. x7, by contrast, is written once, commits early,
// and then sits with its RAT entry permanently not-busy for the rest of
// the loop -- so every later iteration's `bne x6, x7, loop` dispatch
// freshly reads x7 straight out of the architectural register file,
// guaranteeing a post-injection iteration actually observes the
// corrupted bound and diverges control flow from core A.
//
// Flips *two* bits of x7, not one, in `data_mem[7]` directly -- a fourth
// iteration, needed only after Phase 9's own ECC work (ecc_register_file.v)
// landed on top of this same lockstep DMR. A single-bit corruption there
// is now exactly the class of fault SECDED ECC exists to silently heal:
// the very next read transparently corrects it before it can ever reach
// a branch, so lockstep_fault stopped firing (correctly -- ECC did its
// job before lockstep ever needed to). Flipping two bits produces an
// uncorrectable (dbe) error instead, which ecc_register_file.v passes
// through unmodified (with a fault flag raised, but no attempted
// correction) rather than silently reconstructing -- data genuinely
// reaches the pipeline wrong, so it still diverges control flow the same
// way the original single-bit test intended. This is the intended,
// realistic layering of the two RAS mechanisms: ECC catches the common
// single-bit case for free; lockstep is the backstop for whatever ECC
// structurally cannot fix (double-bit upsets here, or -- in general --
// any fault outside ECC-protected storage entirely, e.g. in combinational
// logic).
module tb_lockstep #(
    parameter IMEM_FILE0 = "test.mem",
    parameter IMEM_FILE1 = "idle_thread.mem",
    parameter DMEM_FILE = "test_data.mem",
    parameter TEST_NAME = "test",
    parameter integer MAX_CYCLES = 20000,
    parameter integer FAULT_INJECT_CYCLE = 0
)();
    localparam [63:0] PASS_CODE = 64'hFFFF0000;

    reg clk, reset;
    wire [63:0] pc_out0, pc_out1;
    wire ecall_halt0, ecall_halt1;
    wire lockstep_fault;
    integer cycles;
    reg done0, done1;
    reg halt0_seen, halt1_seen;

    lockstep_dual_core #(.IMEM_FILE0(IMEM_FILE0), .IMEM_FILE1(IMEM_FILE1), .DMEM_FILE(DMEM_FILE)) uut (
        .clk(clk), .reset(reset),
        .pc_out0(pc_out0), .pc_out1(pc_out1),
        .ecall_halt0(ecall_halt0), .ecall_halt1(ecall_halt1),
        .lockstep_fault(lockstep_fault)
    );

    initial begin clk = 0; forever #50 clk = ~clk; end
    initial begin reset = 1; cycles = 0; done0 = 0; done1 = 0; #100; reset = 0; end

    // Same settling-delay convention as tb_core_ooo.v -- see its header
    // for why the condition itself, not just later register reads, must
    // wait for this edge's NBA updates to settle.
    always @(posedge clk) begin
        if (!reset) begin
            cycles = cycles + 1;

            if (FAULT_INJECT_CYCLE != 0 && cycles == FAULT_INJECT_CYCLE) begin
                // Two bits, not one: see this file's header for why a
                // single-bit flip here is no longer enough once the
                // register file is ECC-protected (Phase 9's SECDED work,
                // layered on top of this same lockstep DMR).
                uut.core_b.core.t0_regfile0.data_mem[7] =
                    uut.core_b.core.t0_regfile0.data_mem[7] ^ 64'h3;
                uut.core_b.core.t0_regfile1.data_mem[7] =
                    uut.core_b.core.t0_regfile1.data_mem[7] ^ 64'h3;
            end

            #1;
            halt0_seen = ecall_halt0;
            halt1_seen = ecall_halt1;

            if (!done0 && halt0_seen) begin
                done0 = 1;
                if (uut.core_a.core.t0_regfile0.data_mem[31] === PASS_CODE)
                    $display("[PASS-T0] %0s | %0d cycles | PC=%h | lockstep_fault=%b", TEST_NAME, cycles, pc_out0, lockstep_fault);
                else
                    $display("[FAIL-T0] %0s | check #%0d | %0d cycles | PC=%h | lockstep_fault=%b",
                              TEST_NAME, uut.core_a.core.t0_regfile0.data_mem[31], cycles, pc_out0, lockstep_fault);
            end
            if (!done1 && halt1_seen) begin
                done1 = 1;
                if (uut.core_a.core.t1_regfile0.data_mem[31] === PASS_CODE)
                    $display("[PASS-T1] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, pc_out1);
                else
                    $display("[FAIL-T1] %0s | check #%0d | %0d cycles | PC=%h", TEST_NAME, uut.core_a.core.t1_regfile0.data_mem[31], cycles, pc_out1);
            end

            if (done0 && done1) begin
                $display("[LOCKSTEP] %0s | fault=%b", TEST_NAME, lockstep_fault);
                $finish;
            end

            if (cycles >= MAX_CYCLES) begin
                if (!done0) $display("[TIMEOUT-T0] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, pc_out0);
                if (!done1) $display("[TIMEOUT-T1] %0s | %0d cycles | PC=%h", TEST_NAME, cycles, pc_out1);
                $display("[LOCKSTEP] %0s | fault=%b", TEST_NAME, lockstep_fault);
                $finish;
            end
        end
    end
endmodule
