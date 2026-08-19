`timescale 1ns / 1ps
// Phase 9: lockstep dual-modular redundancy (DMR). Two fully independent
// riscv64_ooo_proc_solo instances (own private L1+L2+memory each -- NOT
// sharing an L2 the way dual_core_riscv64_ooo.v's cooperating cores do;
// redundancy means each copy must be able to keep working with zero
// coupling to the other, which is the opposite goal from Phase 8's
// coherency system) run the *identical* program from the *identical*
// initial state. Core A is the "primary" whose outputs this module
// exposes; core B is a pure checker, existing only to be compared
// against.
//
// Fault-detection scope, deliberately narrower than "catch every
// possible fault": lockstep_fault is a continuous, cycle-by-cycle
// comparison of externally-observable control flow (each thread's PC,
// each thread's ecall_halt) between the two cores, latched (sticky)
// once any divergence is ever observed, matching how a real safety
// system's fault flag needs to persist for downstream handling rather
// than self-clear the instant a transient mismatch passes. This is a
// real, deliberate scope/cost tradeoff, not an oversight: comparing full
// architectural register-file state every cycle would catch a fault the
// instant it's *computed*, including a "dead" wrong value that never
// again affects control flow -- but at the cost of a much wider,
// per-cycle comparator (every register, both write ports, both
// threads). PC/halt-only comparison is far cheaper and still catches
// the overwhelming majority of realistic faults, since almost any
// corrupted value eventually reaches *some* branch condition, memory
// address, or the final PASS_CODE/halt outcome that this project's own
// test convention already checks independently -- the same kind of
// detection-latency-vs-comparator-cost tradeoff real lockstep systems
// have to make too, not something unique to this scope.
module lockstep_dual_core #(
    parameter IMEM_FILE0 = "instructions0.mem",
    parameter IMEM_FILE1 = "instructions1.mem",
    parameter DMEM_FILE = "data.mem",   // identical initial state for both copies -- a single
                                         // shared path, not two independently-specifiable ones, so
                                         // there is no way to accidentally lockstep two cores that
                                         // never actually started from the same world state.
    parameter IMEM_WORDS = 8192,
    parameter DMEM_WORDS = 4096,
    parameter ROB_DEPTH = 8,
    parameter ALU_RS_DEPTH = 4,
    parameter MUL_RS_DEPTH = 2,
    parameter LSQ_DEPTH = 4,
    parameter ENABLE_DUAL_ISSUE = 1,
    parameter LANES = 4,
    parameter VEC_RS_DEPTH = 2,
    parameter L1_LINES = 16,
    parameter L1_LINE_BYTES = 32,
    parameter SBUF_DEPTH = 4,
    parameter L2_LINES = 64
)(
    input clk,
    input reset,
    output wire [63:0] pc_out0,   // core A's (primary) outputs
    output wire [63:0] pc_out1,
    output wire ecall_halt0,
    output wire ecall_halt1,
    output reg lockstep_fault     // sticky: sets on first divergence, stays set until reset
);
    wire [63:0] a_pc0, a_pc1, b_pc0, b_pc1;
    wire a_halt0, a_halt1, b_halt0, b_halt1;

    riscv64_ooo_proc_solo #(
        .IMEM_FILE0(IMEM_FILE0), .IMEM_FILE1(IMEM_FILE1), .DMEM_FILE(DMEM_FILE),
        .IMEM_WORDS(IMEM_WORDS), .DMEM_WORDS(DMEM_WORDS),
        .ROB_DEPTH(ROB_DEPTH), .ALU_RS_DEPTH(ALU_RS_DEPTH), .MUL_RS_DEPTH(MUL_RS_DEPTH),
        .LSQ_DEPTH(LSQ_DEPTH), .ENABLE_DUAL_ISSUE(ENABLE_DUAL_ISSUE), .LANES(LANES),
        .VEC_RS_DEPTH(VEC_RS_DEPTH), .L1_LINES(L1_LINES), .L1_LINE_BYTES(L1_LINE_BYTES),
        .SBUF_DEPTH(SBUF_DEPTH), .L2_LINES(L2_LINES)
    ) core_a (
        .clk(clk), .reset(reset),
        .pc_out0(a_pc0), .pc_out1(a_pc1),
        .ecall_halt0(a_halt0), .ecall_halt1(a_halt1)
    );

    riscv64_ooo_proc_solo #(
        .IMEM_FILE0(IMEM_FILE0), .IMEM_FILE1(IMEM_FILE1), .DMEM_FILE(DMEM_FILE),
        .IMEM_WORDS(IMEM_WORDS), .DMEM_WORDS(DMEM_WORDS),
        .ROB_DEPTH(ROB_DEPTH), .ALU_RS_DEPTH(ALU_RS_DEPTH), .MUL_RS_DEPTH(MUL_RS_DEPTH),
        .LSQ_DEPTH(LSQ_DEPTH), .ENABLE_DUAL_ISSUE(ENABLE_DUAL_ISSUE), .LANES(LANES),
        .VEC_RS_DEPTH(VEC_RS_DEPTH), .L1_LINES(L1_LINES), .L1_LINE_BYTES(L1_LINE_BYTES),
        .SBUF_DEPTH(SBUF_DEPTH), .L2_LINES(L2_LINES)
    ) core_b (
        .clk(clk), .reset(reset),
        .pc_out0(b_pc0), .pc_out1(b_pc1),
        .ecall_halt0(b_halt0), .ecall_halt1(b_halt1)
    );

    assign pc_out0 = a_pc0;
    assign pc_out1 = a_pc1;
    assign ecall_halt0 = a_halt0;
    assign ecall_halt1 = a_halt1;

    wire mismatch_this_cycle = (a_pc0 != b_pc0) || (a_pc1 != b_pc1) ||
                                (a_halt0 != b_halt0) || (a_halt1 != b_halt1);

    always @(posedge clk or posedge reset) begin
        if (reset)
            lockstep_fault <= 1'b0;
        else if (mismatch_this_cycle)
            lockstep_fault <= 1'b1;
    end
endmodule
