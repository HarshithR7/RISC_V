`timescale 1ns / 1ps
// Blackbox stub for synthesis-only area estimation: instruction memory is
// a real hardware memory macro (BRAM or external SRAM in any actual
// deployment), not logic gates -- its own area is a well-known function
// of IMEM_WORDS x 16 bits and is reported separately, not folded into the
// core's LUT count. `(* blackbox *)` tells Yosys this module's internals
// are opaque: `instruction` must be treated as a genuinely unconstrained
// signal (neither a compile-time constant nor "don't care"), which keeps
// the rest of the core's control/datapath logic honestly intact instead
// of being constant- or X-propagated away (see riscv64_proc_synth.v's
// header for why the two more obvious approaches -- real $readmemh
// content, or none at all -- both collapse the whole design instead).
(* blackbox *)
module instruction_fetch #(
    parameter IMEM_FILE  = "instructions.mem",
    parameter IMEM_WORDS = 8192
)(
    input clk,
    input [63:0] pc,
    output wire [31:0] instruction
);
endmodule
