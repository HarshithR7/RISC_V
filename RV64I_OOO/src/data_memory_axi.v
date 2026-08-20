`timescale 1ns / 1ps
// FPGA bring-up (real hardware timing + PS-side program loading): a
// registered-read, AXI-writable variant of the sibling single-cycle
// core's data_memory.v, implementing ONLY the RVV unit-stride vector
// port -- the one interface l2_cache.v's backing-memory instance
// actually uses (its scalar port is permanently tied off). Not a
// wrapper around data_memory.v (unlike instruction_fetch_reg.v): that
// module's read is combinational, used unconditionally by both this
// port and a tied-off scalar port sharing the same array, so cleanly
// separating "just the vector port, registered, plus a write port" is
// simplest as its own small, self-contained implementation rather than
// fighting the original's structure.
//
// Real Xilinx Block RAM has no combinational-read mode at this design's
// backing-memory capacity -- same reasoning as instruction_fetch_reg.v's
// own header. l2_cache.v's ST_MEM_WAIT/ST_MEM_WAIT2 pair already assumes
// exactly this one-cycle latency (see l2_cache.v's own comment there),
// so this module is a correct drop-in wherever l2_cache.v's
// USE_AXI_MEM parameter selects it.
module data_memory_axi #(
    parameter DMEM_FILE  = "data.mem",
    parameter DMEM_WORDS = 4096,
    parameter VLEN = 256,
    // Simulation-only convenience: preload content from a file at
    // elaboration time, same as data_memory.v itself. FPGA builds set
    // this to 0 (content arrives only via the AXI write port below) --
    // $readmemh referencing a file that doesn't exist at synthesis time
    // is exactly the kind of thing that should never even be attempted
    // on a real hardware build.
    parameter LOAD_FROM_FILE = 1
)(
    input clk,

    input vmem_read,   // unused (read is unconditional, matching
                        // data_memory.v's own "also used for partial-
                        // vector-store read-modify-write" behavior) --
                        // kept for interface-shape symmetry with the
                        // caller's existing port list.
    input vmem_write,
    input [63:0] vmem_addr,
    input [VLEN-1:0] vmem_write_data,
    output reg [VLEN-1:0] vmem_read_data,

    // ---- AXI-side program/data-load write port -------------------------
    // Independent of the core's own vmem_write port. The two are only
    // ever meant to be active in disjoint phases (AXI loads the program
    // while the core sits in reset; the core's own vmem_write only fires
    // once running) -- not arbitrated against each other, the same
    // "documented invariant, not defensively handled" style this project
    // uses elsewhere for conditions that can't happen given how the
    // surrounding system is actually driven.
    input axi_wr_en,
    input [$clog2(DMEM_WORDS)-1:0] axi_wr_addr,
    input [63:0] axi_wr_data
);
    localparam VDWORDS = VLEN / 64;

    reg [63:0] memory [0:DMEM_WORDS-1];

    generate
        if (LOAD_FROM_FILE) begin : sim_load
            initial begin
                for (integer i = 0; i < DMEM_WORDS; i = i + 1)
                    memory[i] = 64'b0;
                $readmemh(DMEM_FILE, memory);
            end
        end
    endgenerate

    wire [$clog2(DMEM_WORDS)-1:0] vword_addr = vmem_addr[$clog2(DMEM_WORDS)+2:3];

    integer vi;
    always @(posedge clk) begin
        if (vmem_write)
            for (vi = 0; vi < VDWORDS; vi = vi + 1)
                memory[vword_addr + vi] <= vmem_write_data[vi*64 +: 64];
        if (axi_wr_en)
            memory[axi_wr_addr] <= axi_wr_data;
        // Registered read: address sampled this cycle, result visible
        // next cycle -- reads memory's PRE-this-cycle contents (standard
        // NBA "old value" semantics), matching a real BRAM's read-first
        // behavior on a same-cycle read/write collision. l2_cache.v's
        // ST_MEM_WAIT already accounts for this one-cycle gap.
        for (vi = 0; vi < VDWORDS; vi = vi + 1)
            vmem_read_data[vi*64 +: 64] <= memory[vword_addr + vi];
    end
endmodule
