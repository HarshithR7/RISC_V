`timescale 1ns / 1ps
// Phase 17: per-thread, read-only instruction cache. Direct-mapped, one
// instance per thread (replacing that thread's 3x instruction_fetch_reg
// instances -- see riscv64_ooo_proc.v), serving all 3 fetch-group
// addresses (pc/pc1/pc2) from one shared tag/valid/data structure via
// combinational compares, same "multiple read ports into one array is
// trivial for a read-only structure" instinct l1_cache.v's own
// cpu_read2_* port already established.
//
// Private-per-thread, no coherency: unlike the data path's genuinely
// shared, MESI-coherent l2_cache.v, this project's instruction memory is
// *already* per-thread-private (IMEM_FILE0/IMEM_FILE1 are literally
// different program images per thread in every test) -- so this cache
// needs no snoop port, no L2 integration, nothing shared across threads
// or cores. No self-modifying-code protection exists anywhere in this
// codebase, and none is added here either -- a documented scope cut, the
// same convention l1_cache.v's own single primary port already uses.
//
// Hit latency is exactly 1 cycle, matching today's instruction_fetch_reg
// timing exactly, so riscv64_ooo_proc.v's existing Phase 11
// t0_pc_latched/t0_fetch_valid plumbing (built assuming fetch is always
// 1-cycle) needs zero changes for the hit case -- only a miss needs new
// stall handling, via the new `stall` output below, registered on the
// same edge as t0_fetch_valid so it describes "was the fetch that's now
// visible actually valid," the identical convention.
//
// Backing store: rather than reinventing a $readmemh-loaded array (which
// would silently drop Phase 12's AXI-loadable capability), a miss fills
// a line by instantiating LINE_BYTES/4 parallel instruction_fetch_reg
// sub-instances, each reading a different fixed word offset within the
// target line -- the exact same "N parallel instances = N read ports"
// trick riscv64_ooo_proc.v already uses for its 3 fetch lanes, just
// scaled up to cover one whole line at once. This preserves
// USE_AXI_MEM=1 for free (axi_wr_* just fans out to all of them, same as
// today's 3-way fan-out), with zero changes needed to
// instruction_fetch_axi.v/axi_ooo_ctrl.v/fpga_top.v. MISS_LATENCY is a
// separate, purely artificial wait-cycle count layered on top, whose
// only job is to make a miss genuinely cost more than a hit -- the
// backing store itself is otherwise instant, so without this a "cache"
// would have nothing real to cache.
//
// Line-boundary crossing: the 3-wide fetch group spans up to 12 bytes
// (pc..pc+8+3); with the default 32-byte line, roughly a third of
// alignments spill pc2 into the next line. v1 policy is conservative: ALL
// lanes stall together until *every* line the group needs is resident --
// no partial-width degraded fetch. Only line(pc) and line(pc2) are ever
// checked; line(pc1) is always provably covered by one of those two (pc1
// can only leave line(pc) when pc's in-line offset is exactly
// LINE_BYTES-4, the last word -- and at that offset pc2's offset is 4
// bytes into the very same next line pc1 just crossed into).
//
// No abort input: like div_fu.v (see div_rs.v's identical precedent), a
// fill in progress can't be cancelled. A misprediction/JALR-redirect can
// legitimately arrive while a fill for an already-abandoned speculative-
// path line is still running (redirects are asynchronous to fetch, same
// as ever) -- t0_redirect_needed still wins riscv64_ooo_proc.v's
// t0_next_pc mux regardless of icache_stall, so the PC itself redirects
// immediately, but this module has no way to abandon the in-flight fill
// early; it simply finishes (harmlessly, since nothing depends on that
// specific fill completing) before the *next* miss -- for the redirected
// target's own line, if it's not already resident -- can even be
// detected, let alone start. Not a correctness gap, just a documented
// latency interaction.
module icache #(
    parameter IMEM_FILE = "instructions.mem",
    parameter IMEM_WORDS = 8192,
    parameter USE_AXI_MEM = 0,
    parameter LINES = 16,
    parameter LINE_BYTES = 32,
    parameter MISS_LATENCY = 4
)(
    input clk,
    input reset,

    input [63:0] pc,
    input [63:0] pc1,
    input [63:0] pc2,
    output reg [31:0] instruction0,
    output reg [31:0] instruction1,
    output reg [31:0] instruction2,
    output reg stall,

    input axi_wr_en,
    input [$clog2(IMEM_WORDS)-1:0] axi_wr_addr,
    input [15:0] axi_wr_data
);
    localparam OFF_BITS = $clog2(LINE_BYTES);
    localparam IDX_BITS = $clog2(LINES);
    localparam TAG_BITS = 64 - IDX_BITS - OFF_BITS;
    localparam WORDS_PER_LINE = LINE_BYTES / 4;

    reg valid [0:LINES-1];
    reg [TAG_BITS-1:0] tag [0:LINES-1];
    reg [LINE_BYTES*8-1:0] line [0:LINES-1];

    function [31:0] extract_word;
        input [LINE_BYTES*8-1:0] ld_line;
        input [63:0] addr;
        reg [OFF_BITS-1:0] off;
        begin
            off = addr[OFF_BITS-1:0];
            extract_word = ld_line[{off[OFF_BITS-1:2], 2'b00}*8 +: 32];
        end
    endfunction

    // ---- Combinational hit check (line(pc) and line(pc2) only -- see
    // header for why line(pc1) never needs its own check) ----------------
    wire [IDX_BITS-1:0] idx_pc  = pc[IDX_BITS+OFF_BITS-1:OFF_BITS];
    wire [TAG_BITS-1:0] tag_pc  = pc[63:IDX_BITS+OFF_BITS];
    wire hit_pc  = valid[idx_pc] && (tag[idx_pc] == tag_pc);

    wire [IDX_BITS-1:0] idx_pc2 = pc2[IDX_BITS+OFF_BITS-1:OFF_BITS];
    wire [TAG_BITS-1:0] tag_pc2 = pc2[63:IDX_BITS+OFF_BITS];
    wire hit_pc2 = valid[idx_pc2] && (tag[idx_pc2] == tag_pc2);

    wire same_line  = (pc[63:OFF_BITS] == pc2[63:OFF_BITS]);
    wire pc1_is_a    = (pc1[63:OFF_BITS] == pc[63:OFF_BITS]);
    wire group_hit  = hit_pc && (same_line || hit_pc2);

    wire [31:0] instr_pc_comb  = extract_word(line[idx_pc], pc);
    wire [31:0] instr_pc1_comb = extract_word(pc1_is_a ? line[idx_pc] : line[idx_pc2], pc1);
    wire [31:0] instr_pc2_comb = extract_word(same_line ? line[idx_pc] : line[idx_pc2], pc2);

    // ---- Fill: LINE_BYTES/4 parallel instruction_fetch_reg "read ports,"
    // retargeted at a line's base address for the duration of a fill --
    // see header for the backing-store-reuse rationale. -------------------
    reg [63:0] fill_base;
    wire [LINE_BYTES*8-1:0] filled_line_flat;
    genvar gi;
    generate
        for (gi = 0; gi < WORDS_PER_LINE; gi = gi + 1) begin : fillers
            wire [31:0] w;
            wire [63:0] word_addr = fill_base + gi * 4;
            instruction_fetch_reg #(.IMEM_FILE(IMEM_FILE), .IMEM_WORDS(IMEM_WORDS), .USE_AXI_MEM(USE_AXI_MEM)) filler_i (
                .clk(clk), .pc(word_addr), .instruction(w),
                .axi_wr_en(axi_wr_en), .axi_wr_addr(axi_wr_addr), .axi_wr_data(axi_wr_data)
            );
            assign filled_line_flat[gi*32 +: 32] = w;
        end
    endgenerate

    localparam ST_IDLE = 1'b0, ST_FILL_WAIT = 1'b1;
    reg fsm;
    reg [IDX_BITS-1:0] target_idx;
    reg [TAG_BITS-1:0] target_tag;
    reg pending_b;
    reg [7:0] wait_cnt;

    integer li;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            fsm <= ST_IDLE;
            stall <= 1'b0;
            instruction0 <= 32'b0;
            instruction1 <= 32'b0;
            instruction2 <= 32'b0;
            for (li = 0; li < LINES; li = li + 1)
                valid[li] <= 1'b0;
        end else begin
            // Output registration: same "describes the fetch that's now
            // visible, one cycle after the address was presented" timing
            // as t0_pc_latched/t0_fetch_valid -- see header. Purely a
            // function of pc/pc2 and the (possibly still-filling)
            // valid[]/tag[] arrays, so it self-corrects the instant a fill
            // completes, no explicit fsm-state gating needed here.
            stall <= !group_hit;
            if (group_hit) begin
                instruction0 <= instr_pc_comb;
                instruction1 <= instr_pc1_comb;
                instruction2 <= instr_pc2_comb;
            end

            case (fsm)
                ST_IDLE: begin
                    if (!group_hit) begin
                        if (!hit_pc) begin
                            fill_base  <= {pc[63:OFF_BITS], {OFF_BITS{1'b0}}};
                            target_idx <= idx_pc;
                            target_tag <= tag_pc;
                            pending_b  <= !same_line && !hit_pc2;
                        end else begin
                            // hit_pc true but group_hit false means line(pc2)
                            // must be the missing one (see header).
                            fill_base  <= {pc2[63:OFF_BITS], {OFF_BITS{1'b0}}};
                            target_idx <= idx_pc2;
                            target_tag <= tag_pc2;
                            pending_b  <= 1'b0;
                        end
                        wait_cnt <= MISS_LATENCY + 1;
                        fsm <= ST_FILL_WAIT;
                    end
                end

                ST_FILL_WAIT: begin
                    if (wait_cnt != 0) begin
                        wait_cnt <= wait_cnt - 1'b1;
                    end else begin
                        valid[target_idx] <= 1'b1;
                        tag[target_idx]   <= target_tag;
                        line[target_idx]  <= filled_line_flat;
                        if (pending_b) begin
                            fill_base  <= {pc2[63:OFF_BITS], {OFF_BITS{1'b0}}};
                            target_idx <= idx_pc2;
                            target_tag <= tag_pc2;
                            pending_b  <= 1'b0;
                            wait_cnt   <= MISS_LATENCY + 1;
                            // stays in ST_FILL_WAIT for the 2nd line
                        end else begin
                            fsm <= ST_IDLE;
                        end
                    end
                end

                default: fsm <= ST_IDLE;
            endcase
        end
    end
endmodule
