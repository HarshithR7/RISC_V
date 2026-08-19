`timescale 1ns / 1ps
// Shared L2 cache and MESI coherence director for exactly 2 L1s (see
// l1_cache.v's header for the per-L1 protocol this module drives).
// Direct-mapped, inclusive of both L1s, and write-through to the backing
// data_memory.v -- an L2 line is never itself dirty relative to memory,
// so L2 needs no writeback-to-memory path of its own beyond mirroring
// every update it makes into memory as it makes it. This is what lets L2
// skip tracking M/E/S for its own copy: it only needs a "valid" bit per
// line plus two presence bits (does core0's/core1's L1 currently hold a
// copy) -- a *directory*, not a third MESI level.
//
// The presence directory is a safe over-approximation, not exact
// bookkeeping: l1_cache.v silently evicts a *clean* (S/E) line without
// telling L2 at all (see its own header), so a presence bit can say "yes"
// after the L1 has actually already dropped the line. This is safe --
// every snoop response is a live lookup against the target L1's own real
// tag/state arrays, never against this directory -- it just occasionally
// causes one harmless extra snoop that resolves to a miss. The one case
// where the directory *must* stay accurate is an L2-internal eviction
// (replacing a line to make room for a different address): any L1 still
// holding presence for the *evicted* address must be force-invalidated
// first, or this module's own inclusive-L2 property (an L1 can only ever
// hold what L2 currently tracks) breaks.
//
// One request in flight at a time, fixed priority core 0 over core 1 --
// the same "thread 0 always wins" convention riscv64_ooo_proc.v's own
// Phase 7 cross-thread store-port arbiter already uses, kept for
// consistency, and a deliberately simple (if not maximally fair)
// arbitration choice appropriate for a 2-requester coherence point.
//
// Memory-side access reuses data_memory.v's existing RVV vector port
// unmodified (unit-stride, VLEN-wide, 8-byte-aligned) -- LINE_BYTES*8
// bits, one whole cache line per access -- rather than inventing a new
// wide memory port: Phase 6 never puts any real traffic on that port
// (`vmem_read`/`vmem_write` are permanently tied to 0 in the OoO core,
// since vector load/store was explicitly out of scope there), so it was
// sitting unused and is a natural fit here.
module l2_cache #(
    parameter L2_LINES = 64,
    parameter LINE_BYTES = 32,
    parameter ADDR_BITS = 64,
    parameter DMEM_FILE = "data.mem",
    parameter DMEM_WORDS = 4096
)(
    input clk,
    input reset,

    // ---- core 0's L1 -----------------------------------------------------
    input c0_req_valid,
    input [1:0] c0_req_type,   // 0=BusRd 1=BusRdX 2=Writeback(dirty eviction) 3=BusUpgr
    input [ADDR_BITS-1:0] c0_req_addr,
    input [LINE_BYTES*8-1:0] c0_req_wb_data,
    output reg c0_resp_valid,
    output reg [LINE_BYTES*8-1:0] c0_resp_data,
    output reg c0_resp_exclusive,

    // ---- core 1's L1 -----------------------------------------------------
    input c1_req_valid,
    input [1:0] c1_req_type,
    input [ADDR_BITS-1:0] c1_req_addr,
    input [LINE_BYTES*8-1:0] c1_req_wb_data,
    output reg c1_resp_valid,
    output reg [LINE_BYTES*8-1:0] c1_resp_data,
    output reg c1_resp_exclusive,

    // ---- snoop out to core 0's / core 1's L1 -----------------------------
    // Purely combinational, driven straight off FSM state (see the
    // want_snoop*/assign block below) -- NOT registered. l1_cache.v's own
    // snoop response is itself combinational (a live tag/state lookup),
    // so asserting the request via a non-blocking `<=` here would mean
    // this same clocked block could only ever observe last cycle's (pre-
    // assertion) response when it tries to read it back in the same
    // state -- a same-cycle NBA-vs-combinational-readback mistake this
    // module's development actually made and had to debug (see the
    // isolated tb_cache_mesi.v test that caught it). Driving these
    // outputs combinationally means the moment `fsm` lands on a
    // snoop-needing state, the request and the target L1's response are
    // both valid in that same cycle, so the clocked logic below can
    // safely consume the response without any extra wait state.
    output wire snoop0_req_valid,
    output wire [1:0] snoop0_req_type,
    output wire [ADDR_BITS-1:0] snoop0_req_addr,
    input snoop0_resp_hit,
    input snoop0_resp_dirty,
    input [LINE_BYTES*8-1:0] snoop0_resp_data,

    output wire snoop1_req_valid,
    output wire [1:0] snoop1_req_type,
    output wire [ADDR_BITS-1:0] snoop1_req_addr,
    input snoop1_resp_hit,
    input snoop1_resp_dirty,
    input [LINE_BYTES*8-1:0] snoop1_resp_data,

    // Phase 9 (ECC): protects only l2_data[] (see the ECC section below),
    // not the directory (l2_valid/l2_tag/presence0/presence1) -- same
    // "bulk data, not small control metadata" scope as l1_cache.v.
    output ecc_l2_sbe_fault,
    output ecc_l2_dbe_fault
);
    localparam OFF_BITS = $clog2(LINE_BYTES);
    localparam IDX_BITS = $clog2(L2_LINES);
    localparam TAG_BITS = ADDR_BITS - IDX_BITS - OFF_BITS;
    localparam VLEN = LINE_BYTES * 8;

    localparam REQ_BUSRD = 2'b00, REQ_BUSRDX = 2'b01, REQ_WB = 2'b10, REQ_UPGR = 2'b11;

    reg l2_valid [0:L2_LINES-1];
    reg [TAG_BITS-1:0] l2_tag [0:L2_LINES-1];
    reg [VLEN-1:0] l2_data [0:L2_LINES-1];
    // Phase 9 (ECC): check bits for l2_data[]. LINE_BYTES wide -- see
    // ecc_line.v's header for why that's exactly right (8 check bits per
    // 64-bit word, LINE_BYTES/8 words per line).
    reg [LINE_BYTES-1:0] l2_check [0:L2_LINES-1];
    reg presence0 [0:L2_LINES-1];
    reg presence1 [0:L2_LINES-1];

    integer li;
    initial begin
        for (li = 0; li < L2_LINES; li = li + 1) begin
            l2_valid[li] = 1'b0;
            l2_check[li] = {LINE_BYTES{1'b0}};
            presence0[li] = 1'b0;
            presence1[li] = 1'b0;
        end
    end

    // ---- Backing memory (whole-line access via the reused vmem port) ----
    reg mem_vread, mem_vwrite;
    reg [ADDR_BITS-1:0] mem_vaddr;
    reg [VLEN-1:0] mem_vwdata;
    wire [VLEN-1:0] mem_vrdata;
    data_memory #(.DMEM_FILE(DMEM_FILE), .DMEM_WORDS(DMEM_WORDS), .VLEN(VLEN)) backing_mem (
        .clk(clk),
        .mem_read(1'b0), .mem_write(1'b0), .func3(3'b011), .mem_addr(64'b0), .write_data(64'b0),
        .read_data(),
        .vmem_read(mem_vread), .vmem_write(mem_vwrite), .vmem_addr(mem_vaddr),
        .vmem_write_data(mem_vwdata), .vmem_read_data(mem_vrdata)
    );

    function [IDX_BITS-1:0] idx_of; input [ADDR_BITS-1:0] a; begin idx_of = a[IDX_BITS+OFF_BITS-1:OFF_BITS]; end endfunction
    function [TAG_BITS-1:0] tag_of; input [ADDR_BITS-1:0] a; begin tag_of = a[ADDR_BITS-1:IDX_BITS+OFF_BITS]; end endfunction
    function [ADDR_BITS-1:0] line_addr_of; input [TAG_BITS-1:0] t; input [IDX_BITS-1:0] i;
        begin line_addr_of = {t, i, {OFF_BITS{1'b0}}}; end
    endfunction

    localparam ST_IDLE        = 0,
               ST_EVICT_SNOOP = 1,  // force-invalidate old occupant's presence before reuse
               ST_EVICT_MEMWR = 2,  // flush recovered dirty data from the forced invalidate
               ST_MEM_FETCH   = 3,  // L2 miss: pull the new line in from memory
               ST_MEM_WAIT    = 4,
               ST_SNOOP_OTHER = 5,  // normal coherence snoop of the non-requesting core
               ST_RESPOND     = 6,
               ST_COOLDOWN    = 7;  // see its own comment below
    reg [2:0] fsm;
    reg req_core;                    // 0 or 1: which core this transaction serves
    reg [1:0] req_type_r;
    reg [ADDR_BITS-1:0] req_addr_r;
    reg [LINE_BYTES*8-1:0] req_wbdata_r;
    reg other_had_dirty;
    reg [LINE_BYTES*8-1:0] other_dirty_data;
    reg need_mem_fetch;

    wire [IDX_BITS-1:0] req_idx = idx_of(req_addr_r);
    wire [TAG_BITS-1:0] req_tag = tag_of(req_addr_r);
    wire l2_hit = l2_valid[req_idx] && (l2_tag[req_idx] == req_tag);
    wire other_presence = req_core ? presence0[req_idx] : presence1[req_idx];

    // ---- Combinational snoop assertion -- see the port comment above --
    wire evict_needed = (fsm == ST_EVICT_SNOOP) && (req_type_r != REQ_WB) &&
                         l2_valid[req_idx] && (l2_tag[req_idx] != req_tag);
    wire evict_want0 = evict_needed && presence0[req_idx];
    wire evict_want1 = evict_needed && presence1[req_idx];
    wire normal_snoop = (fsm == ST_SNOOP_OTHER) && other_presence;
    wire normal_want0 = normal_snoop && req_core;   // requester is core1 -> snoop core0
    wire normal_want1 = normal_snoop && !req_core;  // requester is core0 -> snoop core1
    wire [1:0] normal_type = (req_type_r == REQ_BUSRD) ? REQ_BUSRD : REQ_BUSRDX;

    assign snoop0_req_valid = evict_want0 || normal_want0;
    assign snoop0_req_type  = evict_want0 ? REQ_BUSRDX : normal_type;
    assign snoop0_req_addr  = evict_want0 ? line_addr_of(l2_tag[req_idx], req_idx) : req_addr_r;

    assign snoop1_req_valid = evict_want1 || normal_want1;
    assign snoop1_req_type  = evict_want1 ? REQ_BUSRDX : normal_type;
    assign snoop1_req_addr  = evict_want1 ? line_addr_of(l2_tag[req_idx], req_idx) : req_addr_r;

    wire evict_other_dirty = (evict_want0 && snoop0_resp_hit && snoop0_resp_dirty) ||
                              (evict_want1 && snoop1_resp_hit && snoop1_resp_dirty);
    wire [LINE_BYTES*8-1:0] evict_other_data = evict_want0 ? snoop0_resp_data : snoop1_resp_data;
    wire normal_other_dirty = normal_snoop && (req_core ? (snoop0_resp_hit && snoop0_resp_dirty)
                                                          : (snoop1_resp_hit && snoop1_resp_dirty));
    wire [LINE_BYTES*8-1:0] normal_other_data = req_core ? snoop0_resp_data : snoop1_resp_data;

    // ---- Phase 9 (ECC): one shared decode/encode instance, keyed on
    // req_idx -- unlike l1_cache.v, only one L2 transaction is ever in
    // flight at a time (a single FSM, no independent concurrent access
    // points), so req_idx is the only index that ever matters and stays
    // stable (derived from the latched req_addr_r, not a live request
    // line) for the whole transaction. Still needs the same registered-
    // latching treatment l1_cache.v's fault flags needed, and for the
    // same underlying reason: c0_resp_valid/c1_resp_valid are themselves
    // `<=`-assigned *inside* the ST_RESPOND state's own body, so they
    // only become visible the cycle *after* fsm actually holds
    // ST_RESPOND (once it's already moved on to ST_COOLDOWN) -- a
    // combinational `fsm == ST_RESPOND` gate would therefore be one cycle
    // early relative to when a caller watching resp_valid actually
    // samples it (found via tb_ecc_l2.v initially reporting sbe=0 on a
    // real corruption, the same way tb_ecc_l1.v caught L1's version of
    // this). Fixed by latching access_sbe/access_dbe in that same
    // ST_RESPOND body, alongside c0_resp_valid/c1_resp_valid themselves.
    //
    // None of L2's 3 write sites (a requester's own dirty writeback, a
    // memory fetch, or an absorbed dirty snoop-forward) are a
    // read-modify-write of L2's own existing data -- L2 always replaces
    // the whole line, never merges a partial store the way l1_cache.v's
    // CPU-facing side does -- so, unlike l1_cache.v's wr_write_value,
    // l2_write_value never depends on l2_line_corrected; it's a plain
    // FSM-state mux over the 3 fresh incoming values.
    wire [LINE_BYTES*8-1:0] l2_write_value =
        (fsm == ST_MEM_WAIT)    ? mem_vrdata :
        (fsm == ST_SNOOP_OTHER) ? normal_other_data :
                                   req_wbdata_r; // ST_EVICT_SNOOP (REQ_WB)
    wire [LINE_BYTES-1:0] l2_write_check;
    wire [LINE_BYTES*8-1:0] l2_line_corrected;
    wire l2_line_sbe, l2_line_dbe;
    ecc_line #(.LINE_BYTES(LINE_BYTES)) ecc_l2_i (
        .wr_line(l2_write_value), .wr_check(l2_write_check),
        .rd_line(l2_data[req_idx]), .rd_check(l2_check[req_idx]),
        .rd_line_corrected(l2_line_corrected), .rd_sbe(l2_line_sbe), .rd_dbe(l2_line_dbe)
    );
    reg access_sbe, access_dbe;
    assign ecc_l2_sbe_fault = access_sbe;
    assign ecc_l2_dbe_fault = access_dbe;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            fsm <= ST_IDLE;
            mem_vread <= 1'b0; mem_vwrite <= 1'b0;
            c0_resp_valid <= 1'b0; c1_resp_valid <= 1'b0;
            access_sbe <= 1'b0; access_dbe <= 1'b0;
            for (li = 0; li < L2_LINES; li = li + 1) begin
                l2_valid[li] <= 1'b0;
                l2_check[li] <= {LINE_BYTES{1'b0}};
                presence0[li] <= 1'b0;
                presence1[li] <= 1'b0;
            end
        end else begin
            c0_resp_valid <= 1'b0;
            c1_resp_valid <= 1'b0;
            mem_vread <= 1'b0;
            mem_vwrite <= 1'b0;

            case (fsm)
                ST_IDLE: begin
                    if (c0_req_valid) begin
                        req_core <= 1'b0;
                        req_type_r <= c0_req_type; req_addr_r <= c0_req_addr; req_wbdata_r <= c0_req_wb_data;
                        fsm <= ST_EVICT_SNOOP;
                    end else if (c1_req_valid) begin
                        req_core <= 1'b1;
                        req_type_r <= c1_req_type; req_addr_r <= c1_req_addr; req_wbdata_r <= c1_req_wb_data;
                        fsm <= ST_EVICT_SNOOP;
                    end
                end

                // A dirty (M) eviction writeback from the requester's own
                // L1: just absorb it into L2 (write-through), no coherence
                // action needed (the requester's own line, nobody else can
                // hold M simultaneously by MESI's own invariant).
                //
                // Otherwise, if this L2 slot currently holds a *different*
                // valid address than the one now needed (inclusive-L2
                // replacement), evict_needed/evict_want0/evict_want1 above
                // have already combinationally asserted whichever snoop(s)
                // are needed to force-invalidate any L1 presence for the
                // *old* address this same cycle -- their responses are
                // valid right now, so just consume them and clear the
                // stale presence bits.
                ST_EVICT_SNOOP: begin
                    if (req_type_r == REQ_WB) begin
                        l2_valid[req_idx] <= 1'b1;
                        l2_tag[req_idx]   <= req_tag;
                        l2_data[req_idx]  <= l2_write_value;
                        l2_check[req_idx] <= l2_write_check;
                        mem_vwrite <= 1'b1; mem_vaddr <= line_addr_of(req_tag, req_idx); mem_vwdata <= req_wbdata_r;
                        if (req_core) presence1[req_idx] <= 1'b1; else presence0[req_idx] <= 1'b1;
                        fsm <= ST_RESPOND;
                    end
                    else if (evict_needed && (presence0[req_idx] || presence1[req_idx])) begin
                        presence0[req_idx] <= 1'b0;
                        presence1[req_idx] <= 1'b0;
                        other_had_dirty  <= evict_other_dirty;
                        other_dirty_data <= evict_other_data;
                        fsm <= ST_EVICT_MEMWR;
                    end
                    else begin
                        need_mem_fetch <= !l2_hit;
                        fsm <= l2_hit ? ST_SNOOP_OTHER : ST_MEM_FETCH;
                    end
                end

                // Flush any dirty data recovered from the forced
                // replacement-invalidate above into memory before
                // proceeding (this L2 slot's *old* address's last known
                // value must not be lost).
                ST_EVICT_MEMWR: begin
                    if (other_had_dirty) begin
                        mem_vwrite <= 1'b1;
                        mem_vaddr  <= line_addr_of(l2_tag[req_idx], req_idx);
                        mem_vwdata <= other_dirty_data;
                    end
                    need_mem_fetch <= 1'b1;
                    fsm <= ST_MEM_FETCH;
                end

                ST_MEM_FETCH: begin
                    if (need_mem_fetch) begin
                        mem_vread <= 1'b1; mem_vaddr <= line_addr_of(req_tag, req_idx);
                        fsm <= ST_MEM_WAIT;
                    end else begin
                        fsm <= ST_SNOOP_OTHER;
                    end
                end

                ST_MEM_WAIT: begin
                    // data_memory.v's vector read port is combinational
                    // (valid the same cycle vmem_read is asserted -- see
                    // its own header), so mem_vrdata is already settled by
                    // the time this state is reached; one cycle here just
                    // matches the read-request/consume-result convention
                    // used throughout this pair of modules.
                    l2_valid[req_idx] <= 1'b1;
                    l2_tag[req_idx]   <= req_tag;
                    l2_data[req_idx]  <= l2_write_value;
                    l2_check[req_idx] <= l2_write_check;
                    fsm <= ST_SNOOP_OTHER;
                end

                // normal_want0/normal_want1 above have already
                // combinationally asserted the real coherence snoop (if
                // needed) this same cycle -- consume the (already valid)
                // response directly.
                ST_SNOOP_OTHER: begin
                    if (normal_snoop && (req_type_r != REQ_BUSRD)) begin
                        if (req_core) presence0[req_idx] <= 1'b0; else presence1[req_idx] <= 1'b0;
                    end
                    if (req_core) presence1[req_idx] <= 1'b1; else presence0[req_idx] <= 1'b1;

                    if (normal_other_dirty) begin
                        // The other core's copy was more current than
                        // whatever L2 already had -- absorb it (into both
                        // L2's own array and memory, keeping the write-
                        // through invariant intact) before responding.
                        // Note the *line-aligned* address here, not
                        // req_addr_r directly -- a CPU-level BusRd/BusRdX
                        // request carries the real byte address (possibly
                        // with a non-zero in-line offset), and the vector
                        // memory port needs the line's base address.
                        l2_data[req_idx] <= l2_write_value;
                        l2_check[req_idx] <= l2_write_check;
                        mem_vwrite <= 1'b1;
                        mem_vaddr  <= line_addr_of(req_tag, req_idx);
                        mem_vwdata <= normal_other_data;
                    end
                    fsm <= ST_RESPOND;
                end

                // A plain writeback (REQ_WB, an M-dirty eviction) just needs
                // an ack pulse -- l1_cache.v's ST_EVICT_WB doesn't read
                // resp_data/resp_exclusive for it (see l1_cache.v's header),
                // it only waits for l2_resp_valid before issuing the *real*
                // request for the line it actually wanted.
                ST_RESPOND: begin
                    if (req_core) begin
                        c1_resp_valid <= 1'b1;
                        c1_resp_data <= l2_line_corrected;
                        c1_resp_exclusive <= !presence0[req_idx];
                    end else begin
                        c0_resp_valid <= 1'b1;
                        c0_resp_data <= l2_line_corrected;
                        c0_resp_exclusive <= !presence1[req_idx];
                    end
                    access_sbe <= l2_line_sbe;
                    access_dbe <= l2_line_dbe;
                    fsm <= ST_COOLDOWN;
                end

                // The requester's own l2_req_valid clears one cycle after
                // it sees this response (see l1_cache.v: it reacts to
                // l2_resp_valid with a non-blocking clear, so the clear
                // isn't visible until the cycle *after* the response).
                // Returning straight from ST_RESPOND to ST_IDLE would let
                // this same requester's now-stale, not-yet-cleared
                // req_valid be sampled as if it were a brand new request
                // on that exact edge -- a real race this module's
                // development actually hit (an isolated coherency test
                // caught it: a spurious extra transaction replaying the
                // just-completed one, confusing a later real request).
                // One unconditional cooldown cycle here guarantees the
                // requester's clear has already landed by the time
                // ST_IDLE looks again.
                ST_COOLDOWN: fsm <= ST_IDLE;

                default: fsm <= ST_IDLE;
            endcase
        end
    end
endmodule
