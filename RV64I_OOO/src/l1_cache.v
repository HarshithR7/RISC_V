`timescale 1ns / 1ps
// Private per-core L1 data cache with a real MESI coherency state machine.
// Direct-mapped (one way per set -- same "correct by construction, not a
// knob worth exposing" instinct as this project's other fixed-depth
// structures, e.g. branch_rs.v/div_rs.v), single outstanding miss at a
// time (blocking, non-pipelined -- matches div_fu.v's own "real, multi-
// cycle, but not pipelined" scope).
//
// Every line carries a 2-bit MESI state (I/S/E/M). This L1 is write-back
// (a dirty M line is only written to L2 on eviction or when snooped by
// the other core) -- l2_cache.v itself is write-through to the backing
// data_memory.v, so L2 never needs its own eviction/writeback path to
// memory, only a directory of which core(s) currently hold each line.
//
// Protocol with l2_cache.v (see its own header for the full picture):
//   - CPU miss: hold l2_req_valid/type/addr stable until l2_resp_valid
//     pulses (one cycle) with the fetched line and whether to install as
//     E (no other core has it) or S (the other core still shares it).
//   - Snoop: l2_cache.v asks this L1, combinationally, "do you have
//     address X, and is it dirty" (snoop_req_valid/type/addr) while
//     servicing the OTHER core's request; this L1 answers the same cycle
//     (snoop_resp_hit/dirty/data are plain combinational reads of this
//     L1's own tag/state arrays) and, on the same clock edge, applies the
//     resulting downgrade (BusRd-type snoop: M/E -> S) or invalidate
//     (BusRdX/Upgr-type snoop: any state -> I). A snoop response can
//     never block on anything (it's a pure lookup), so it's always safe
//     to answer even while this L1 has its own request outstanding to
//     l2_cache.v -- see l2_cache.v's header for why this can't deadlock.
//
// A miss's old occupant (if valid) is evicted -- written back to L2 first
// if dirty -- *before* this L1 ever asks L2 for the new line, so the
// affected set is always genuinely Invalid for the whole time a request
// is outstanding. This is what keeps a same-index snoop arriving during
// that window trivially correct (it just reports "no valid copy") without
// this module needing to reason about a new, not-yet-installed line being
// snoop-visible before its own fill completes -- a deliberate, documented
// simplification, not an oversight.
module l1_cache #(
    parameter LINES = 16,       // direct-mapped: also the number of sets
    parameter LINE_BYTES = 32,  // 4 doublewords per line
    parameter ADDR_BITS = 64
)(
    input clk,
    input reset,

    // ---- CPU-side load port (from lsq.v's load-issue path) ------------
    input cpu_read_req,
    input [ADDR_BITS-1:0] cpu_read_addr,
    input [2:0] cpu_read_func3,
    output cpu_read_valid,      // pulses exactly once per accepted request
    output [63:0] cpu_read_data,

    // ---- CPU-side store-buffer drain port ------------------------------
    input cpu_write_req,
    input [ADDR_BITS-1:0] cpu_write_addr,
    input [63:0] cpu_write_data,
    input [2:0] cpu_write_func3,
    output cpu_write_done,      // pulses once the write has landed in M state

    output busy,                 // one outstanding CPU request at a time

    // ---- L2-side request/response --------------------------------------
    output reg l2_req_valid,
    output reg [1:0] l2_req_type,   // 0=BusRd 1=BusRdX 2=Writeback(eviction) 3=BusUpgr
    output reg [ADDR_BITS-1:0] l2_req_addr,
    output reg [LINE_BYTES*8-1:0] l2_req_wb_data,
    input l2_resp_valid,
    input [LINE_BYTES*8-1:0] l2_resp_data,
    input l2_resp_exclusive,   // 1 = install as E, 0 = install as S (BusRd only)

    // ---- snoop (l2_cache.v forwarding the OTHER core's request) -------
    input snoop_req_valid,
    input [1:0] snoop_req_type,   // 0=BusRd (may downgrade), 1=BusRdX/Upgr (must invalidate)
    input [ADDR_BITS-1:0] snoop_req_addr,
    output snoop_resp_hit,
    output snoop_resp_dirty,
    output [LINE_BYTES*8-1:0] snoop_resp_data
);
    localparam OFF_BITS = $clog2(LINE_BYTES);
    localparam IDX_BITS = $clog2(LINES);
    localparam TAG_BITS = ADDR_BITS - IDX_BITS - OFF_BITS;
    localparam DWORDS_PER_LINE = LINE_BYTES / 8;

    localparam I = 2'b00, S = 2'b01, E = 2'b10, M = 2'b11;
    localparam REQ_BUSRD = 2'b00, REQ_BUSRDX = 2'b01, REQ_WB = 2'b10, REQ_UPGR = 2'b11;

    reg [1:0] state [0:LINES-1];
    reg [TAG_BITS-1:0] tag [0:LINES-1];
    reg [LINE_BYTES*8-1:0] line [0:LINES-1];

    integer li;
    initial begin
        for (li = 0; li < LINES; li = li + 1)
            state[li] = I;
    end

    // ---- Same func3 sub-word extract/merge semantics as data_memory.v -
    localparam LB = 3'b000, LH = 3'b001, LW = 3'b010, LD = 3'b011,
               LBU = 3'b100, LHU = 3'b101, LWU = 3'b110;

    function [LINE_BYTES*8-1:0] merge_write;
        input [LINE_BYTES*8-1:0] old_line;
        input [ADDR_BITS-1:0] addr;
        input [63:0] wdata;
        input [2:0] func3;
        reg [OFF_BITS-1:0] off;
        reg [LINE_BYTES*8-1:0] result;
        begin
            off = addr[OFF_BITS-1:0];
            result = old_line;
            case (func3)
                3'b000: result[off*8 +: 8]   = wdata[7:0];    // SB
                3'b001: result[{off[OFF_BITS-1:1],1'b0}*8 +: 16] = wdata[15:0]; // SH
                3'b010: result[{off[OFF_BITS-1:2],2'b00}*8 +: 32] = wdata[31:0]; // SW
                default: result[{off[OFF_BITS-1:3],3'b000}*8 +: 64] = wdata;    // SD
            endcase
            merge_write = result;
        end
    endfunction

    function [63:0] extract_read;
        input [LINE_BYTES*8-1:0] ld_line;
        input [ADDR_BITS-1:0] addr;
        input [2:0] func3;
        reg [OFF_BITS-1:0] off;
        reg [7:0] bval; reg [15:0] hval; reg [31:0] wval; reg [63:0] dval;
        begin
            off = addr[OFF_BITS-1:0];
            bval = ld_line[off*8 +: 8];
            hval = ld_line[{off[OFF_BITS-1:1],1'b0}*8 +: 16];
            wval = ld_line[{off[OFF_BITS-1:2],2'b00}*8 +: 32];
            dval = ld_line[{off[OFF_BITS-1:3],3'b000}*8 +: 64];
            case (func3)
                LB:  extract_read = {{56{bval[7]}}, bval};
                LH:  extract_read = {{48{hval[15]}}, hval};
                LW:  extract_read = {{32{wval[31]}}, wval};
                LBU: extract_read = {56'b0, bval};
                LHU: extract_read = {48'b0, hval};
                LWU: extract_read = {32'b0, wval};
                default: extract_read = dval; // LD
            endcase
        end
    endfunction

    // ---- Snoop: pure combinational lookup ------------------------------
    wire [IDX_BITS-1:0] snoop_idx = snoop_req_addr[IDX_BITS+OFF_BITS-1:OFF_BITS];
    wire [TAG_BITS-1:0] snoop_tag = snoop_req_addr[ADDR_BITS-1:IDX_BITS+OFF_BITS];
    assign snoop_resp_hit   = snoop_req_valid && (state[snoop_idx] != I) && (tag[snoop_idx] == snoop_tag);
    assign snoop_resp_dirty = snoop_resp_hit && (state[snoop_idx] == M);
    assign snoop_resp_data  = line[snoop_idx];

    // ---- CPU-facing FSM --------------------------------------------------
    localparam ST_IDLE = 0, ST_EVICT_WB = 1, ST_REQ = 2, ST_WAIT = 3, ST_DONE_R = 4, ST_DONE_W = 5;
    reg [2:0] fsm;
    reg is_write_op;
    reg [ADDR_BITS-1:0] op_addr;
    reg [63:0] op_wdata;
    reg [2:0] op_func3;
    reg [IDX_BITS-1:0] op_idx;
    reg [TAG_BITS-1:0] op_tag;

    assign busy = (fsm != ST_IDLE);
    assign cpu_read_valid = (fsm == ST_DONE_R);
    assign cpu_write_done = (fsm == ST_DONE_W);

    reg [63:0] read_data_r;
    assign cpu_read_data = read_data_r;

    wire [IDX_BITS-1:0] rd_idx = cpu_read_addr[IDX_BITS+OFF_BITS-1:OFF_BITS];
    wire [TAG_BITS-1:0] rd_tag = cpu_read_addr[ADDR_BITS-1:IDX_BITS+OFF_BITS];
    wire rd_hit = (state[rd_idx] != I) && (tag[rd_idx] == rd_tag);

    wire [IDX_BITS-1:0] wr_idx = cpu_write_addr[IDX_BITS+OFF_BITS-1:OFF_BITS];
    wire [TAG_BITS-1:0] wr_tag = cpu_write_addr[ADDR_BITS-1:IDX_BITS+OFF_BITS];
    wire wr_hit_m_or_e = (state[wr_idx] != I) && (tag[wr_idx] == wr_tag) &&
                          (state[wr_idx] == M || state[wr_idx] == E);
    wire wr_hit_s      = (state[wr_idx] != I) && (tag[wr_idx] == wr_tag) && (state[wr_idx] == S);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            fsm <= ST_IDLE;
            l2_req_valid <= 1'b0;
            for (li = 0; li < LINES; li = li + 1)
                state[li] <= I;
        end else begin
            // Snoop-driven state update -- applies every cycle regardless
            // of this L1's own FSM state (see module header for why this
            // is always safe).
            if (snoop_req_valid && snoop_resp_hit) begin
                if (snoop_req_type == 1'b1)
                    state[snoop_idx] <= I;
                else if (state[snoop_idx] == M || state[snoop_idx] == E)
                    state[snoop_idx] <= S;
            end

            case (fsm)
                ST_IDLE: begin
                    if (cpu_read_req && rd_hit) begin
                        read_data_r <= extract_read(line[rd_idx], cpu_read_addr, cpu_read_func3);
                        fsm <= ST_DONE_R;
                    end else if (cpu_read_req && !rd_hit) begin
                        is_write_op <= 1'b0;
                        op_addr <= cpu_read_addr; op_func3 <= cpu_read_func3;
                        op_idx <= rd_idx; op_tag <= rd_tag;
                        if (state[rd_idx] == M) begin
                            l2_req_valid <= 1'b1; l2_req_type <= REQ_WB;
                            l2_req_addr <= {tag[rd_idx], rd_idx, {OFF_BITS{1'b0}}};
                            l2_req_wb_data <= line[rd_idx];
                            state[rd_idx] <= I;
                            fsm <= ST_EVICT_WB;
                        end else begin
                            state[rd_idx] <= I;
                            l2_req_valid <= 1'b1; l2_req_type <= REQ_BUSRD;
                            l2_req_addr <= cpu_read_addr;
                            fsm <= ST_WAIT;
                        end
                    end else if (cpu_write_req && wr_hit_m_or_e) begin
                        line[wr_idx] <= merge_write(line[wr_idx], cpu_write_addr, cpu_write_data, cpu_write_func3);
                        state[wr_idx] <= M;
                        fsm <= ST_DONE_W;
                    end else if (cpu_write_req && wr_hit_s) begin
                        // Already shared and clean: just need to invalidate
                        // the other core's copy (BusUpgr), no data transfer
                        // needed since this L1's own copy is already current.
                        is_write_op <= 1'b1;
                        op_addr <= cpu_write_addr; op_wdata <= cpu_write_data; op_func3 <= cpu_write_func3;
                        op_idx <= wr_idx; op_tag <= wr_tag;
                        l2_req_valid <= 1'b1; l2_req_type <= REQ_UPGR;
                        l2_req_addr <= cpu_write_addr;
                        fsm <= ST_WAIT;
                    end else if (cpu_write_req && !wr_hit_m_or_e && !wr_hit_s) begin
                        is_write_op <= 1'b1;
                        op_addr <= cpu_write_addr; op_wdata <= cpu_write_data; op_func3 <= cpu_write_func3;
                        op_idx <= wr_idx; op_tag <= wr_tag;
                        if (state[wr_idx] == M) begin
                            l2_req_valid <= 1'b1; l2_req_type <= REQ_WB;
                            l2_req_addr <= {tag[wr_idx], wr_idx, {OFF_BITS{1'b0}}};
                            l2_req_wb_data <= line[wr_idx];
                            state[wr_idx] <= I;
                            fsm <= ST_EVICT_WB;
                        end else begin
                            state[wr_idx] <= I;
                            l2_req_valid <= 1'b1; l2_req_type <= REQ_BUSRDX;
                            l2_req_addr <= cpu_write_addr;
                            fsm <= ST_WAIT;
                        end
                    end
                end

                // Eviction writeback acknowledged -- now issue the real
                // request for the line the CPU actually wanted.
                ST_EVICT_WB: begin
                    if (l2_resp_valid) begin
                        l2_req_valid <= 1'b1;
                        l2_req_type <= is_write_op ? REQ_BUSRDX : REQ_BUSRD;
                        l2_req_addr <= op_addr;
                        fsm <= ST_WAIT;
                    end
                end

                ST_WAIT: begin
                    if (l2_resp_valid) begin
                        l2_req_valid <= 1'b0;
                        if (l2_req_type == REQ_UPGR) begin
                            // Already had the data (was S); just needed the
                            // other core invalidated. Apply the write now.
                            line[op_idx] <= merge_write(line[op_idx], op_addr, op_wdata, op_func3);
                            state[op_idx] <= M;
                            fsm <= ST_DONE_W;
                        end else if (is_write_op) begin
                            tag[op_idx] <= op_tag;
                            line[op_idx] <= merge_write(l2_resp_data, op_addr, op_wdata, op_func3);
                            state[op_idx] <= M;
                            fsm <= ST_DONE_W;
                        end else begin
                            tag[op_idx] <= op_tag;
                            line[op_idx] <= l2_resp_data;
                            state[op_idx] <= l2_resp_exclusive ? E : S;
                            read_data_r <= extract_read(l2_resp_data, op_addr, op_func3);
                            fsm <= ST_DONE_R;
                        end
                    end
                end

                ST_DONE_R, ST_DONE_W: fsm <= ST_IDLE;

                default: fsm <= ST_IDLE;
            endcase
        end
    end
endmodule
