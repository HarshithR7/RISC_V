`timescale 1ns / 1ps
// 64-bit-wide data memory supporting RV64I's full load/store set: byte,
// halfword, word (32-bit, signed or unsigned via LW/LWU) and doubleword.
// Byte/halfword/word accesses are supported at any offset *within* a
// doubleword; LD/SD are assumed doubleword-aligned (low 3 address bits
// ignored), consistent with the RV32I sibling core's within-word-only
// support for sub-word accesses.
module data_memory #(
    parameter DMEM_FILE  = "data.mem",
    parameter DMEM_WORDS = 4096,
    // RVV vector port width -- see riscv64_proc.v's LANES parameter
    // ("configurable vector width", RV64I/README.md). Must be a multiple
    // of 64 (a whole number of doublewords): LANES is expected to be
    // even, since SEW=32 always in this scoped implementation, so
    // VLEN=LANES*32 only fails that requirement for an odd LANES, which
    // isn't a configuration this core is designed to support anyway.
    parameter VLEN = 128
)(
    input clk,
    input mem_read,
    input mem_write,
    input [2:0] func3,
    input [63:0] mem_addr,
    input [63:0] write_data,
    output reg [63:0] read_data,

    // RVV unit-stride vector load/store port (SEW=32, LMUL=1 -- see
    // RV64I/README.md's scoped RVV implementation), VLEN bits wide.
    // Requires 8-byte (doubleword) address alignment; mutually exclusive
    // with the scalar port above at the control level (never asserted
    // simultaneously).
    input vmem_read,
    input vmem_write,
    input [63:0] vmem_addr,
    input [VLEN-1:0] vmem_write_data,
    output wire [VLEN-1:0] vmem_read_data
);
    localparam VDWORDS = VLEN / 64;
    localparam LB  = 3'b000;
    localparam LH  = 3'b001;
    localparam LW  = 3'b010;
    localparam LD  = 3'b011;
    localparam LBU = 3'b100;
    localparam LHU = 3'b101;
    localparam LWU = 3'b110;

    reg [63:0] memory [0:DMEM_WORDS-1];
    wire [$clog2(DMEM_WORDS)-1:0] word_addr = mem_addr[$clog2(DMEM_WORDS)+2:3];
    wire [2:0] byte_offset = mem_addr[2:0];
    wire [2:0] half_offset = {byte_offset[2:1], 1'b0}; // 0,2,4,6
    wire [2:0] wrd_offset  = {byte_offset[2], 2'b00};  // 0,4

    initial begin
        for (integer i = 0; i < DMEM_WORDS; i = i + 1)
            memory[i] = 64'b0;
        $readmemh(DMEM_FILE, memory);
        $display("Memory initialized from %s.", DMEM_FILE);
    end

    wire [$clog2(DMEM_WORDS)-1:0] vword_addr = vmem_addr[$clog2(DMEM_WORDS)+2:3];

    reg [63:0] word_after;
    always @(posedge clk) begin
        if (mem_write) begin
            word_after = memory[word_addr];
            case (func3)
                3'b000: word_after[byte_offset*8 +: 8]  = write_data[7:0];   // SB
                3'b001: word_after[half_offset*8 +: 16] = write_data[15:0]; // SH
                3'b010: word_after[wrd_offset*8 +: 32]  = write_data[31:0]; // SW
                default: word_after = write_data;                          // SD
            endcase
            memory[word_addr] <= word_after;
            $display("Time=%0t | MEM WRITE | Addr=%h | func3=%b | Data=%h -> DWord=%h",
                     $time, mem_addr, func3, write_data, word_after);
        end
        if (vmem_write) begin
            for (integer vi = 0; vi < VDWORDS; vi = vi + 1)
                memory[vword_addr + vi] <= vmem_write_data[vi*64 +: 64];
            $display("Time=%0t | VMEM WRITE | Addr=%h | Data=%h", $time, vmem_addr, vmem_write_data);
        end
    end

    // Unconditional (not gated on vmem_read): the top level also uses this
    // to read-modify-write a partial-length (vl<VLMAX) vector store,
    // merging the new lanes with the existing content of the untouched
    // ones.
    genvar vg;
    generate
        for (vg = 0; vg < VDWORDS; vg = vg + 1) begin : vmem_read_gen
            assign vmem_read_data[vg*64 +: 64] = memory[vword_addr + vg];
        end
    endgenerate

    reg [63:0] word;
    reg [7:0]  byte_val;
    reg [15:0] half_val;
    reg [31:0] word_val;
    always @(*) begin
        word     = memory[word_addr];
        byte_val = word[byte_offset*8 +: 8];
        half_val = word[half_offset*8 +: 16];
        word_val = word[wrd_offset*8 +: 32];
        if (mem_read) begin
            case (func3)
                LB:  read_data = {{56{byte_val[7]}}, byte_val};
                LH:  read_data = {{48{half_val[15]}}, half_val};
                LW:  read_data = {{32{word_val[31]}}, word_val};
                LBU: read_data = {56'b0, byte_val};
                LHU: read_data = {48'b0, half_val};
                LWU: read_data = {32'b0, word_val};
                default: read_data = word; // LD
            endcase
            $strobe("Time=%0t | MEM READ  | Addr=%h | func3=%b | ReadData=%h",
                     $time, mem_addr, func3, read_data);
        end else begin
            read_data = 64'b0;
        end
    end
endmodule
