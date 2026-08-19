`timescale 1ns / 1ps
// ECC-protected drop-in replacement for the sibling RV64I core's
// register_file.v (same port list, same reset/write/read semantics),
// used only within RV64I_OOO -- the single-cycle core's own copy stays
// untouched, per this project's standing rule of never modifying
// RV64I/src (see this file's own header note in every prior phase that
// reuses it unchanged). Built as a *new* file rather than editing the
// shared one specifically so Phase 9's redundancy work never touches the
// sibling core's reference implementation.
//
// Each of the 32 architectural registers is stored as {64-bit data, 8-bit
// SECDED check} (see ecc64.v for the code itself). Every write computes
// fresh check bits alongside the data; every read decodes and
// transparently corrects a single-bit error, and reports it via
// sbe_fault/dbe_fault (OR'd across whichever of the two read ports
// actually named a real -- non-x0 -- register this cycle). x0 is still
// hardwired to the constant zero on read exactly as the plain
// register_file.v is, so it's never ECC-checked (there's nothing stored
// there to protect).
//
// x0's `check_mem[0]` reset value (8'b0) matches ecc64's encode of
// 64'b0 (the SECDED parity of an all-zero word is itself all-zero) --
// so even though x0's storage is technically live and reset like every
// other slot, it never spuriously reports an error while never being
// architecturally written, and every non-x0 register starts from the
// same known-good {0, 0} state after reset.
module ecc_register_file (
    input clk,
    input reset,
    input reg_write,
    input  [4:0] read_reg1,
    input  [4:0] read_reg2,
    input  [4:0] write_reg,
    input  [63:0] write_data,
    output reg [63:0] read_data1,
    output reg [63:0] read_data2,

    input reg_write2,
    input [4:0] write_reg2,
    input [63:0] write_data2,

    output sbe_fault,
    output dbe_fault
);
    reg [63:0] data_mem  [0:31];
    reg [7:0]  check_mem [0:31];
    integer i;

    wire [7:0] wr_check1, wr_check2;
    ecc64 enc1 (
        .wr_data(write_data),  .wr_check(wr_check1),
        .rd_data(64'b0), .rd_check(8'b0),
        .rd_data_corrected(), .rd_sbe(), .rd_dbe()
    );
    ecc64 enc2 (
        .wr_data(write_data2), .wr_check(wr_check2),
        .rd_data(64'b0), .rd_check(8'b0),
        .rd_data_corrected(), .rd_sbe(), .rd_dbe()
    );

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                data_mem[i]  <= 64'b0;
                check_mem[i] <= 8'b0;
            end
        end
        else begin
            // Same write-port-2-wins-on-conflict ordering as
            // register_file.v (last non-blocking assignment in program
            // order to the same index wins); data and check are updated
            // together per port so a conflict can never leave one port's
            // data paired with the other port's check bits.
            if (reg_write && write_reg != 5'd0) begin
                data_mem[write_reg]  <= write_data;
                check_mem[write_reg] <= wr_check1;
            end
            if (reg_write2 && write_reg2 != 5'd0) begin
                data_mem[write_reg2]  <= write_data2;
                check_mem[write_reg2] <= wr_check2;
            end
        end
    end

    wire [63:0] corrected1, corrected2;
    wire sbe1, dbe1, sbe2, dbe2;
    ecc64 dec1 (
        .wr_data(64'b0), .wr_check(),
        .rd_data(data_mem[read_reg1]), .rd_check(check_mem[read_reg1]),
        .rd_data_corrected(corrected1), .rd_sbe(sbe1), .rd_dbe(dbe1)
    );
    ecc64 dec2 (
        .wr_data(64'b0), .wr_check(),
        .rd_data(data_mem[read_reg2]), .rd_check(check_mem[read_reg2]),
        .rd_data_corrected(corrected2), .rd_sbe(sbe2), .rd_dbe(dbe2)
    );

    always @(*) begin
        read_data1 = (read_reg1 == 5'd0) ? 64'b0 : corrected1;
        read_data2 = (read_reg2 == 5'd0) ? 64'b0 : corrected2;
    end

    assign sbe_fault = (read_reg1 != 5'd0 && sbe1) || (read_reg2 != 5'd0 && sbe2);
    assign dbe_fault = (read_reg1 != 5'd0 && dbe1) || (read_reg2 != 5'd0 && dbe2);
endmodule
