`timescale 1ns / 1ps
// Expands RV64C compressed (16-bit) instructions into their exact 32-bit
// RV64I/M equivalent, so decode_control_unit/execute/data_memory need no
// awareness of compression at all -- they only ever see real 32-bit
// instruction words. `fetch32` is {next_halfword, this_halfword}: the
// low 16 bits are the halfword addressed by the current PC (a complete
// compressed instruction, or the low half of an unaligned 32-bit one);
// the high 16 bits are only used when this instruction turns out to be a
// full 32-bit instruction starting at a 2-byte-aligned (not necessarily
// 4-byte-aligned) address.
module compressed_decoder (
    input [31:0] fetch32,
    output reg [31:0] instruction,
    output wire is_compressed
);
    wire [15:0] c = fetch32[15:0];
    wire [1:0]  op = c[1:0];
    wire [2:0]  f3 = c[15:13];
    assign is_compressed = (op != 2'b11);

    // Opcodes of the target (expanded) 32-bit ISA
    localparam OP_R      = 7'b0110011;
    localparam OP_I      = 7'b0010011;
    localparam OP_LOAD   = 7'b0000011;
    localparam OP_S      = 7'b0100011;
    localparam OP_B      = 7'b1100011;
    localparam OP_LUI    = 7'b0110111;
    localparam OP_JAL    = 7'b1101111;
    localparam OP_JALR   = 7'b1100111;
    localparam OP_IMM_32 = 7'b0011011;
    localparam OP_32     = 7'b0111011;
    localparam OP_SYSTEM = 7'b1110011;

    function [31:0] mk_r;
        input [6:0] opc; input [2:0] fn3; input [6:0] fn7; input [4:0] rd, rs1, rs2;
        mk_r = {fn7, rs2, rs1, fn3, rd, opc};
    endfunction
    function [31:0] mk_i;
        input [6:0] opc; input [2:0] fn3; input [4:0] rd, rs1; input [11:0] immv;
        mk_i = {immv, rs1, fn3, rd, opc};
    endfunction
    function [31:0] mk_s;
        input [6:0] opc; input [2:0] fn3; input [4:0] rs1, rs2; input [11:0] immv;
        mk_s = {immv[11:5], rs2, rs1, fn3, immv[4:0], opc};
    endfunction
    function [31:0] mk_b;
        input [6:0] opc; input [2:0] fn3; input [4:0] rs1, rs2; input [12:0] immv;
        mk_b = {immv[12], immv[10:5], rs2, rs1, fn3, immv[4:1], immv[11], opc};
    endfunction
    function [31:0] mk_u;
        input [6:0] opc; input [4:0] rd; input [19:0] immv;
        mk_u = {immv, rd, opc};
    endfunction
    function [31:0] mk_j;
        input [6:0] opc; input [4:0] rd; input [20:0] immv;
        mk_j = {immv[20], immv[10:1], immv[11], immv[19:12], rd, opc};
    endfunction

    // Compressed register fields: 3-bit c-regs address x8-x15; CI/CR/CSS
    // use the full 5-bit register number directly.
    wire [4:0] rd_rs1_5 = c[11:7];
    wire [4:0] rs2_5    = c[6:2];
    wire [4:0] rd3  = {2'b01, c[4:2]};  // CIW rd'
    wire [4:0] rs1p = {2'b01, c[9:7]};  // CL/CS/CA/CB rs1'/rd'
    wire [4:0] rs2p = {2'b01, c[4:2]};  // CL/CS/CA rs2'/rd'

    always @(*) begin
        instruction = 32'h00000013; // default: NOP (addi x0,x0,0) for anything unrecognized

        if (!is_compressed) begin
            instruction = fetch32; // already a full 32-bit instruction
        end else begin
            case (op)
                2'b00: case (f3)
                    3'b000: // C.ADDI4SPN
                        instruction = mk_i(OP_I, 3'b000, rd3, 5'd2,
                            {2'b00, c[10:7], c[12:11], c[5], c[6], 2'b00});
                    3'b010: // C.LW
                        instruction = mk_i(OP_LOAD, 3'b010, rs2p, rs1p,
                            {5'b0, c[5], c[12:10], c[6], 2'b00});
                    3'b011: // C.LD
                        instruction = mk_i(OP_LOAD, 3'b011, rs2p, rs1p,
                            {4'b0, c[6:5], c[12:10], 3'b000});
                    3'b110: // C.SW
                        instruction = mk_s(OP_S, 3'b010, rs1p, rs2p,
                            {5'b0, c[5], c[12:10], c[6], 2'b00});
                    3'b111: // C.SD
                        instruction = mk_s(OP_S, 3'b011, rs1p, rs2p,
                            {4'b0, c[6:5], c[12:10], 3'b000});
                    default: ;
                endcase

                2'b01: case (f3)
                    3'b000: // C.NOP / C.ADDI
                        instruction = mk_i(OP_I, 3'b000, rd_rs1_5, rd_rs1_5,
                            {{6{c[12]}}, c[12], c[6:2]});
                    3'b001: // C.ADDIW
                        instruction = mk_i(OP_IMM_32, 3'b000, rd_rs1_5, rd_rs1_5,
                            {{6{c[12]}}, c[12], c[6:2]});
                    3'b010: // C.LI
                        instruction = mk_i(OP_I, 3'b000, rd_rs1_5, 5'd0,
                            {{6{c[12]}}, c[12], c[6:2]});
                    3'b011: begin
                        if (rd_rs1_5 == 5'd2) // C.ADDI16SP
                            instruction = mk_i(OP_I, 3'b000, 5'd2, 5'd2,
                                {{2{c[12]}}, c[12], c[4:3], c[5], c[2], c[6], 4'b0000});
                        else // C.LUI
                            instruction = mk_u(OP_LUI, rd_rs1_5, {{14{c[12]}}, c[12], c[6:2]});
                    end
                    3'b100: case (c[11:10])
                        2'b00: // C.SRLI
                            instruction = mk_i(OP_I, 3'b101, rs1p, rs1p, {6'b000000, c[12], c[6:2]});
                        2'b01: // C.SRAI
                            instruction = mk_i(OP_I, 3'b101, rs1p, rs1p, {6'b010000, c[12], c[6:2]});
                        2'b10: // C.ANDI
                            instruction = mk_i(OP_I, 3'b111, rs1p, rs1p, {{6{c[12]}}, c[12], c[6:2]});
                        2'b11: begin
                            if (c[12] == 1'b0) case (c[6:5])
                                2'b00: instruction = mk_r(OP_R, 3'b000, 7'b0100000, rs1p, rs1p, rs2p); // C.SUB
                                2'b01: instruction = mk_r(OP_R, 3'b100, 7'b0000000, rs1p, rs1p, rs2p); // C.XOR
                                2'b10: instruction = mk_r(OP_R, 3'b110, 7'b0000000, rs1p, rs1p, rs2p); // C.OR
                                2'b11: instruction = mk_r(OP_R, 3'b111, 7'b0000000, rs1p, rs1p, rs2p); // C.AND
                            endcase else case (c[6:5])
                                2'b00: instruction = mk_r(OP_32, 3'b000, 7'b0100000, rs1p, rs1p, rs2p); // C.SUBW
                                2'b01: instruction = mk_r(OP_32, 3'b000, 7'b0000000, rs1p, rs1p, rs2p); // C.ADDW
                                default: ;
                            endcase
                        end
                    endcase
                    3'b101: // C.J
                        instruction = mk_j(OP_JAL, 5'd0,
                            {{9{c[12]}}, c[12], c[8], c[10:9], c[6], c[7], c[2], c[11], c[5:3], 1'b0});
                    3'b110: // C.BEQZ
                        instruction = mk_b(OP_B, 3'b000, rs1p, 5'd0,
                            {{4{c[12]}}, c[12], c[6:5], c[2], c[11:10], c[4:3], 1'b0});
                    3'b111: // C.BNEZ
                        instruction = mk_b(OP_B, 3'b001, rs1p, 5'd0,
                            {{4{c[12]}}, c[12], c[6:5], c[2], c[11:10], c[4:3], 1'b0});
                endcase

                2'b10: case (f3)
                    3'b000: // C.SLLI
                        instruction = mk_i(OP_I, 3'b001, rd_rs1_5, rd_rs1_5, {6'b000000, c[12], c[6:2]});
                    3'b010: // C.LWSP
                        instruction = mk_i(OP_LOAD, 3'b010, rd_rs1_5, 5'd2,
                            {4'b0, c[3:2], c[12], c[6:4], 2'b00});
                    3'b011: // C.LDSP
                        instruction = mk_i(OP_LOAD, 3'b011, rd_rs1_5, 5'd2,
                            {3'b0, c[4:2], c[12], c[6:5], 3'b000});
                    3'b100: begin
                        if (c[12] == 1'b0) begin
                            if (rs2_5 == 5'd0) // C.JR
                                instruction = mk_i(OP_JALR, 3'b000, 5'd0, rd_rs1_5, 12'b0);
                            else // C.MV
                                instruction = mk_r(OP_R, 3'b000, 7'b0000000, rd_rs1_5, 5'd0, rs2_5);
                        end else begin
                            if (rd_rs1_5 == 5'd0 && rs2_5 == 5'd0) // C.EBREAK
                                instruction = mk_i(OP_SYSTEM, 3'b000, 5'd0, 5'd0, 12'd1);
                            else if (rs2_5 == 5'd0) // C.JALR
                                instruction = mk_i(OP_JALR, 3'b000, 5'd1, rd_rs1_5, 12'b0);
                            else // C.ADD
                                instruction = mk_r(OP_R, 3'b000, 7'b0000000, rd_rs1_5, rd_rs1_5, rs2_5);
                        end
                    end
                    3'b110: // C.SWSP
                        instruction = mk_s(OP_S, 3'b010, 5'd2, rs2_5,
                            {4'b0, c[8:7], c[12:9], 2'b00});
                    3'b111: // C.SDSP
                        instruction = mk_s(OP_S, 3'b011, 5'd2, rs2_5,
                            {3'b0, c[9:7], c[12:10], 3'b000});
                    default: ;
                endcase

                default: ; // op==11 handled above (not compressed)
            endcase
        end
    end
endmodule
