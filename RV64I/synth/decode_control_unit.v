`timescale 1ns / 1ps
// Combinational RV64I decode + control. Instructions are still 32 bits;
// immediates are sign-extended to 64 bits. Adds OP-IMM-32 (ADDIW/SLLIW/
// SRLIW/SRAIW) and OP-32 (ADDW/SUBW/SLLW/SRLW/SRAW) over the RV32I core,
// plus wider (6-bit) shift amounts for the 64-bit shift instructions and a
// `word_op` flag telling the execute stage to compute in 32 bits and
// sign-extend the result to 64, per the *W instruction semantics.
//
// M extension: MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU share R_TYPE's
// opcode with funct7=0000001 (RVM_FUNCT7); MULW/DIVW/DIVUW/REMW/REMUW
// share OP_32's opcode the same way. `is_muldiv` + `muldiv_op` select
// which one for the execute stage; `word_op` (already used by ADDW etc.)
// is reused unchanged to select the truncate-then-sign-extend behavior.
module decode_control_unit(
    input [31:0] instruction,

    output wire [6:0] opcode,
    output wire [4:0] rs1,
    output wire [4:0] rs2,
    output wire [4:0] rd,
    output wire [2:0] func3,
    output wire [6:0] func7,
    output reg  [63:0] imm,

    output reg reg_write,
    output reg mem_read,
    output reg mem_write,
    output reg mem_to_reg,
    output reg [3:0] alu_op,
    output reg alu_src,
    output reg word_op,
    output reg is_muldiv,
    output reg [2:0] muldiv_op,
    output reg is_amo,
    output reg is_lr,
    output reg is_sc,
    output reg [4:0] amo_funct5,

    // F extension
    output reg is_fp_op,       // this instruction uses the FPU (arithmetic/compare/convert/move)
    output reg [4:0] fp_op,    // which FPU operation (see fpu.v)
    output reg rs1_is_fp,
    output reg rs2_is_fp,
    output reg rd_is_fp,
    output reg [4:0] rs3,      // R4-type (FMADD family) third source register
    output reg fp_mem_op,      // FLW/FSW: reuse the integer load/store datapath, routed to/from the FP regfile
    output reg word_int_src,   // FCVT.S.W/WU (32-bit int source) vs FCVT.S.L/LU (64-bit): selects sign-extension width
    output reg fp_reg_write,   // separate from `reg_write` (integer): set when rd is an FP register
    output reg is_double,      // D extension: this FP op is double- rather than single-precision

    // RVV (scoped: VLEN=128, SEW=32, LMUL=1 only -- see RV64I/README.md)
    output reg is_vector_op,
    output reg is_vsetvli,
    output reg [4:0] v_op,      // see vector_alu.v
    output reg [1:0] v_form,    // 0=.vv 1=.vx 2=.vi
    output reg [4:0] v_imm,     // raw 5-bit immediate for the .vi form (sign-extended downstream)
    output reg vreg_write,
    output reg is_vle,
    output reg is_vse,
    output reg v_vm,            // instruction[25]: 1=unmasked (always), 0=masked by v0
    output reg is_vcompare,     // result is a 1-bit-per-lane mask (vmseq/etc), not per-lane 32-bit values
    output reg is_vreduce       // vredsum/etc: fold vs2's active lanes (seeded by vs1[0]) into one scalar in vd[0]
);
    assign opcode = instruction[6:0];
    assign rs1    = instruction[19:15];
    assign rs2    = instruction[24:20];
    assign rd     = instruction[11:7];
    assign func3  = instruction[14:12];
    assign func7  = instruction[31:25];

    localparam R_TYPE    = 7'b0110011;
    localparam I_TYPE    = 7'b0010011;
    localparam S_TYPE    = 7'b0100011;
    localparam B_TYPE    = 7'b1100011;
    localparam LUI_TYPE  = 7'b0110111;
    localparam AUIPC     = 7'b0010111;
    localparam JAL       = 7'b1101111;
    localparam JALR      = 7'b1100111;
    localparam LOAD      = 7'b0000011;
    localparam OP_IMM_32 = 7'b0011011;
    localparam OP_32     = 7'b0111011;
    localparam AMO       = 7'b0101111;
    localparam LOAD_FP   = 7'b0000111;
    localparam STORE_FP  = 7'b0100111;
    localparam MADD      = 7'b1000011;
    localparam MSUB      = 7'b1000111;
    localparam NMSUB     = 7'b1001011;
    localparam NMADD     = 7'b1001111;
    localparam OP_FP     = 7'b1010011;
    localparam OP_V      = 7'b1010111;

    // Matches fpu.v's fp_op encoding exactly.
    localparam FP_FADD=5'd0, FP_FSUB=5'd1, FP_FMUL=5'd2, FP_FDIV=5'd3, FP_FSQRT=5'd4,
               FP_FSGNJ=5'd5, FP_FSGNJN=5'd6, FP_FSGNJX=5'd7, FP_FMIN=5'd8, FP_FMAX=5'd9,
               FP_FEQ=5'd10, FP_FLT=5'd11, FP_FLE=5'd12, FP_FCLASS=5'd13,
               FP_FCVT_W_S=5'd14, FP_FCVT_WU_S=5'd15, FP_FCVT_L_S=5'd16, FP_FCVT_LU_S=5'd17,
               FP_FCVT_S_W=5'd18, FP_FCVT_S_WU=5'd19, FP_FCVT_S_L=5'd20, FP_FCVT_S_LU=5'd21,
               FP_FMV_X_W=5'd22, FP_FMV_W_X=5'd23,
               FP_FMADD=5'd24, FP_FMSUB=5'd25, FP_FNMSUB=5'd26, FP_FNMADD=5'd27,
               FP_FCVT_S_D=5'd28, FP_FCVT_D_S=5'd29;

    localparam ALU_ADD   = 4'b0010;
    localparam ALU_SUB   = 4'b1010;
    localparam ALU_AND   = 4'b0100;
    localparam ALU_OR    = 4'b0101;
    localparam ALU_XOR   = 4'b0011;
    localparam ALU_SLL   = 4'b0110;
    localparam ALU_SRL   = 4'b0111;
    localparam ALU_SRA   = 4'b1000;
    localparam ALU_SLT   = 4'b1011;
    localparam ALU_SLTU  = 4'b1100;

    localparam RVM_FUNCT7 = 7'b0000001;
    localparam MULDIV_MUL    = 3'b000;
    localparam MULDIV_MULH   = 3'b001;
    localparam MULDIV_MULHSU = 3'b010;
    localparam MULDIV_MULHU  = 3'b011;
    localparam MULDIV_DIV    = 3'b100;
    localparam MULDIV_DIVU   = 3'b101;
    localparam MULDIV_REM    = 3'b110;
    localparam MULDIV_REMU   = 3'b111;

    localparam AMO_FUNCT5_LR = 5'b00010;
    localparam AMO_FUNCT5_SC = 5'b00011;

    // Immediate generation (sign-extended to 64 bits)
    always @(*) begin
        case (opcode)
            LOAD, I_TYPE, JALR, OP_IMM_32:
                imm = {{52{instruction[31]}}, instruction[31:20]};
            LOAD_FP:
                // Real scalar FLW/FLD (func3=010/011) encode a real I-type
                // immediate here; RVV's vle32.v (func3=110) reuses these
                // same bits as nf/mew/mop/vm/lumop structural fields, not
                // an offset -- vector unit-stride loads have no immediate
                // at all (address is rs1 alone), so imm must stay 0 for
                // them, not whatever garbage those bits decode to as a
                // number.
                imm = (func3 == 3'b110) ? 64'b0 : {{52{instruction[31]}}, instruction[31:20]};
            S_TYPE:
                imm = {{52{instruction[31]}}, instruction[31:25], instruction[11:7]};
            STORE_FP:
                // Same reasoning as LOAD_FP above, for FSW/FSD vs. vse32.v.
                imm = (func3 == 3'b110) ? 64'b0 : {{52{instruction[31]}}, instruction[31:25], instruction[11:7]};
            B_TYPE:
                imm = {{51{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            JAL:
                imm = {{43{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
            LUI_TYPE, AUIPC:
                imm = {{32{instruction[31]}}, instruction[31:12], 12'b0};
            default:
                imm = 64'b0;
        endcase
    end

    // Control logic
    always @(*) begin
        reg_write  = 0;
        mem_read   = 0;
        mem_write  = 0;
        mem_to_reg = 0;
        alu_op     = ALU_ADD;
        alu_src    = 0;
        word_op    = 0;
        is_muldiv  = 0;
        muldiv_op  = 3'b000;
        is_amo     = 0;
        is_lr      = 0;
        is_sc      = 0;
        amo_funct5 = 5'b0;
        is_fp_op   = 0;
        fp_op      = 5'b0;
        rs1_is_fp  = 0;
        rs2_is_fp  = 0;
        rd_is_fp   = 0;
        rs3        = 5'b0;
        fp_mem_op  = 0;
        word_int_src = 0;
        fp_reg_write = 0;
        is_double  = 0;
        is_vector_op = 0;
        is_vsetvli = 0;
        v_op       = 5'b0;
        v_form     = 2'b0;
        v_imm      = 5'b0;
        vreg_write = 0;
        is_vle     = 0;
        is_vse     = 0;
        v_vm       = 1'b1;
        is_vcompare = 0;
        is_vreduce  = 0;

        case (opcode)
            R_TYPE: begin
                reg_write = 1;
                alu_src   = 0;
                if (func7 == RVM_FUNCT7) begin
                    is_muldiv = 1;
                    muldiv_op = func3; // func3 encoding already matches MUL/MULH/.../REMU order
                end else begin
                    case ({func7, func3})
                        10'b0000000_000: alu_op = ALU_ADD;
                        10'b0100000_000: alu_op = ALU_SUB;
                        10'b0000000_111: alu_op = ALU_AND;
                        10'b0000000_110: alu_op = ALU_OR;
                        10'b0000000_100: alu_op = ALU_XOR;
                        10'b0000000_001: alu_op = ALU_SLL;
                        10'b0000000_101: alu_op = ALU_SRL;
                        10'b0100000_101: alu_op = ALU_SRA;
                        10'b0000000_010: alu_op = ALU_SLT;
                        10'b0000000_011: alu_op = ALU_SLTU;
                    endcase
                end
            end

            OP_32: begin
                reg_write = 1;
                alu_src   = 0;
                word_op   = 1;
                if (func7 == RVM_FUNCT7) begin
                    is_muldiv = 1;
                    muldiv_op = func3; // MULW/DIVW/DIVUW/REMW/REMUW (no MULHW family exists)
                end else begin
                    case ({func7, func3})
                        10'b0000000_000: alu_op = ALU_ADD;  // ADDW
                        10'b0100000_000: alu_op = ALU_SUB;  // SUBW
                        10'b0000000_001: alu_op = ALU_SLL;  // SLLW
                        10'b0000000_101: alu_op = ALU_SRL;  // SRLW
                        10'b0100000_101: alu_op = ALU_SRA;  // SRAW
                    endcase
                end
            end

            I_TYPE: begin
                reg_write = 1;
                alu_src   = 1;
                case (func3)
                    3'b000: alu_op = ALU_ADD;   // ADDI
                    3'b010: alu_op = ALU_SLT;   // SLTI
                    3'b011: alu_op = ALU_SLTU;  // SLTIU
                    3'b100: alu_op = ALU_XOR;   // XORI
                    3'b110: alu_op = ALU_OR;    // ORI
                    3'b111: alu_op = ALU_AND;   // ANDI
                    3'b001: alu_op = ALU_SLL;   // SLLI (shamt = instruction[25:20], funct6 == 0)
                    3'b101: begin               // SRLI / SRAI (shamt = instruction[25:20])
                        if (instruction[31:26] == 6'b000000) alu_op = ALU_SRL;
                        else if (instruction[31:26] == 6'b010000) alu_op = ALU_SRA;
                    end
                endcase
            end

            OP_IMM_32: begin
                reg_write = 1;
                alu_src   = 1;
                word_op   = 1;
                case (func3)
                    3'b000: alu_op = ALU_ADD;  // ADDIW
                    3'b001: alu_op = ALU_SLL;  // SLLIW (shamt = instruction[24:20], funct7 == 0)
                    3'b101: begin              // SRLIW / SRAIW (shamt = instruction[24:20])
                        if (func7 == 7'b0000000) alu_op = ALU_SRL;
                        else if (func7 == 7'b0100000) alu_op = ALU_SRA;
                    end
                endcase
            end

            LOAD: begin
                mem_to_reg = 1;
                reg_write  = 1;
                mem_read   = 1;
                alu_src    = 1;
                alu_op     = ALU_ADD;
            end

            S_TYPE: begin
                mem_write = 1;
                alu_src   = 1;
                alu_op    = ALU_ADD;
            end

            // A extension: LR/SC/AMO*. Address is rs1 directly (no
            // immediate offset -- imm defaults to 0 for this opcode, so
            // execute's existing rs1_data+imm address calc still works).
            // rd always comes from the *old* memory value (mem_to_reg),
            // for both AMO and LR; SC's rd is synthesized separately at
            // the top level (0=success/1=failure), overriding this.
            AMO: begin
                reg_write  = 1;
                mem_read   = 1;
                mem_to_reg = 1;
                alu_src    = 0;
                amo_funct5 = instruction[31:27];
                if (instruction[31:27] == AMO_FUNCT5_LR) begin
                    is_lr = 1;
                end else if (instruction[31:27] == AMO_FUNCT5_SC) begin
                    is_sc = 1; // mem_write left 0 here -- top level gates it on reservation success
                end else begin
                    is_amo    = 1;
                    mem_write = 1;
                end
            end

            B_TYPE: begin
                alu_src = 0;
                alu_op  = ALU_SUB;
            end

            JAL: reg_write = 1;

            JALR: begin
                reg_write = 1;
                alu_src   = 1;
                alu_op    = ALU_ADD;
            end

            LUI_TYPE: begin
                reg_write = 1;
                alu_op    = ALU_ADD; // rd = 0 + imm (operand-1 forced to 0 in execute)
                alu_src   = 1;
            end

            AUIPC: begin
                reg_write = 1;
                alu_op    = ALU_ADD; // rd = pc + imm (operand-1 forced to pc in execute)
                alu_src   = 1;
            end

            // F/D extension: FLW/FSW/FLD/FSD reuse the integer load/store
            // address calculation (rs1(int) + imm) and data_memory's
            // existing width path unchanged (func3 010=word/FLW-FSW,
            // 011=doubleword/FLD-FSD -- already exactly LW/SW vs LD/SD's
            // own encoding); only the destination/source register file
            // differs. is_double follows func3 directly.
            //
            // RVV also shares this opcode for its unit-stride loads/stores
            // (real hardware disambiguates the same way: vector width
            // encodings, func3=110 for a 32-bit element here, don't
            // overlap scalar F/D's 010/011). Vector loads/stores have no
            // immediate offset field at all (always unit-stride from rs1
            // alone), unlike FLW/FSW/FLD/FSD.
            LOAD_FP: begin
                if (func3 == 3'b110) begin
                    is_vle = 1;
                    vreg_write = 1;
                    alu_src = 1; // imm defaults to 0 for this opcode+func3 (not in the imm case list) -> address = rs1 alone
                    alu_op  = ALU_ADD;
                end else begin
                    mem_read  = 1;
                    alu_src   = 1;
                    alu_op    = ALU_ADD;
                    rd_is_fp  = 1;
                    fp_mem_op = 1;
                    fp_reg_write = 1;
                    is_double = (func3 == 3'b011);
                end
            end

            STORE_FP: begin
                if (func3 == 3'b110) begin
                    is_vse = 1;
                    alu_src = 1;
                    alu_op  = ALU_ADD;
                end else begin
                    mem_write = 1;
                    alu_src   = 1;
                    alu_op    = ALU_ADD;
                    rs2_is_fp = 1;
                    fp_mem_op = 1;
                    is_double = (func3 == 3'b011);
                end
            end

            MADD, MSUB, NMSUB, NMADD: begin
                is_fp_op  = 1;
                rs1_is_fp = 1;
                rs2_is_fp = 1;
                rd_is_fp  = 1;
                fp_reg_write = 1;
                rs3       = instruction[31:27];
                is_double = (instruction[26:25] == 2'b01); // fmt field: 00=single, 01=double
                case (opcode)
                    MADD:  fp_op = FP_FMADD;
                    MSUB:  fp_op = FP_FMSUB;
                    NMSUB: fp_op = FP_FNMSUB;
                    NMADD: fp_op = FP_FNMADD;
                endcase
            end

            // D's encodings are identical to F's with funct7's LSB set to
            // 1 (e.g. FADD.S=0000000 -> FADD.D=0000001); is_double just
            // follows that bit directly except for the two cross-format
            // conversions (FCVT.S.D/FCVT.D.S), which have their own
            // dedicated funct7 values and use rs2 to select the source
            // format instead.
            OP_FP: begin
                is_fp_op = 1;
                is_double = func7[0];
                case (func7)
                    7'b0000000, 7'b0000001: begin fp_op = FP_FADD;  rs1_is_fp=1; rs2_is_fp=1; rd_is_fp=1; end
                    7'b0000100, 7'b0000101: begin fp_op = FP_FSUB;  rs1_is_fp=1; rs2_is_fp=1; rd_is_fp=1; end
                    7'b0001000, 7'b0001001: begin fp_op = FP_FMUL;  rs1_is_fp=1; rs2_is_fp=1; rd_is_fp=1; end
                    7'b0001100, 7'b0001101: begin fp_op = FP_FDIV;  rs1_is_fp=1; rs2_is_fp=1; rd_is_fp=1; end
                    7'b0101100, 7'b0101101: begin fp_op = FP_FSQRT; rs1_is_fp=1; rd_is_fp=1; end
                    7'b0010000, 7'b0010001: begin
                        rs1_is_fp=1; rs2_is_fp=1; rd_is_fp=1;
                        case (func3)
                            3'b000: fp_op = FP_FSGNJ;
                            3'b001: fp_op = FP_FSGNJN;
                            3'b010: fp_op = FP_FSGNJX;
                        endcase
                    end
                    7'b0010100, 7'b0010101: begin
                        rs1_is_fp=1; rs2_is_fp=1; rd_is_fp=1;
                        case (func3)
                            3'b000: fp_op = FP_FMIN;
                            3'b001: fp_op = FP_FMAX;
                        endcase
                    end
                    7'b1010000, 7'b1010001: begin
                        rs1_is_fp=1; rs2_is_fp=1; rd_is_fp=0;
                        case (func3)
                            3'b010: fp_op = FP_FEQ;
                            3'b001: fp_op = FP_FLT;
                            3'b000: fp_op = FP_FLE;
                        endcase
                    end
                    7'b1100000, 7'b1100001: begin // FCVT.{W,WU,L,LU}.{S,D} -- float -> int
                        rs1_is_fp = 1; rd_is_fp = 0;
                        case (rs2)
                            5'b00000: fp_op = FP_FCVT_W_S;
                            5'b00001: fp_op = FP_FCVT_WU_S;
                            5'b00010: fp_op = FP_FCVT_L_S;
                            5'b00011: fp_op = FP_FCVT_LU_S;
                        endcase
                    end
                    7'b1101000, 7'b1101001: begin // FCVT.{S,D}.{W,WU,L,LU} -- int -> float
                        rs1_is_fp = 0; rd_is_fp = 1;
                        case (rs2)
                            5'b00000: begin fp_op = FP_FCVT_S_W;  word_int_src = 1; end
                            5'b00001: begin fp_op = FP_FCVT_S_WU; word_int_src = 1; end
                            5'b00010: begin fp_op = FP_FCVT_S_L;  word_int_src = 0; end
                            5'b00011: begin fp_op = FP_FCVT_S_LU; word_int_src = 0; end
                        endcase
                    end
                    7'b1110000, 7'b1110001: begin // FMV.X.{W,D} / FCLASS.{S,D}
                        rs1_is_fp = 1; rd_is_fp = 0;
                        fp_op = (func3 == 3'b001) ? FP_FCLASS : FP_FMV_X_W;
                    end
                    7'b1111000, 7'b1111001: begin // FMV.{W,D}.X
                        rs1_is_fp = 0; rd_is_fp = 1;
                        fp_op = FP_FMV_W_X;
                    end
                    7'b0100000: begin // FCVT.S.D: double -> single (rs2=00001 selects double source)
                        rs1_is_fp = 1; rd_is_fp = 1;
                        fp_op = FP_FCVT_S_D;
                        is_double = 1'b1; // source is double; result routing inside fpu.v narrows it
                    end
                    7'b0100001: begin // FCVT.D.S: single -> double (rs2=00000 selects single source)
                        rs1_is_fp = 1; rd_is_fp = 1;
                        fp_op = FP_FCVT_D_S;
                        is_double = 1'b0; // source is single; fpu.v widens it regardless of this flag for this op
                    end
                endcase
                // rd_is_fp was set by whichever case above matched; route
                // the write-enable to the correct register file.
                fp_reg_write = rd_is_fp;
                reg_write = ~rd_is_fp;
            end

            // RVV (scoped: SEW=32/LMUL=1 only). vsetvli/vsetvl share
            // func3=111; this core doesn't distinguish them further since
            // it only ever configures the one fixed vtype it supports --
            // rd always gets the new vl, computed at the top level from
            // rs1's value (needs the actual register read, not available
            // here). Arithmetic/compare ops decode funct6
            // (instruction[31:26], funct7 minus the vm bit) and func3
            // selects which operand2 comes from (.vv/.vx/.vi); vs2/vd
            // reuse the same field positions as rs2/rd, vs1/scalar-rs1/
            // immediate reuse rs1's. `vm` (instruction[25]) selects
            // masked (0, by v0) vs unmasked (1, always) execution.
            //
            // Reductions (vredsum/etc) share OPMVV's func3=010 with
            // VMUL/VDIV* but occupy a disjoint, low funct6 range
            // (000000-000111) -- the mul/div group's funct6 values all
            // start at 100000 and never collide with it. They reuse the
            // elementwise ADD/AND/OR/XOR/MIN*/MAX* op codes (the fold
            // operator is the same; only the *shape* of the result
            // differs, handled by is_vreduce at the top level, not by a
            // separate v_op space).
            //
            // The non-reduction arithmetic/compare ops are decoded in
            // TWO disjoint funct6 tables, gated on func3's group, because
            // real RVV reuses funct6 values across groups: vsll's funct6
            // (100101, OPIVV/OPIVX/OPIVI) is numerically identical to
            // vmul's funct6 (100101, OPMVV/OPMVX) -- func3 is the only
            // thing that disambiguates them. A single funct6-only case
            // (as an earlier version of this decoder had, before shifts
            // were added) would silently alias vsll.vv onto vmul.vv.
            OP_V: begin
                if (func3 == 3'b111) begin
                    is_vsetvli = 1;
                    reg_write  = 1;
                end else begin
                    v_vm  = instruction[25];
                    v_imm = instruction[19:15];
                    case (func3)
                        3'b000: v_form = 2'd0; // OPIVV
                        3'b100: v_form = 2'd1; // OPIVX
                        3'b011: v_form = 2'd2; // OPIVI
                        3'b010: v_form = 2'd0; // OPMVV (vmul/vdiv*/reductions)
                        3'b110: v_form = 2'd1; // OPMVX (vmul.vx/vdiv*.vx)
                        default: v_form = 2'd0;
                    endcase

                    if (func3 == 3'b010 && instruction[31:26] < 6'b001000) begin
                        is_vreduce = 1;
                        vreg_write = 1;
                        case (instruction[31:26])
                            6'b000000: v_op = 5'd0;  // VREDSUM  (reuses VADD)
                            6'b000001: v_op = 5'd2;  // VREDAND  (reuses VAND)
                            6'b000010: v_op = 5'd3;  // VREDOR
                            6'b000011: v_op = 5'd4;  // VREDXOR
                            6'b000100: v_op = 5'd13; // VREDMINU
                            6'b000101: v_op = 5'd12; // VREDMIN
                            6'b000110: v_op = 5'd15; // VREDMAXU
                            6'b000111: v_op = 5'd14; // VREDMAX
                            default:   v_op = 5'd0;
                        endcase
                    end else if (func3 == 3'b010 || func3 == 3'b110) begin
                        // OPMVV/OPMVX: multiply/divide/remainder group.
                        is_vector_op = 1;
                        vreg_write   = 1;
                        case (instruction[31:26]) // funct6
                            6'b100101: v_op = 5'd5;  // VMUL
                            6'b100000: v_op = 5'd16; // VDIVU
                            6'b100001: v_op = 5'd17; // VDIV
                            6'b100010: v_op = 5'd18; // VREMU
                            6'b100011: v_op = 5'd19; // VREM
                            default:   v_op = 5'd0;
                        endcase
                    end else begin
                        // OPIVV/OPIVX/OPIVI: integer arithmetic/compare/
                        // min-max/shift group.
                        is_vector_op = 1;
                        vreg_write   = 1;
                        case (instruction[31:26]) // funct6
                            6'b000000: v_op = 5'd0;  // VADD
                            6'b000010: v_op = 5'd1;  // VSUB
                            6'b001001: v_op = 5'd2;  // VAND
                            6'b001010: v_op = 5'd3;  // VOR
                            6'b001011: v_op = 5'd4;  // VXOR
                            6'b011000: begin v_op = 5'd6;  is_vcompare = 1; end // VMSEQ
                            6'b011001: begin v_op = 5'd7;  is_vcompare = 1; end // VMSNE
                            6'b011011: begin v_op = 5'd8;  is_vcompare = 1; end // VMSLT
                            6'b011010: begin v_op = 5'd9;  is_vcompare = 1; end // VMSLTU
                            6'b011101: begin v_op = 5'd10; is_vcompare = 1; end // VMSLE
                            6'b011100: begin v_op = 5'd11; is_vcompare = 1; end // VMSLEU
                            6'b000101: v_op = 5'd12; // VMIN
                            6'b000100: v_op = 5'd13; // VMINU
                            6'b000111: v_op = 5'd14; // VMAX
                            6'b000110: v_op = 5'd15; // VMAXU
                            6'b100101: v_op = 5'd20; // VSLL
                            6'b101000: v_op = 5'd21; // VSRL
                            6'b101001: v_op = 5'd22; // VSRA
                            default:   v_op = 5'd0;
                        endcase
                    end
                end
            end

            default: ; // SYSTEM/FENCE/unknown: no register or memory writes
        endcase
    end
endmodule
