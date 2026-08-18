`timescale 1ns / 1ps
// Combinational RV64I+M decode for the out-of-order core -- same
// immediate-generation and control tables as RV64I/src/decode_control_unit.v,
// subsetted to RV64I+M only (no A/C/F/D/V) and restructured to classify
// each instruction by which reservation-station bank it dispatches to
// (is_alu/is_muldiv/is_branch_class/is_load/is_store/is_ecall) instead of
// driving a single-cycle datapath's control signals directly.
module decode_ooo (
    input [31:0] instruction,

    output wire [4:0] rs1,
    output wire [4:0] rs2,
    output wire [4:0] rd,
    output wire [2:0] func3,
    output reg  [63:0] imm,

    output reg is_alu,          // -> ALU RS (includes LUI/AUIPC, computed there)
    output reg is_muldiv,       // -> Mul RS (muldiv_op[2]==0) or Div RS (muldiv_op[2]==1)
    output reg is_branch,       // -> Branch RS, no destination register
    output reg is_jal,          // -> Branch RS, has destination register (return addr)
    output reg is_jalr,         // -> Branch RS, has destination register (return addr)
    output reg is_load,         // -> LSQ
    output reg is_store,        // -> LSQ, no destination register
    output reg is_ecall,        // -> ROB only, zero-latency, no destination register

    output reg [3:0] alu_op,
    output reg word_op,
    output reg [2:0] muldiv_op,
    output reg reg_write,       // this instruction has a destination register (rd, subject to rd==0 exclusion elsewhere)
    output reg src2_is_imm,     // operand2 comes from the immediate, not a renamed rs2
    // LUI/AUIPC's instruction[19:15] is part of the U-type immediate, not
    // a real register field -- dispatch must not treat `rs1` as a source
    // register to rename/wait on for these two opcodes at all. When
    // either flag is set, src1 is immediately ready with a known value
    // (0 for LUI, the current PC for AUIPC) and `rs1` above must be
    // ignored for dependency-tracking purposes.
    output reg src1_is_zero,    // LUI: operand1 = 0 (result = 0 + imm = imm)
    output reg src1_is_pc,      // AUIPC: operand1 = pc

    // Phase 6 (DLP): scoped RVV decode -- .vv-form elementwise
    // arithmetic/logic/min-max/shift/divide only (VADD..VSRA in
    // vector_alu.v's op encoding). Deliberately excludes, and never sets
    // is_vec for: .vx/.vi forms (would need a scalar/immediate operand
    // path into the vector datapath -- a genuine cross-domain dependency
    // this scoped integration avoids entirely), compares (mask-shaped
    // results), reductions (a fold, not an elementwise op -- different
    // execution model), vsetvli (no vl/vtype state tracked -- vl is
    // implicitly fixed at VLMAX=LANES), and vector load/store (no vector
    // LSQ integration). vs1/vs2/vd reuse the same rs1/rs2/rd outputs
    // above -- RVV's operand fields sit at the identical instruction bit
    // positions as the scalar ISA's, so no separate vs1/vs2/vd ports are
    // needed.
    output reg is_vec,
    output reg [4:0] v_op,

    // vmv.v.x (broadcast scalar rs1 into all vector lanes, real RVV
    // encoding) -- added specifically to make the vector pipeline
    // testable without also building vector load/store (deferred, see
    // is_vec's header): with no other way to get non-zero data into a
    // vector register from a hand-assembled program, elementwise vv-form
    // ops alone (0 op 0 = 0, always) can never be verified against real
    // data. is_vmv implies is_vec; the top level treats rs1 as an
    // ordinary scalar source (same rs1/RAT/CDB path as any ALU
    // instruction -- no new decode field needed for it) and broadcasts
    // its resolved value into all lanes once ready.
    output reg is_vmv
);
    assign rs1   = instruction[19:15];
    assign rs2   = instruction[24:20];
    assign rd    = instruction[11:7];
    assign func3 = instruction[14:12];
    wire [6:0] opcode = instruction[6:0];
    wire [6:0] func7  = instruction[31:25];

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
    localparam SYSTEM    = 7'b1110011;
    localparam OP_V      = 7'b1010111;

    localparam VADD=5'd0, VSUB=5'd1, VAND=5'd2, VOR=5'd3, VXOR=5'd4, VMUL=5'd5,
               VMIN=5'd12, VMINU=5'd13, VMAX=5'd14, VMAXU=5'd15,
               VDIVU=5'd16, VDIV=5'd17, VREMU=5'd18, VREM=5'd19,
               VSLL=5'd20, VSRL=5'd21, VSRA=5'd22;

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

    always @(*) begin
        case (opcode)
            LOAD, I_TYPE, JALR, OP_IMM_32:
                imm = {{52{instruction[31]}}, instruction[31:20]};
            S_TYPE:
                imm = {{52{instruction[31]}}, instruction[31:25], instruction[11:7]};
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

    always @(*) begin
        is_alu = 0; is_muldiv = 0; is_branch = 0; is_jal = 0; is_jalr = 0;
        is_load = 0; is_store = 0; is_ecall = 0;
        alu_op = ALU_ADD; word_op = 0; muldiv_op = 3'b000;
        reg_write = 0; src2_is_imm = 0; src1_is_zero = 0; src1_is_pc = 0;
        is_vec = 0; v_op = 5'd0; is_vmv = 0;

        case (opcode)
            R_TYPE: begin
                reg_write = 1;
                if (func7 == RVM_FUNCT7) begin
                    is_muldiv = 1;
                    muldiv_op = func3;
                end else begin
                    is_alu = 1;
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
                        default: alu_op = ALU_ADD;
                    endcase
                end
            end

            OP_32: begin
                reg_write = 1;
                word_op = 1;
                if (func7 == RVM_FUNCT7) begin
                    is_muldiv = 1;
                    muldiv_op = func3;
                end else begin
                    is_alu = 1;
                    case ({func7, func3})
                        10'b0000000_000: alu_op = ALU_ADD;
                        10'b0100000_000: alu_op = ALU_SUB;
                        10'b0000000_001: alu_op = ALU_SLL;
                        10'b0000000_101: alu_op = ALU_SRL;
                        10'b0100000_101: alu_op = ALU_SRA;
                        default: alu_op = ALU_ADD;
                    endcase
                end
            end

            I_TYPE: begin
                reg_write = 1; is_alu = 1; src2_is_imm = 1;
                case (func3)
                    3'b000: alu_op = ALU_ADD;
                    3'b010: alu_op = ALU_SLT;
                    3'b011: alu_op = ALU_SLTU;
                    3'b100: alu_op = ALU_XOR;
                    3'b110: alu_op = ALU_OR;
                    3'b111: alu_op = ALU_AND;
                    3'b001: alu_op = ALU_SLL;
                    3'b101: alu_op = (instruction[31:26] == 6'b010000) ? ALU_SRA : ALU_SRL;
                    default: alu_op = ALU_ADD;
                endcase
            end

            OP_IMM_32: begin
                reg_write = 1; is_alu = 1; src2_is_imm = 1; word_op = 1;
                case (func3)
                    3'b000: alu_op = ALU_ADD;
                    3'b001: alu_op = ALU_SLL;
                    3'b101: alu_op = (func7 == 7'b0100000) ? ALU_SRA : ALU_SRL;
                    default: alu_op = ALU_ADD;
                endcase
            end

            LOAD:  begin reg_write = 1; is_load = 1; end
            S_TYPE: is_store = 1;

            B_TYPE: is_branch = 1;
            JAL:  begin reg_write = 1; is_jal = 1; end
            JALR: begin reg_write = 1; is_jalr = 1; src2_is_imm = 1; end

            LUI_TYPE:  begin reg_write = 1; is_alu = 1; src2_is_imm = 1; alu_op = ALU_ADD; src1_is_zero = 1; end
            AUIPC:     begin reg_write = 1; is_alu = 1; src2_is_imm = 1; alu_op = ALU_ADD; src1_is_pc = 1; end

            SYSTEM: begin
                // ECALL/EBREAK -- this scoped core only needs the halt
                // signal (asserted at commit/retire, see riscv64_ooo_proc.v).
                // Also routed through the ALU bank as a trivial, always-
                // ready ADD(0,0)=0 op (src1_is_zero + src2_is_imm with
                // imm=0, the default for an opcode with no imm-generation
                // case above) -- reuses the existing CDB-broadcast-marks-
                // ROB-entry-done path uniformly instead of needing a
                // second, separately-arbitrated "mark done" port on
                // rob.v just for this one zero-latency case. reg_write
                // stays 0 (ECALL never writes a register), so this
                // produces a harmless, unused, discarded ALU result.
                is_ecall = 1;
                is_alu = 1;
                src1_is_zero = 1;
                src2_is_imm = 1;
            end

            // Scoped RVV: .vv form only (func3=000 OPIVV, func3=010
            // OPMVV), integer arithmetic/logic/min-max/shift/mul/div
            // group only -- see is_vec's own port comment for everything
            // deliberately excluded. reg_write is NOT set here (vd lives
            // in the vector register file, not the scalar one -- the top
            // level uses is_vec/reg_write together to route renaming and
            // commit to the correct RAT/register file).
            OP_V: begin
                if (func3 == 3'b000) begin
                    // OPIVV: integer arithmetic/logic/min-max/shift.
                    is_vec = 1;
                    case (instruction[31:26]) // funct6
                        6'b000000: v_op = VADD;
                        6'b000010: v_op = VSUB;
                        6'b001001: v_op = VAND;
                        6'b001010: v_op = VOR;
                        6'b001011: v_op = VXOR;
                        6'b000101: v_op = VMIN;
                        6'b000100: v_op = VMINU;
                        6'b000111: v_op = VMAX;
                        6'b000110: v_op = VMAXU;
                        6'b100101: v_op = VSLL;
                        6'b101000: v_op = VSRL;
                        6'b101001: v_op = VSRA;
                        default: is_vec = 0; // unrecognized funct6 (e.g. compares): out of scope
                    endcase
                end else if (func3 == 3'b010 && instruction[31:26] >= 6'b100000) begin
                    // OPMVV, mul/div/rem group only (funct6>=100000
                    // excludes the disjoint low-range reduction encodings
                    // -- see decode_control_unit.v's identical split).
                    is_vec = 1;
                    case (instruction[31:26])
                        6'b100101: v_op = VMUL;
                        6'b100000: v_op = VDIVU;
                        6'b100001: v_op = VDIV;
                        6'b100010: v_op = VREMU;
                        6'b100011: v_op = VREM;
                        default: is_vec = 0;
                    endcase
                end else if (func3 == 3'b100 && instruction[31:26] == 6'b010111) begin
                    // vmv.v.x (real RVV encoding, OPIVX/VRXUNARY0) -- see
                    // is_vmv's own port comment.
                    is_vec = 1;
                    is_vmv = 1;
                end
                // Everything else in OP_V (.vi, other OPIVX/OPMVX forms,
                // OPMVV's reduction range, vsetvli): not decoded, is_vec
                // stays 0 -- this instruction simply never dispatches
                // (same safe-failure convention as an unrecognized
                // opcode).
            end

            default: ; // unrecognized: no register/memory effect
        endcase
    end
endmodule
