`timescale 1ns/1ps

module execute_tb;

reg [6:0] opcode;
reg [2:0] func3;
reg [31:0] rs1_data;
reg [31:0] rs2_data;
reg [31:0] imm;
reg [31:0] pc;
reg [4:0] rd;
reg alu_src;
reg [3:0] alu_op;

wire [31:0] alu_result;
wire [31:0] mem_addr;
wire [31:0] next_pc;
wire branch_taken;
wire [4:0] rd_out;

integer errors = 0;

execute1 uut (
    .opcode(opcode),
    .func3(func3),
    .rs1_data(rs1_data),
    .rs2_data(rs2_data),
    .imm(imm),
    .pc(pc),
    .rd(rd),
    .alu_src(alu_src),
    .alu_op(alu_op),
    .alu_result(alu_result),
    .mem_addr(mem_addr),
    .next_pc(next_pc),
    .branch_taken(branch_taken),
    .rd_out(rd_out)
);

task check32(input [255:0] name, input [31:0] got, input [31:0] expected);
    begin
        if (got !== expected) begin
            $display("FAIL: %0s | Expected=%h Got=%h", name, expected, got);
            errors = errors + 1;
        end else begin
            $display("PASS: %0s | %h", name, got);
        end
    end
endtask

task check1(input [255:0] name, input got, input expected);
    begin
        if (got !== expected) begin
            $display("FAIL: %0s | Expected=%b Got=%b", name, expected, got);
            errors = errors + 1;
        end else begin
            $display("PASS: %0s | %b", name, got);
        end
    end
endtask

initial begin
    opcode = 7'b0000000; func3 = 3'b0; rs1_data = 0; rs2_data = 0;
    imm = 0; pc = 0; rd = 0; alu_src = 0; alu_op = 4'b0000;
    #10;

    // ADD (R-type)
    opcode = 7'b0110011; rs1_data = 32'h5; rs2_data = 32'h3; alu_op = 4'b0010; alu_src = 0;
    #10 check32("ADD", alu_result, 32'h8);

    // SUB (R-type)
    rs1_data = 32'h8; rs2_data = 32'h3; alu_op = 4'b1010;
    #10 check32("SUB", alu_result, 32'h5);

    // ANDI (I-type)
    opcode = 7'b0010011; rs1_data = 32'hF; imm = 32'hA; alu_op = 4'b0100; alu_src = 1;
    #10 check32("ANDI", alu_result, 32'hA);

    // OR (R-type)
    opcode = 7'b0110011; rs1_data = 32'hF; rs2_data = 32'hF0; alu_op = 4'b0101; alu_src = 0;
    #10 check32("OR", alu_result, 32'hFF);

    // SLT: -1 < 1 => 1
    rs1_data = 32'hFFFFFFFF; rs2_data = 32'h1; alu_op = 4'b1011;
    #10 check32("SLT (-1 < 1)", alu_result, 32'h1);

    // SLTU: 0xFFFFFFFF < 1 (unsigned) => 0
    alu_op = 4'b1100;
    #10 check32("SLTU (unsigned)", alu_result, 32'h0);

    // BEQ taken
    opcode = 7'b1100011; func3 = 3'b000; rs1_data = 32'hA; rs2_data = 32'hA; imm = 32'h10;
    #10 begin
        check1("BEQ taken", branch_taken, 1'b1);
        check32("BEQ target", next_pc, pc + imm);
    end

    // BNE not taken (equal operands)
    func3 = 3'b001;
    #10 check1("BNE not taken", branch_taken, 1'b0);

    // BLT signed: -1 < 1 taken
    func3 = 3'b100; rs1_data = 32'hFFFFFFFF; rs2_data = 32'h1;
    #10 check1("BLT signed taken", branch_taken, 1'b1);

    // BLTU unsigned: 0xFFFFFFFF < 1 not taken
    func3 = 3'b110;
    #10 check1("BLTU unsigned not taken", branch_taken, 1'b0);

    // JALR: rs1=0x10, imm=0x5 -> target = 0x14 (LSB cleared), rd = pc+4
    opcode = 7'b1100111; rs1_data = 32'h10; imm = 32'h5; pc = 32'h100;
    #10 begin
        check32("JALR target", next_pc, 32'h14);
        check32("JALR return addr", alu_result, pc + 4);
    end

    // JAL: rd = pc+4
    opcode = 7'b1101111; pc = 32'h200; imm = 32'h20;
    #10 begin
        check32("JAL target", next_pc, pc + imm);
        check32("JAL return addr", alu_result, pc + 4);
    end

    // LUI: result = imm regardless of rs1 garbage value
    opcode = 7'b0110111; rs1_data = 32'hDEADBEEF; imm = 32'hF000_0000; alu_op = 4'b0010; alu_src = 1;
    #10 check32("LUI ignores rs1", alu_result, 32'hF000_0000);

    // AUIPC: result = pc + imm
    opcode = 7'b0010111; pc = 32'h1000; imm = 32'h2000;
    #10 check32("AUIPC", alu_result, 32'h3000);

    // LOAD address calc
    opcode = 7'b0000011; rs1_data = 32'h100; imm = 32'h8;
    #10 check32("LOAD address", mem_addr, 32'h108);

    // STORE address calc
    opcode = 7'b0100011; rs1_data = 32'h200; imm = 32'hC;
    #10 check32("STORE address", mem_addr, 32'h20C);

    #10;
    if (errors == 0)
        $display("execute_tb: ALL TESTS PASSED");
    else
        $display("execute_tb: %0d TEST(S) FAILED", errors);

    $finish;
end

endmodule
