

`timescale 1ns / 1ps

module riscv_processor #(
    parameter IMEM_FILE  = "instructions.mem",
    parameter DMEM_FILE  = "data.mem",
    parameter IMEM_WORDS = 4096,
    parameter DMEM_WORDS = 4096
)(
    input clk,
    input reset,
    output wire [31:0] alu_result,
    output wire [31:0] output_mem_read,
    output wire [31:0] pc_out,
    output wire ecall_halt
);
    // Internal wires
    wire [31:0] pc, next_pc;
    wire [31:0] instruction;
    wire [31:0] read_data1, read_data2;      // Data from register file
    wire [31:0] write_back_data;             // Data to write back to register file
    wire [31:0] imm;
    wire [4:0] rs1, rs2, rd;
    wire reg_write, mem_to_reg;

    // Control signals
    wire branch, jump, alu_src, mem_read, mem_write;
    wire [3:0] alu_op;                      // ALU operation code
    wire [2:0] func3;                       // Function code (3 bits)
    wire [6:0] func7;                       // Function code (7 bits)
    wire branch_taken;

    // Memory address wire
    wire [31:0] mem_addr;                   // Memory address

    assign pc_out = pc;
    assign ecall_halt = (instruction[6:0] == 7'b1110011); // ECALL/EBREAK (SYSTEM opcode)

    // Program Counter Module
    Program_counter pc_module (
        .clk(clk),
        .reset(reset),
        .pc_in(next_pc),
        .pc_out(pc)
    );

    // Instruction Fetch Module
    instruction_fetch #(
        .IMEM_FILE(IMEM_FILE),
        .IMEM_WORDS(IMEM_WORDS)
    ) if_module (
        .clk(clk),
        .pc(pc),
        .instruction(instruction)
    );

    // Instruction Decode and Control Unit Module (combinational)
    Instruction_decode_control_unit id_cu_module (
        .instruction(instruction),          // Input instruction
        .rs1(rs1),                          // Source register 1
        .rs2(rs2),                          // Source register 2
        .rd(rd),                            // Destination register
        .imm(imm),                          // Immediate value
        .alu_src(alu_src),                  // ALU source signal
        .branch(branch),                    // Branch control signal
        .jump(jump),                        // Jump control signal
        .reg_write(reg_write),              // Register write control signal
        .mem_read(mem_read),                // Memory read control signal
        .mem_write(mem_write),              // Memory write control signal
        .mem_to_reg(mem_to_reg),            // Memory-to-register control signal
        .alu_op(alu_op),                    // ALU operation code
        .func3(func3),                      // Function code 3 bits
        .func7(func7)                       // Function code 7 bits
    );

    // Unified Register File Module
    register_rw regfile (
        .clk(clk),
        .reset(reset),
        .reg_write(reg_write),              // Write enable signal
        .read_reg1(rs1),                    // Read register 1 address
        .read_reg2(rs2),                    // Read register 2 address
        .write_reg(rd),                     // Write register address
        .write_data(write_back_data),       // Data to write back to register file
        .read_data1(read_data1),            // Output data for rs1
        .read_data2(read_data2)             // Output data for rs2
    );

    // Execute Module
    execute1 ex_module (
        .opcode(instruction[6:0]),          // Extract opcode from instruction
        .func3(func3),                      // Selects branch/load/store variant
        .rs1_data(read_data1),              // Data from rs1 in register file
        .rs2_data(read_data2),              // Data from rs2 in register file
        .imm(imm),
        .pc(pc),
        .rd(rd),
        .alu_src(alu_src),
        .alu_op(alu_op),
        .alu_result(alu_result),
        .mem_addr(mem_addr),
        .next_pc(next_pc),
        .branch_taken(branch_taken)
    );

    // Data Memory Module
    data_memory #(
        .DMEM_FILE(DMEM_FILE),
        .DMEM_WORDS(DMEM_WORDS)
    ) dmem (
        .clk(clk),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .func3(func3),                      // Selects byte/halfword/word access
        .mem_addr(mem_addr),
        .write_data(read_data2),            // Store data comes from rs2
        .read_data(output_mem_read)
    );

    // Register write-back mux: LOAD results come from memory, everything
    // else (R/I-type ALU results, JAL/JALR return address, LUI/AUIPC) comes
    // from the execute stage.
    assign write_back_data = mem_to_reg ? output_mem_read : alu_result;

endmodule
