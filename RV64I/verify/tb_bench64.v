`timescale 1ns / 1ps
// Instrumented variant of tb_core64.v for performance benchmarking: same
// pass/fail contract (x31 = 0xFFFF0000 on pass), but also counts, every
// cycle, whether the retiring instruction was a vector instruction and/or
// touched memory, via hierarchical references into the DUT's decode
// outputs (uut.is_vector_op etc) -- the same technique tb_core64.v
// already uses for uut.regfile.registers[31].
//
// This core is single-cycle and never stalls, so instructions retired
// equals cycles elapsed exactly -- CPI is always 1.0 by construction,
// not something that varies per program. That's worth stating plainly
// rather than presenting a CPI column that never moves: the actually
// meaningful comparison this testbench exists to support is cycle count
// (equivalently, instruction count) between a scalar and a vectorized
// version of the same computation.
module tb_bench64 #(
    parameter IMEM_FILE = "test.mem",
    parameter DMEM_FILE = "test_data.mem",
    parameter TEST_NAME = "test",
    parameter integer MAX_CYCLES = 200000
)();
    localparam [63:0] PASS_CODE = 64'hFFFF0000;

    reg clk, reset;
    wire [63:0] alu_result, output_mem_read, pc_out;
    wire ecall_halt;
    integer cycles;
    integer vec_instrs;
    integer mem_ops;
    integer bytes_moved;

    riscv64_processor #(.IMEM_FILE(IMEM_FILE), .DMEM_FILE(DMEM_FILE)) uut (
        .clk(clk), .reset(reset),
        .alu_result(alu_result), .output_mem_read(output_mem_read),
        .pc_out(pc_out), .ecall_halt(ecall_halt)
    );

    initial begin clk = 0; forever #50 clk = ~clk; end
    initial begin
        reset = 1; cycles = 0; vec_instrs = 0; mem_ops = 0; bytes_moved = 0;
        #100; reset = 0;
    end

    wire is_vec = uut.is_vector_op | uut.is_vsetvli | uut.is_vle | uut.is_vse
                | uut.is_vcompare | uut.is_vreduce;
    wire is_scalar_mem = uut.mem_read | uut.mem_write;
    wire is_vec_mem = uut.is_vle | uut.is_vse;
    wire is_mem = is_scalar_mem | is_vec_mem;

    // Real per-access byte width, not an assumed constant: scalar
    // accesses vary (LB/LH/LW/LD, and AMOs -- func3 encodes width
    // exactly the way data_memory.v itself decodes it), vector accesses
    // are always the full 128-bit port (16 bytes) in this scoped
    // implementation (SEW=32, 4 lanes, VLEN=128 -- see RV64I/README.md).
    reg [4:0] scalar_bytes;
    always @(*) begin
        case (uut.func3)
            3'b000, 3'b100: scalar_bytes = 1; // LB/LBU/AMO.B (byte)
            3'b001, 3'b101: scalar_bytes = 2; // LH/LHU/AMO.H (halfword)
            3'b010, 3'b110: scalar_bytes = 4; // LW/LWU/SW/AMO.W (word)
            3'b011:         scalar_bytes = 8; // LD/SD/AMO.D (doubleword)
            default:        scalar_bytes = 0;
        endcase
    end
    wire [4:0] cycle_bytes = is_scalar_mem ? scalar_bytes : (is_vec_mem ? 5'd16 : 5'd0);

    always @(posedge clk) begin
        if (!reset) begin
            cycles = cycles + 1;
            if (is_vec) vec_instrs = vec_instrs + 1;
            if (is_mem) mem_ops = mem_ops + 1;
            bytes_moved = bytes_moved + cycle_bytes;
            if (ecall_halt) begin
                #1;
                if (uut.regfile.registers[31] === PASS_CODE)
                    $display("[PASS] %0s | cycles=%0d instrs=%0d vec_instrs=%0d mem_ops=%0d bytes_moved=%0d",
                              TEST_NAME, cycles, cycles, vec_instrs, mem_ops, bytes_moved);
                else
                    $display("[FAIL] %0s | check #%0d | cycles=%0d",
                              TEST_NAME, uut.regfile.registers[31], cycles);
                $finish;
            end
            if (cycles >= MAX_CYCLES) begin
                $display("[TIMEOUT] %0s | cycles=%0d", TEST_NAME, cycles);
                $finish;
            end
        end
    end
endmodule
