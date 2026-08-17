`timescale 1ns / 1ps

module riscv_processor_tb;

    reg clk;
    reg reset;
    wire [31:0] alu_result;
    wire [31:0] output_mem_read;
    wire [31:0] pc_out;
    wire ecall_halt;

    integer cycles;
    localparam MAX_CYCLES = 2000;

    riscv_processor uut (
        .clk(clk),
        .reset(reset),
        .alu_result(alu_result),
        .output_mem_read(output_mem_read),
        .pc_out(pc_out),
        .ecall_halt(ecall_halt)
    );

    // Clock generation: 100ns period
    initial begin
        clk = 0;
        forever #50 clk = ~clk;
    end

    initial begin
        reset = 1;
        cycles = 0;
        #100;              // hold reset for a full clock period
        reset = 0;
    end

    // Run until the program signals completion (ECALL) or we time out
    always @(posedge clk) begin
        if (!reset) begin
            cycles = cycles + 1;
            if (ecall_halt) begin
                $display("Time=%0t | ECALL encountered at PC=%h after %0d cycles. Halting.",
                          $time, pc_out, cycles);
                #10 $finish;
            end
            if (cycles >= MAX_CYCLES) begin
                $display("Time=%0t | TIMEOUT after %0d cycles at PC=%h (no ECALL reached).",
                          $time, cycles, pc_out);
                $finish;
            end
        end
    end

    initial begin
        $dumpfile("riscv_processor.vcd");
        $dumpvars(0, riscv_processor_tb);
    end

endmodule
