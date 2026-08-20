`timescale 1ns / 1ps
// Isolated unit test for instruction_fetch_reg.v's Phase 12
// USE_AXI_MEM=1 path (instruction_fetch_axi.v under the hood): AXI-
// writes 4 halfwords (two 32-bit instructions' worth) directly into the
// memory, standing in for the PS side writing a program image before
// the core ever runs, then confirms fetching at pc=0 and pc=4 (the
// registered read, one cycle after the address is presented -- same
// timing already verified for the $readmemh path in Phase 11's full
// regression) returns the correct halfword-paired instructions -- not
// $readmemh-loaded, not hand-poked, genuinely AXI-loaded content.
module tb_instruction_fetch_axi;
    localparam IMEM_WORDS = 64;

    reg clk;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg [63:0] pc;
    wire [31:0] instruction;
    reg axi_wr_en;
    reg [$clog2(IMEM_WORDS)-1:0] axi_wr_addr;
    reg [15:0] axi_wr_data;

    instruction_fetch_reg #(.IMEM_FILE("unused.mem"), .IMEM_WORDS(IMEM_WORDS), .USE_AXI_MEM(1)) dut (
        .clk(clk), .pc(pc), .instruction(instruction),
        .axi_wr_en(axi_wr_en), .axi_wr_addr(axi_wr_addr), .axi_wr_data(axi_wr_data)
    );

    integer errors;
    integer i;
    reg [15:0] halfwords [0:3];

    initial begin
        errors = 0;
        pc = 0; axi_wr_en = 0; axi_wr_addr = 0; axi_wr_data = 0;
        halfwords[0] = 16'h1111; // instruction 0 (pc=0), low halfword
        halfwords[1] = 16'h2222; // instruction 0, high halfword
        halfwords[2] = 16'h3333; // instruction 1 (pc=4), low halfword
        halfwords[3] = 16'h4444; // instruction 1, high halfword
        @(negedge clk);

        for (i = 0; i < 4; i = i + 1) begin
            axi_wr_en = 1;
            axi_wr_addr = i;
            axi_wr_data = halfwords[i];
            @(negedge clk);
        end
        axi_wr_en = 0;

        pc = 64'h0;
        @(negedge clk); // address settling cycle
        @(negedge clk); // registered read: result valid this cycle
        if (instruction !== 32'h22221111) begin
            $display("[FAIL] AXI-loaded instruction at pc=0: %h (want 22221111)", instruction);
            errors = errors + 1;
        end

        pc = 64'h4;
        @(negedge clk);
        @(negedge clk);
        if (instruction !== 32'h44443333) begin
            $display("[FAIL] AXI-loaded instruction at pc=4: %h (want 44443333)", instruction);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("[PASS] tb_instruction_fetch_axi: all checks passed");
        else
            $display("[FAIL] tb_instruction_fetch_axi: %0d error(s)", errors);
        $finish;
    end
endmodule
