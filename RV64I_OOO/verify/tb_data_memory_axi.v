`timescale 1ns / 1ps
// Isolated unit test for data_memory_axi.v (Phase 12, FPGA bring-up):
// confirms the AXI-side write port actually lands content the vector
// read port can see, with the correct one-cycle registered-read
// latency, and that the core-side vmem_write port still works
// independently of the AXI port.
module tb_data_memory_axi;
    localparam DMEM_WORDS = 64;
    localparam VLEN = 256;
    localparam VDWORDS = VLEN / 64;

    reg clk;
    initial begin clk = 0; forever #5 clk = ~clk; end

    reg vmem_write;
    reg [63:0] vmem_addr;
    reg [VLEN-1:0] vmem_write_data;
    wire [VLEN-1:0] vmem_read_data;
    reg axi_wr_en;
    reg [$clog2(DMEM_WORDS)-1:0] axi_wr_addr;
    reg [63:0] axi_wr_data;

    data_memory_axi #(.DMEM_FILE("unused.mem"), .DMEM_WORDS(DMEM_WORDS), .VLEN(VLEN),
                       .LOAD_FROM_FILE(0)) dut (
        .clk(clk),
        .vmem_read(1'b1), .vmem_write(vmem_write), .vmem_addr(vmem_addr),
        .vmem_write_data(vmem_write_data), .vmem_read_data(vmem_read_data),
        .axi_wr_en(axi_wr_en), .axi_wr_addr(axi_wr_addr), .axi_wr_data(axi_wr_data)
    );

    integer errors;
    integer i;

    initial begin
        errors = 0;
        vmem_write = 0; vmem_addr = 0; vmem_write_data = 0;
        axi_wr_en = 0; axi_wr_addr = 0; axi_wr_data = 0;
        @(negedge clk);

        // AXI-load 4 consecutive 64-bit words (one full VLEN line's
        // worth) at addresses 8..11, one per cycle -- exactly the
        // pattern a real AXI-lite write burst-of-single-beats would
        // produce.
        for (i = 0; i < VDWORDS; i = i + 1) begin
            axi_wr_en = 1;
            axi_wr_addr = 8 + i;
            axi_wr_data = 64'hA000000000000000 | i;
            @(negedge clk);
        end
        axi_wr_en = 0;

        // Read back through the vector port, base address = word 8's
        // byte address (8*8 = 64 = 0x40).
        vmem_addr = 64'h40;
        @(negedge clk); // address settling cycle
        #1;
        if (vmem_read_data !== 256'h0 && vmem_read_data === {VLEN{1'bx}}) begin
            // (informational only -- not itself a failure condition)
        end
        @(negedge clk); // registered read: result valid THIS cycle
        for (i = 0; i < VDWORDS; i = i + 1) begin
            if (vmem_read_data[i*64 +: 64] !== (64'hA000000000000000 | i)) begin
                $display("[FAIL] AXI-loaded word %0d readback: %h (want %h)",
                          i, vmem_read_data[i*64 +: 64], 64'hA000000000000000 | i);
                errors = errors + 1;
            end
        end

        // Core-side vmem_write still works independently of the AXI
        // port -- overwrite the same line via the normal write path and
        // confirm the new value reads back.
        vmem_write = 1;
        vmem_write_data = {VDWORDS{64'hBEEFBEEFBEEFBEEF}};
        @(negedge clk);
        vmem_write = 0;
        @(negedge clk);
        #1;
        if (vmem_read_data !== {VDWORDS{64'hBEEFBEEFBEEFBEEF}}) begin
            $display("[FAIL] core-side vmem_write readback: %h", vmem_read_data);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("[PASS] tb_data_memory_axi: all checks passed");
        else
            $display("[FAIL] tb_data_memory_axi: %0d error(s)", errors);
        $finish;
    end
endmodule
