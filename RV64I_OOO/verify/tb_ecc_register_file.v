`timescale 1ns / 1ps
// Isolated unit test for ecc_register_file.v: writes known values through
// both write ports, reads them back cleanly, then uses a testbench-only
// hierarchical procedural write (same one-time-corruption technique
// established in tb_lockstep.v, and for the same reason -- Icarus can't
// force an indexed word of a variable array, and a real bit-flip is a
// one-time event anyway, not a sustained force) to flip one bit directly
// in the DUT's own storage and confirm a subsequent read transparently
// corrects it and raises sbe_fault, then flips a second bit and confirms
// dbe_fault fires instead (uncorrected).
module tb_ecc_register_file;
    reg clk, reset;
    reg reg_write, reg_write2;
    reg [4:0] read_reg1, read_reg2, write_reg, write_reg2;
    reg [63:0] write_data, write_data2;
    wire [63:0] read_data1, read_data2;
    wire sbe_fault, dbe_fault;

    ecc_register_file dut (
        .clk(clk), .reset(reset),
        .reg_write(reg_write), .read_reg1(read_reg1), .read_reg2(read_reg2),
        .write_reg(write_reg), .write_data(write_data),
        .read_data1(read_data1), .read_data2(read_data2),
        .reg_write2(reg_write2), .write_reg2(write_reg2), .write_data2(write_data2),
        .sbe_fault(sbe_fault), .dbe_fault(dbe_fault)
    );

    initial begin clk = 0; forever #5 clk = ~clk; end

    integer errors;

    task do_write;
        input [4:0] reg1; input [63:0] data1;
        input [4:0] reg2; input [63:0] data2;
        begin
            @(negedge clk);
            reg_write = (reg1 != 5'd0); write_reg = reg1; write_data = data1;
            reg_write2 = (reg2 != 5'd0); write_reg2 = reg2; write_data2 = data2;
            @(negedge clk);
            reg_write = 0; reg_write2 = 0;
        end
    endtask

    initial begin
        errors = 0;
        reset = 1; reg_write = 0; reg_write2 = 0;
        read_reg1 = 0; read_reg2 = 0;
        write_reg = 0; write_reg2 = 0; write_data = 0; write_data2 = 0;
        @(negedge clk); @(negedge clk);
        reset = 0;

        // Clean write/read through both ports, no corruption.
        do_write(5'd5, 64'hDEADBEEFCAFEBABE, 5'd6, 64'h1122334455667788);
        read_reg1 = 5'd5; read_reg2 = 5'd6;
        #1;
        if (read_data1 !== 64'hDEADBEEFCAFEBABE || read_data2 !== 64'h1122334455667788 ||
            sbe_fault !== 1'b0 || dbe_fault !== 1'b0) begin
            $display("[FAIL] clean write/read: rd1=%h rd2=%h sbe=%b dbe=%b", read_data1, read_data2, sbe_fault, dbe_fault);
            errors = errors + 1;
        end

        // x0 always reads zero and never reports a fault regardless of
        // its (never-written) underlying storage.
        read_reg1 = 5'd0; read_reg2 = 5'd5;
        #1;
        if (read_data1 !== 64'b0 || sbe_fault !== 1'b0) begin
            $display("[FAIL] x0 read: rd1=%h sbe=%b", read_data1, sbe_fault);
            errors = errors + 1;
        end

        // Single-bit corruption of x5's stored data: flip one data bit
        // directly in the DUT's own array -- a one-time procedural write,
        // not force/release (see tb_lockstep.v's header for why).
        dut.data_mem[5][10] = ~dut.data_mem[5][10];
        read_reg1 = 5'd5; read_reg2 = 5'd0;
        #1;
        if (read_data1 !== 64'hDEADBEEFCAFEBABE || sbe_fault !== 1'b1 || dbe_fault !== 1'b0) begin
            $display("[FAIL] single-bit data corruption: rd1=%h (want DEADBEEFCAFEBABE) sbe=%b (want 1) dbe=%b (want 0)",
                      read_data1, sbe_fault, dbe_fault);
            errors = errors + 1;
        end

        // Single-bit corruption of x6's stored CHECK bits (not data) --
        // must still transparently resolve to the correct data with
        // sbe_fault, since the corrupted bit is itself part of the
        // Hamming codeword.
        dut.check_mem[6][2] = ~dut.check_mem[6][2];
        read_reg1 = 5'd6; read_reg2 = 5'd0;
        #1;
        if (read_data1 !== 64'h1122334455667788 || sbe_fault !== 1'b1 || dbe_fault !== 1'b0) begin
            $display("[FAIL] single-bit check-bit corruption: rd1=%h (want 1122334455667788) sbe=%b (want 1) dbe=%b (want 0)",
                      read_data1, sbe_fault, dbe_fault);
            errors = errors + 1;
        end

        // Double-bit corruption of x5's stored data (already has one bad
        // bit from above; flip a second, different bit): must report
        // dbe_fault, not silently "correct" to a wrong value.
        dut.data_mem[5][10] = ~dut.data_mem[5][10]; // repair first flip
        dut.data_mem[5][20] = ~dut.data_mem[5][20];
        dut.data_mem[5][40] = ~dut.data_mem[5][40];
        read_reg1 = 5'd5; read_reg2 = 5'd0;
        #1;
        if (dbe_fault !== 1'b1) begin
            $display("[FAIL] double-bit corruption: dbe_fault=%b (want 1)", dbe_fault);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("[PASS] tb_ecc_register_file: all checks passed");
        else
            $display("[FAIL] tb_ecc_register_file: %0d error(s)", errors);
        $finish;
    end
endmodule
