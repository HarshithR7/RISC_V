`timescale 1ns / 1ps
// Iterative (restoring-division) DIV/DIVU/REM/REMU functional unit for the
// out-of-order core, covering both 64-bit and *W (32-bit) forms via
// is_signed/is_word. Deliberately multi-cycle, not combinational -- a
// direct response to this project's own synthesis finding for the
// single-cycle core (see RV64I/README.md's "Area and frequency" section):
// a single-cycle combinational RV64 divider synthesized to ~149,000 gates
// pre-optimization and was the dominant area cost of the whole core, times
// five once the vector unit's four parallel per-lane dividers are counted.
// An iterative divider trades that area for latency (~34-66 cycles here),
// which is exactly the tradeoff an out-of-order core is built to hide:
// independent instructions can execute and even commit-eligible-complete
// while a divide is still grinding through its iterations behind them at
// the ROB head.
//
// Interface: assert `start` for one cycle (only accepted while !busy);
// `done` pulses for exactly one cycle once `quotient`/`remainder` are
// valid. Standalone module -- no ROB/tag awareness here, that's added by
// whatever wraps this into a reservation-station-fed functional unit.
//
// Divide-by-zero and the one signed-overflow case (MIN_INT / -1) are
// RISC-V-defined results, not traps -- same convention already
// implemented for the scalar M extension (RV64I/src/execute.v) and RVV's
// vdiv/vrem (RV64I/src/vector_alu.v): divide-by-zero yields all-ones
// (unsigned) / -1 (signed) for the quotient and the dividend for the
// remainder; MIN_INT/-1 yields MIN_INT for the quotient and 0 for the
// remainder. Detected and short-circuited before the iterative datapath
// runs at all, not patched onto its output.
module div_fu (
    input clk,
    input reset,
    input start,          // 1-cycle pulse; ignored while busy
    input is_signed,       // 1 = DIV/REM, 0 = DIVU/REMU
    input is_word,         // 1 = DIVW/REMW/DIVUW/REMUW (32-bit), 0 = 64-bit
    input [63:0] dividend,
    input [63:0] divisor,
    output reg busy,
    output reg done,       // 1-cycle pulse when quotient/remainder are valid
    output reg [63:0] quotient,
    output reg [63:0] remainder
);
    localparam IDLE = 2'd0, RUN = 2'd1, FINISH = 2'd2;
    reg [1:0] state;

    // Latched at IDLE->RUN/FINISH transition; used only from RUN/FINISH.
    reg op_is_word;
    reg quotient_neg, remainder_neg;
    reg is_div_by_zero, is_overflow;
    reg [63:0] eff_dividend_reg;   // width-correct sign/zero-extended dividend,
                                    // reused directly as the divide-by-zero
                                    // remainder result and the overflow
                                    // quotient result (both cases are just
                                    // "the dividend back", already in the
                                    // right width-extended form)
    reg [6:0] target_iters;
    reg [6:0] count;

    // Restoring-division datapath. R is one bit wider than the operand
    // width to hold the trial-subtraction's borrow/sign bit cleanly; D is
    // the same width, zero-extended, and never shifts.
    reg [64:0] R;
    reg [63:0] Q;
    reg [64:0] D;

    // ---- Combinational setup, sampled only when IDLE accepts `start` ----
    // Word forms operate on the low 32 bits, sign- or zero-extended per
    // is_signed into a 64-bit "effective" value -- from that point on the
    // 64-bit-wide datapath below handles both widths uniformly except for
    // Q's initial alignment and the iteration count (see below).
    wire [31:0] dividend32 = dividend[31:0];
    wire [31:0] divisor32  = divisor[31:0];
    wire [63:0] eff_dividend = is_word ? (is_signed ? {{32{dividend32[31]}}, dividend32} : {32'b0, dividend32})
                                        : dividend;
    wire [63:0] eff_divisor  = is_word ? (is_signed ? {{32{divisor32[31]}}, divisor32} : {32'b0, divisor32})
                                        : divisor;
    wire [63:0] min_int_w = is_word ? {32'hFFFFFFFF, 32'h80000000} : 64'h8000000000000000;
    wire div_by_zero = (eff_divisor == 64'b0);
    wire overflow    = is_signed && (eff_dividend == min_int_w) && (eff_divisor == 64'hFFFFFFFFFFFFFFFF);

    wire dividend_neg = is_signed && eff_dividend[63];
    wire divisor_neg  = is_signed && eff_divisor[63];

    reg [63:0] abs_dividend, abs_divisor;
    always @(*) begin
        abs_dividend = dividend_neg ? (~eff_dividend + 64'b1) : eff_dividend;
        abs_divisor  = divisor_neg  ? (~eff_divisor  + 64'b1) : eff_divisor;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            quotient <= 64'b0;
            remainder <= 64'b0;
        end else begin
            done <= 1'b0; // pulses for exactly one cycle, in FINISH below
            case (state)
                IDLE: begin
                    if (start) begin
                        busy <= 1'b1;
                        op_is_word    <= is_word;
                        quotient_neg  <= dividend_neg ^ divisor_neg;
                        remainder_neg <= dividend_neg;
                        is_div_by_zero <= div_by_zero;
                        is_overflow    <= overflow;
                        eff_dividend_reg <= eff_dividend;
                        if (div_by_zero || overflow) begin
                            state <= FINISH;
                        end else begin
                            R <= 65'b0;
                            // Left-justify the true-width dividend into the
                            // top of the (always 64-bit) Q register so a
                            // single uniform "shift Q's MSB into R" step
                            // works for both widths: for is_word, the real
                            // 32-bit value must sit at Q[63:32] with
                            // Q[31:0] zero-padded, not at Q[31:0] with the
                            // upper half zero -- otherwise the first 32
                            // shifts would feed *padding* zero bits into R
                            // instead of the actual dividend bits. Verified
                            // by hand-tracing a small (n=2, register=4)
                            // example before trusting this for n=32/64.
                            Q <= is_word ? {abs_dividend[31:0], 32'b0} : abs_dividend;
                            D <= {1'b0, abs_divisor};
                            count <= 7'b0;
                            target_iters <= is_word ? 7'd32 : 7'd64;
                            state <= RUN;
                        end
                    end
                end

                RUN: begin : run_step
                    reg [128:0] shifted;
                    reg [64:0] trial;
                    shifted = {R, Q} << 1;
                    trial = shifted[128:64] - D;
                    if (trial[64]) begin
                        // Trial subtraction borrowed (would go negative):
                        // restore R, clear the new quotient bit.
                        R <= shifted[128:64];
                        Q <= {shifted[63:1], 1'b0};
                    end else begin
                        R <= trial;
                        Q <= {shifted[63:1], 1'b1};
                    end
                    count <= count + 1'b1;
                    if (count == target_iters - 1'b1)
                        state <= FINISH;
                end

                FINISH: begin : finish_step
                    reg [63:0] q_signed, r_signed;
                    done <= 1'b1;
                    busy <= 1'b0;
                    state <= IDLE;
                    if (is_div_by_zero) begin
                        quotient  <= 64'hFFFFFFFFFFFFFFFF;
                        remainder <= eff_dividend_reg;
                    end else if (is_overflow) begin
                        quotient  <= eff_dividend_reg; // == min_int_w for this width
                        remainder <= 64'b0;
                    end else begin
                        // Q holds the unsigned quotient, R[63:0] the
                        // unsigned remainder (both already correctly
                        // narrow -- see the Q-initialization comment
                        // above -- so truncating to the low 32 bits for
                        // word forms is safe and correct).
                        q_signed = quotient_neg  ? (~Q + 64'b1) : Q;
                        r_signed = remainder_neg ? (~R[63:0] + 64'b1) : R[63:0];
                        if (op_is_word) begin
                            quotient  <= {{32{q_signed[31]}}, q_signed[31:0]};
                            remainder <= {{32{r_signed[31]}}, r_signed[31:0]};
                        end else begin
                            quotient  <= q_signed;
                            remainder <= r_signed;
                        end
                    end
                end
            endcase
        end
    end
endmodule
