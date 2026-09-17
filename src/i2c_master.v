`default_nettype none
`timescale 1ns/1ps

// I2C master byte-level engine, open-drain SCL/SDA with clock-stretch
// support. The caller (cmd_interp) sequences full transactions by issuing
// one primitive command at a time:
//   CMD_START   : issue START (or repeated-START if already active)
//   CMD_STOP    : issue STOP
//   CMD_WR_BYTE : shift out tx_data MSB-first, then sample slave ACK
//   CMD_RD_BYTE : shift in a byte into rx_data, then drive ack_out
//                 (0 = ACK/continue, 1 = NACK/last byte) onto SDA
//
// SCL/SDA are modeled as open-drain: *_oe=1 drives the line low, *_oe=0
// releases it (external pull-ups required, standard practice for I2C on
// Tiny Tapeout uio pins).
module i2c_master (
    input  wire       clk,
    input  wire       rst_n,

    input  wire [4:0] clkdiv,     // per-phase divider; SCL period ~= 4*(clkdiv+1) sys clocks

    input  wire [1:0] cmd,        // 00=START 01=STOP 10=WR_BYTE 11=RD_BYTE
    input  wire       cmd_valid,  // pulse to launch cmd (only when !busy)
    input  wire [7:0] tx_data,
    input  wire       ack_out,    // for RD_BYTE: ack(0)/nack(1) to send

    output reg  [7:0] rx_data,
    output reg        ack_in,     // for WR_BYTE: slave's ack(0)/nack(1)
    output wire       busy,
    output reg        done,       // one-cycle pulse

    // pad-facing open-drain control
    output reg        scl_oe,
    output reg        sda_oe,
    input  wire        scl_in,
    input  wire        sda_in
);

    localparam CMD_START = 2'b00,
               CMD_STOP  = 2'b01,
               CMD_WR    = 2'b10,
               CMD_RD    = 2'b11;

    localparam ST_IDLE     = 4'd0,
               ST_START_A  = 4'd1,   // SDA=1,SCL=1 setup
               ST_START_B  = 4'd2,   // SDA falls (SCL still high)
               ST_START_C  = 4'd3,   // SCL falls
               ST_BIT_LOW  = 4'd4,   // SCL low, present next bit / release for read
               ST_BIT_RISE = 4'd5,   // release SCL, wait for high (stretch)
               ST_BIT_HIGH = 4'd6,   // SCL high, sample point
               ST_BIT_FALL = 4'd7,   // pull SCL low again
               ST_ACK_LOW  = 4'd8,
               ST_ACK_RISE = 4'd9,
               ST_ACK_HIGH = 4'd10,
               ST_ACK_FALL = 4'd11,
               ST_STOP_A   = 4'd12,  // SCL low, SDA low, setup
               ST_STOP_B   = 4'd13,  // release SCL, wait high
               ST_STOP_C   = 4'd14,  // SDA rises while SCL high
               ST_DONE     = 4'd15;

    reg [3:0]  state;
    reg [1:0]  cmd_hold;
    reg [4:0]  clk_cnt;
    reg [2:0]  bit_cnt;
    reg [7:0]  shift_out;
    reg [7:0]  shift_in;
    reg        is_write;   // current byte op is a write

    assign busy = (state != ST_IDLE);

    wire tick = (clk_cnt == clkdiv);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            cmd_hold  <= 2'd0;
            clk_cnt   <= 5'd0;
            bit_cnt   <= 3'd0;
            shift_out <= 8'd0;
            shift_in  <= 8'd0;
            rx_data   <= 8'd0;
            ack_in    <= 1'b0;
            scl_oe    <= 1'b0;   // released (high via pull-up) at reset
            sda_oe    <= 1'b0;
            done      <= 1'b0;
            is_write  <= 1'b0;
        end else begin
            done <= 1'b0;

            if (state != ST_IDLE && state != ST_DONE && !tick) begin
                clk_cnt <= clk_cnt + 5'd1;
            end else begin
                clk_cnt <= 5'd0;

                case (state)
                    ST_IDLE: begin
                        if (cmd_valid) begin
                            cmd_hold <= cmd;
                            case (cmd)
                                CMD_START: state <= ST_START_A;
                                CMD_STOP:  state <= ST_STOP_A;
                                CMD_WR: begin
                                    shift_out <= tx_data;
                                    bit_cnt   <= 3'd0;
                                    is_write  <= 1'b1;
                                    state     <= ST_BIT_LOW;
                                end
                                CMD_RD: begin
                                    shift_in  <= 8'd0;
                                    bit_cnt   <= 3'd0;
                                    is_write  <= 1'b0;
                                    state     <= ST_BIT_LOW;
                                end
                                default: state <= ST_IDLE;
                            endcase
                        end
                    end

                    // ---------------- START ----------------
                    ST_START_A: begin
                        sda_oe <= 1'b0;  // SDA released high
                        scl_oe <= 1'b0;  // SCL released high
                        state  <= ST_START_B;
                    end
                    ST_START_B: begin
                        if (scl_in) begin // wait for SCL actually high
                            sda_oe <= 1'b1; // pull SDA low: START condition
                            state  <= ST_START_C;
                        end
                    end
                    ST_START_C: begin
                        scl_oe <= 1'b1; // pull SCL low
                        state  <= ST_DONE;
                    end

                    // ---------------- 8 data bits ----------------
                    ST_BIT_LOW: begin
                        scl_oe <= 1'b1; // SCL low
                        if (is_write)
                            sda_oe <= ~shift_out[7]; // drive 0 => oe=1; drive 1 => release
                        else
                            sda_oe <= 1'b0; // release SDA, let slave drive
                        state <= ST_BIT_RISE;
                    end
                    ST_BIT_RISE: begin
                        scl_oe <= 1'b0; // release SCL
                        if (scl_in) state <= ST_BIT_HIGH; // else clock-stretch wait
                    end
                    ST_BIT_HIGH: begin
                        if (!is_write)
                            shift_in <= {shift_in[6:0], sda_in};
                        state <= ST_BIT_FALL;
                    end
                    ST_BIT_FALL: begin
                        scl_oe <= 1'b1; // pull SCL low
                        if (is_write) shift_out <= {shift_out[6:0], 1'b0};
                        if (bit_cnt == 3'd7) begin
                            state <= ST_ACK_LOW;
                        end else begin
                            bit_cnt <= bit_cnt + 3'd1;
                            state   <= ST_BIT_LOW;
                        end
                    end

                    // ---------------- ACK/NACK bit ----------------
                    ST_ACK_LOW: begin
                        scl_oe <= 1'b1;
                        if (is_write)
                            sda_oe <= 1'b0;       // release, let slave ACK
                        else
                            sda_oe <= ack_out;    // master drives ack(0)/nack(1)
                        state <= ST_ACK_RISE;
                    end
                    ST_ACK_RISE: begin
                        scl_oe <= 1'b0;
                        if (scl_in) state <= ST_ACK_HIGH;
                    end
                    ST_ACK_HIGH: begin
                        if (is_write) ack_in <= sda_in; // 0=ACK,1=NACK
                        state <= ST_ACK_FALL;
                    end
                    ST_ACK_FALL: begin
                        scl_oe <= 1'b1;
                        rx_data <= shift_in;
                        state   <= ST_DONE;
                    end

                    // ---------------- STOP ----------------
                    ST_STOP_A: begin
                        scl_oe <= 1'b1;
                        sda_oe <= 1'b1; // SDA low, SCL low: setup
                        state  <= ST_STOP_B;
                    end
                    ST_STOP_B: begin
                        scl_oe <= 1'b0; // release SCL
                        if (scl_in) state <= ST_STOP_C;
                    end
                    ST_STOP_C: begin
                        sda_oe <= 1'b0; // release SDA while SCL high: STOP
                        state  <= ST_DONE;
                    end

                    ST_DONE: begin
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end

                    default: state <= ST_IDLE;
                endcase
            end
        end
    end

endmodule
