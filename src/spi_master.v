`default_nettype none
`timescale 1ns/1ps

// SPI master, full-duplex, configurable CPOL/CPHA and clock divider,
// 4 independent active-low chip-select outputs, MSB-first, 8-bit transfers.
//
// Timing model: sclk toggles every (clkdiv+1) system clocks. "phase0" is
// the leading edge (first transition away from idle level), "phase1" is
// the trailing edge (transition back toward idle level).
//   CPHA=0: sample on leading edge,  shift/advance on trailing edge.
//   CPHA=1: sample on trailing edge, shift/advance on trailing edge.
// mosi always presents tx_shift[7]; tx_shift is preloaded with tx_data
// so the first bit is valid before the first (leading) edge in both modes.
module spi_master (
    input  wire       clk,
    input  wire       rst_n,

    input  wire [3:0] clkdiv,     // sclk half-period = (clkdiv+1) sys clocks
    input  wire       cpol,
    input  wire       cpha,
    input  wire [1:0] cs_sel,     // which of the 4 CS lines to assert

    input  wire        start,     // pulse: begin one 8-bit transfer
    input  wire [7:0]  tx_data,
    output reg  [7:0]  rx_data,
    output wire         busy,
    output reg          done,      // one-cycle pulse when rx_data valid

    output wire         sclk,
    output wire         mosi,
    input  wire         miso,
    output wire  [3:0]  cs_n       // active-low, one per device
);

    localparam ST_IDLE = 2'd0,
               ST_XFER = 2'd1,
               ST_DONE = 2'd2;

    reg [1:0]  state;
    reg [3:0]  clk_cnt;
    reg        sclk_int;
    reg        phase;         // 0 = waiting for leading edge, 1 = waiting for trailing
    reg [2:0]  bit_cnt;
    reg [7:0]  tx_shift, rx_shift;
    reg [1:0]  cs_hold;
    reg        active;

    assign busy = (state != ST_IDLE);
    assign sclk = active ? sclk_int : cpol;
    assign mosi = tx_shift[7];
    assign cs_n = active ? ~(4'b0001 << cs_hold) : 4'b1111;

    wire clk_edge = (clk_cnt == clkdiv);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            clk_cnt  <= 4'd0;
            sclk_int <= 1'b0;
            phase    <= 1'b0;
            bit_cnt  <= 3'd0;
            tx_shift <= 8'd0;
            rx_shift <= 8'd0;
            rx_data  <= 8'd0;
            cs_hold  <= 2'd0;
            active   <= 1'b0;
            done     <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    sclk_int <= cpol;
                    if (start) begin
                        tx_shift <= tx_data;
                        cs_hold  <= cs_sel;
                        active   <= 1'b1;
                        bit_cnt  <= 3'd0;
                        clk_cnt  <= 4'd0;
                        phase    <= 1'b0;
                        state    <= ST_XFER;
                    end
                end

                ST_XFER: begin
                    if (clk_edge) begin
                        clk_cnt  <= 4'd0;
                        sclk_int <= ~sclk_int;

                        if (phase == 1'b0) begin
                            // leading edge
                            if (cpha == 1'b0)
                                rx_shift <= {rx_shift[6:0], miso};
                            phase <= 1'b1;
                        end else begin
                            // trailing edge
                            if (cpha == 1'b1)
                                rx_shift <= {rx_shift[6:0], miso};
                            tx_shift <= {tx_shift[6:0], 1'b0};
                            phase <= 1'b0;
                            if (bit_cnt == 3'd7) begin
                                state <= ST_DONE;
                            end else begin
                                bit_cnt <= bit_cnt + 3'd1;
                            end
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 4'd1;
                    end
                end

                ST_DONE: begin
                    rx_data <= rx_shift;
                    done    <= 1'b1;
                    active  <= 1'b0;
                    state   <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
