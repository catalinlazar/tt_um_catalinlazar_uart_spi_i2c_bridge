`default_nettype none
`timescale 1ns/1ps

// 8N1 UART receiver, 16x oversampling, majority-vote mid-bit sample.
module uart_rx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tick_x16,
    input  wire       rxd,

    output reg        data_valid,   // one-cycle pulse when a byte is ready
    output reg [7:0]  data,
    output reg        frame_err
);

    localparam ST_IDLE  = 3'd0,
               ST_START = 3'd1,
               ST_DATA  = 3'd2,
               ST_STOP  = 3'd3;

    reg [2:0] state;
    reg [3:0] samp_cnt;   // 0..15 within a bit period
    reg [2:0] bit_idx;
    reg [7:0] shift;
    reg       rxd_sync0, rxd_sync1;

    // 2-flop synchronizer for the async rx pin
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rxd_sync0 <= 1'b1;
            rxd_sync1 <= 1'b1;
        end else begin
            rxd_sync0 <= rxd;
            rxd_sync1 <= rxd_sync0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            samp_cnt   <= 4'd0;
            bit_idx    <= 3'd0;
            shift      <= 8'd0;
            data       <= 8'd0;
            data_valid <= 1'b0;
            frame_err  <= 1'b0;
        end else begin
            data_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (!rxd_sync1) begin // falling edge -> possible start bit
                        state    <= ST_START;
                        samp_cnt <= 4'd0;
                    end
                end

                ST_START: begin
                    if (tick_x16) begin
                        if (samp_cnt == 4'd7) begin // mid of start bit
                            if (!rxd_sync1) begin
                                samp_cnt <= 4'd0;
                                bit_idx  <= 3'd0;
                                state    <= ST_DATA;
                            end else begin
                                state <= ST_IDLE; // glitch, not a real start
                            end
                        end else begin
                            samp_cnt <= samp_cnt + 4'd1;
                        end
                    end
                end

                ST_DATA: begin
                    if (tick_x16) begin
                        if (samp_cnt == 4'd15) begin
                            samp_cnt      <= 4'd0;
                            shift[bit_idx] <= rxd_sync1;
                            if (bit_idx == 3'd7) begin
                                state <= ST_STOP;
                            end else begin
                                bit_idx <= bit_idx + 3'd1;
                            end
                        end else begin
                            samp_cnt <= samp_cnt + 4'd1;
                        end
                    end
                end

                ST_STOP: begin
                    if (tick_x16) begin
                        if (samp_cnt == 4'd15) begin
                            data       <= shift;
                            data_valid <= 1'b1;
                            frame_err  <= !rxd_sync1; // stop bit should be high
                            state      <= ST_IDLE;
                            samp_cnt   <= 4'd0;
                        end else begin
                            samp_cnt <= samp_cnt + 4'd1;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
