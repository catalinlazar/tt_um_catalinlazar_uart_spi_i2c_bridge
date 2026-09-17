`default_nettype none
`timescale 1ns/1ps

// 8N1 UART transmitter, driven off tick_x16 (counts 16 ticks per bit).
module uart_tx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tick_x16,

    input  wire        wr_en,     // pulse to load a new byte (must be idle)
    input  wire [7:0]  wr_data,
    output wire         busy,

    output reg         txd
);

    localparam ST_IDLE  = 2'd0,
               ST_START = 2'd1,
               ST_DATA  = 2'd2,
               ST_STOP  = 2'd3;

    reg [1:0] state;
    reg [3:0] tick_cnt;
    reg [2:0] bit_idx;
    reg [7:0] shift;

    assign busy = (state != ST_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            tick_cnt <= 4'd0;
            bit_idx  <= 3'd0;
            shift    <= 8'd0;
            txd      <= 1'b1;
        end else begin
            case (state)
                ST_IDLE: begin
                    txd <= 1'b1;
                    if (wr_en) begin
                        shift    <= wr_data;
                        state    <= ST_START;
                        tick_cnt <= 4'd0;
                    end
                end

                ST_START: begin
                    txd <= 1'b0;
                    if (tick_x16) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            bit_idx  <= 3'd0;
                            state    <= ST_DATA;
                        end else begin
                            tick_cnt <= tick_cnt + 4'd1;
                        end
                    end
                end

                ST_DATA: begin
                    txd <= shift[bit_idx];
                    if (tick_x16) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            if (bit_idx == 3'd7) begin
                                state <= ST_STOP;
                            end else begin
                                bit_idx <= bit_idx + 3'd1;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 4'd1;
                        end
                    end
                end

                ST_STOP: begin
                    txd <= 1'b1;
                    if (tick_x16) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            state    <= ST_IDLE;
                        end else begin
                            tick_cnt <= tick_cnt + 4'd1;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
