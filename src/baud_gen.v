`default_nettype none
`timescale 1ns/1ps

// Programmable baud-rate tick generator.
// Produces a single-cycle 'tick_x16' pulse at 16x the baud rate,
// used by both uart_rx (oversampling) and uart_tx (bit timing / 16).
module baud_gen #(
    parameter DIV_W = 16
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire [DIV_W-1:0]  divisor,   // = clk_freq / (baud*16) - 1
    output reg               tick_x16
);

    reg [DIV_W-1:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt      <= {DIV_W{1'b0}};
            tick_x16 <= 1'b0;
        end else begin
            if (cnt == divisor) begin
                cnt      <= {DIV_W{1'b0}};
                tick_x16 <= 1'b1;
            end else begin
                cnt      <= cnt + 1'b1;
                tick_x16 <= 1'b0;
            end
        end
    end

endmodule
