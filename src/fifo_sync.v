`default_nettype none
`timescale 1ns/1ps

// Generic synchronous FIFO, flip-flop based (fine for small depths on TT).
// DEPTH must be a power of two.
module fifo_sync #(
    parameter WIDTH = 8,
    parameter DEPTH = 16,
    parameter ADDR_W = $clog2(DEPTH)
) (
    input  wire             clk,
    input  wire             rst_n,      // active low

    input  wire             wr_en,
    input  wire [WIDTH-1:0] wr_data,
    output wire             full,

    input  wire             rd_en,
    output reg  [WIDTH-1:0] rd_data,
    output wire             empty,

    output wire [ADDR_W:0]  count       // 0..DEPTH
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_W-1:0] wr_ptr, rd_ptr;
    reg [ADDR_W:0]   cnt;

    wire wr_fire = wr_en && !full;
    wire rd_fire = rd_en && !empty;

    assign full  = (cnt == DEPTH[ADDR_W:0]);
    assign empty = (cnt == {(ADDR_W+1){1'b0}});
    assign count = cnt;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= {ADDR_W{1'b0}};
            rd_ptr <= {ADDR_W{1'b0}};
            cnt    <= {(ADDR_W+1){1'b0}};
            rd_data <= {WIDTH{1'b0}};
            for (i = 0; i < DEPTH; i = i + 1) mem[i] <= {WIDTH{1'b0}};
        end else begin
            if (wr_fire) begin
                mem[wr_ptr] <= wr_data;
                wr_ptr <= wr_ptr + 1'b1;
            end
            if (rd_fire) begin
                rd_data <= mem[rd_ptr];
                rd_ptr <= rd_ptr + 1'b1;
            end
            case ({wr_fire, rd_fire})
                2'b10: cnt <= cnt + 1'b1;
                2'b01: cnt <= cnt - 1'b1;
                default: cnt <= cnt;
            endcase
        end
    end

endmodule
