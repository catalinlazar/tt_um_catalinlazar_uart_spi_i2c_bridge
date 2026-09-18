`default_nettype none
`timescale 1ns/1ps

module tb ();

    initial begin
        $dumpfile("tb.vcd");
        $dumpvars(0, tb);
        #1;
    end

    reg        clk;
    reg        rst_n;
    reg        ena;
    reg  [7:0] ui_in;
    reg  [7:0] uio_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // Note: unlike sky130 netlists, the IHP flow's top-level gate-level
    // netlist has no VPWR/VGND ports (power is not modeled as an explicit
    // top-level port in this flow), so GL_TEST doesn't need to wire them.
    tt_um_catalinlazar_uart_spi_i2c_bridge user_project (
        .ui_in  (ui_in),
        .uo_out (uo_out),
        .uio_in (uio_in),
        .uio_out(uio_out),
        .uio_oe (uio_oe),
        .ena    (ena),
        .clk    (clk),
        .rst_n  (rst_n)
    );

endmodule
