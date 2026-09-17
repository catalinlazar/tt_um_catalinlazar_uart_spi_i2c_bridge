`default_nettype none
`timescale 1ns/1ps

// Simple memory-mapped register file, 16 x 8-bit registers.
// addr 0x00 CTRL      : [0]=soft_rst [1]=spi_en [2]=i2c_en
// addr 0x01 SPI_CFG   : [3:0]=clkdiv [4]=cpol [5]=cpha [7:6]=cs_sel
// addr 0x02 I2C_CFG   : [4:0]=clkdiv [5]=stretch_en(informational)
// addr 0x03 STATUS    : read-only, driven from live status bus
// addr 0x04 IRQ_EN    : reserved for future use
// addr 0x05 BAUD_LO   : UART baud divisor, bits [7:0]
// addr 0x06 BAUD_HI   : UART baud divisor, bits [15:8]
// addr 0x07-0x0F      : general purpose scratch registers
module regfile (
    input  wire       clk,
    input  wire       rst_n,

    input  wire        wr_en,
    input  wire [3:0]  addr,
    input  wire [7:0]  wr_data,
    output reg  [7:0]  rd_data,

    input  wire [7:0]  status_live,  // live status bits, mapped at addr 3

    output wire        soft_rst,
    output wire        spi_en,
    output wire        i2c_en,
    output wire [3:0]  spi_clkdiv,
    output wire        spi_cpol,
    output wire        spi_cpha,
    output wire [1:0]  spi_cs_sel,
    output wire [4:0]  i2c_clkdiv,
    output wire        i2c_stretch_en,
    output wire [15:0] uart_baud_div
);

    reg [7:0] regs [0:15];

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 16; i = i + 1) regs[i] <= 8'd0;
            regs[1] <= 8'h04;  // default spi clkdiv
            regs[2] <= 8'h04;  // default i2c clkdiv
            regs[5] <= 8'd64;  // default baud divisor lo (9600 baud @ 10MHz clk)
            regs[6] <= 8'd0;   // default baud divisor hi
        end else begin
            if (wr_en && addr != 4'd3) begin
                regs[addr] <= wr_data;
            end
            // CTRL[0] soft_rst is self-clearing one cycle after being set
            if (regs[0][0]) regs[0][0] <= 1'b0;
        end
    end

    always @(*) begin
        if (addr == 4'd3)
            rd_data = status_live;
        else
            rd_data = regs[addr];
    end

    assign soft_rst       = regs[0][0];
    assign spi_en         = regs[0][1];
    assign i2c_en         = regs[0][2];
    assign spi_clkdiv     = regs[1][3:0];
    assign spi_cpol       = regs[1][4];
    assign spi_cpha       = regs[1][5];
    assign spi_cs_sel     = regs[1][7:6];
    assign i2c_clkdiv     = regs[2][4:0];
    assign i2c_stretch_en = regs[2][5];
    assign uart_baud_div  = {regs[6], regs[5]};

endmodule
