`default_nettype none
`timescale 1ns/1ps

// UART <-> I2C protocol bridge for Tiny Tapeout (1x1 tile).
// (The SPI master was dropped post-synthesis to fit the IHP 1x1 area
// budget -- see README "Area" section for the rationale and history.)
//
// Pin map
// -------
// ui_in[0]   : uart_rx
// ui_in[1]   : reserved / unused (read as 0 internally)
// ui_in[2]   : loopback/self-test enable (1 = internal I2C loopback)
// ui_in[7:3] : reserved / unused (read as 0 internally)
//
// uo_out[0]  : uart_tx
// uo_out[6:1]: reserved / unused (driven low)
// uo_out[7]  : heartbeat/status LED (mirrors I2C busy)
//
// uio[0]     : i2c_scl  (open-drain, needs external pull-up)
// uio[1]     : i2c_sda  (open-drain, needs external pull-up)
// uio[7:2]   : unused (driven as high-Z inputs)
module tt_um_catalinlazar_uart_spi_i2c_bridge (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire        ena,
    input  wire        clk,
    input  wire        rst_n
);

    // -----------------------------------------------------------------
    // Pin aliases
    // -----------------------------------------------------------------
    wire uart_rxd      = ui_in[0];
    wire loopback_en   = ui_in[2];

    wire uart_txd;
    wire heartbeat;

    wire i2c_scl_in = uio_in[0];
    wire i2c_sda_in = uio_in[1];
    wire i2c_scl_oe, i2c_sda_oe;

    assign uo_out = {heartbeat, 6'b0, uart_txd};

    // uio: only [1:0] driven (open-drain I2C), rest are unused inputs
    assign uio_out = {6'b0, 1'b0, 1'b0}; // driving 0 whenever oe=1 (open-drain low)
    assign uio_oe  = {6'b0, i2c_sda_oe, i2c_scl_oe};

    // loopback muxing for standalone self-test (no external I2C device
    // needed): let the I2C engine see its own SDA/SCL. Models an ideal
    // external pull-up: driving low (oe=1) reads back 0, releasing
    // (oe=0) reads back 1 -- so an I2C_WRITE in loopback mode always
    // sees its own address/data bytes ACKed.
    wire i2c_sda_read = loopback_en ? ~i2c_sda_oe   : i2c_sda_in;
    wire i2c_scl_read = loopback_en ? ~i2c_scl_oe   : i2c_scl_in;

    // -----------------------------------------------------------------
    // UART baud generator + RX/TX cores
    // -----------------------------------------------------------------
    wire        tick_x16;
    wire [15:0] baud_div;

    baud_gen #(.DIV_W(16)) u_baud (
        .clk      (clk),
        .rst_n    (rst_n),
        .divisor  (baud_div),
        .tick_x16 (tick_x16)
    );

    wire       rx_byte_valid;
    wire [7:0] rx_byte;
    wire       rx_frame_err;

    uart_rx u_uart_rx (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick_x16   (tick_x16),
        .rxd        (uart_rxd),
        .data_valid (rx_byte_valid),
        .data       (rx_byte),
        .frame_err  (rx_frame_err)
    );

    wire       tx_busy;
    wire       tx_start;
    wire [7:0] tx_start_data;

    uart_tx u_uart_tx (
        .clk      (clk),
        .rst_n    (rst_n),
        .tick_x16 (tick_x16),
        .wr_en    (tx_start),
        .wr_data  (tx_start_data),
        .busy     (tx_busy),
        .txd      (uart_txd)
    );

    // -----------------------------------------------------------------
    // UART RX FIFO (host -> chip) and TX FIFO (chip -> host)
    // -----------------------------------------------------------------
    wire       rxfifo_full;
    wire       rxfifo_empty;
    wire [7:0] rxfifo_rd_data;
    wire       rxfifo_rd_en;
    wire [3:0] rxfifo_count;

    fifo_sync #(.WIDTH(8), .DEPTH(8)) u_rx_fifo (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (rx_byte_valid & !rx_frame_err),
        .wr_data (rx_byte),
        .full    (rxfifo_full),
        .rd_en   (rxfifo_rd_en),
        .rd_data (rxfifo_rd_data),
        .empty   (rxfifo_empty),
        .count   (rxfifo_count)
    );

    wire       txfifo_full;
    wire       txfifo_empty;
    wire [7:0] txfifo_rd_data;
    wire       txfifo_rd_en;
    wire       txfifo_wr_en;
    wire [7:0] txfifo_wr_data;
    wire [3:0] txfifo_count;

    fifo_sync #(.WIDTH(8), .DEPTH(8)) u_tx_fifo (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (txfifo_wr_en),
        .wr_data (txfifo_wr_data),
        .full    (txfifo_full),
        .rd_en   (txfifo_rd_en),
        .rd_data (txfifo_rd_data),
        .empty   (txfifo_empty),
        .count   (txfifo_count)
    );

    // drain TX FIFO into uart_tx whenever it's idle and FIFO has data
    reg tx_pop_pending;
    assign txfifo_rd_en  = !txfifo_empty && !tx_busy && !tx_pop_pending;
    assign tx_start      = tx_pop_pending;
    assign tx_start_data = txfifo_rd_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_pop_pending <= 1'b0;
        end else begin
            if (txfifo_rd_en)
                tx_pop_pending <= 1'b1;
            else if (tx_pop_pending)
                tx_pop_pending <= 1'b0;
        end
    end

    // -----------------------------------------------------------------
    // Register file
    // -----------------------------------------------------------------
    wire        reg_wr_en;
    wire [2:0]  reg_addr;
    wire [7:0]  reg_wr_data;
    wire [7:0]  reg_rd_data;

    wire        cfg_soft_rst, cfg_i2c_en;
    wire [4:0]  cfg_i2c_clkdiv;
    wire        cfg_i2c_stretch_en;

    wire [7:0] status_word;

    regfile u_regfile (
        .clk            (clk),
        .rst_n          (rst_n),
        .wr_en          (reg_wr_en),
        .addr           (reg_addr),
        .wr_data        (reg_wr_data),
        .rd_data        (reg_rd_data),
        .status_live    (status_word),
        .soft_rst       (cfg_soft_rst),
        .i2c_en         (cfg_i2c_en),
        .i2c_clkdiv     (cfg_i2c_clkdiv),
        .i2c_stretch_en (cfg_i2c_stretch_en),
        .uart_baud_div  (baud_div)
    );

    // -----------------------------------------------------------------
    // I2C master
    // -----------------------------------------------------------------
    wire [1:0] i2c_cmd;
    wire       i2c_cmd_valid;
    wire [7:0] i2c_tx_data;
    wire       i2c_ack_out;
    wire [7:0] i2c_rx_data;
    wire       i2c_ack_in;
    wire       i2c_busy;
    wire       i2c_done;

    i2c_master u_i2c (
        .clk      (clk),
        .rst_n    (rst_n),
        .clkdiv   (cfg_i2c_clkdiv),
        .cmd      (i2c_cmd),
        .cmd_valid(i2c_cmd_valid),
        .tx_data  (i2c_tx_data),
        .ack_out  (i2c_ack_out),
        .rx_data  (i2c_rx_data),
        .ack_in   (i2c_ack_in),
        .busy     (i2c_busy),
        .done     (i2c_done),
        .scl_oe   (i2c_scl_oe),
        .sda_oe   (i2c_sda_oe),
        .scl_in   (i2c_scl_read),
        .sda_in   (i2c_sda_read)
    );

    // -----------------------------------------------------------------
    // Status word: [0]=rxfifo_full [1]=txfifo_empty [2]=i2c_busy
    //              [3]=i2c_last_nack [4]=rx_frame_err [5]=i2c_en
    //              [7:6]=reserved
    // -----------------------------------------------------------------
    reg i2c_last_nack;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            i2c_last_nack <= 1'b0;
        else if (i2c_done && i2c_cmd == 2'b10) // was a WR_BYTE
            i2c_last_nack <= i2c_ack_in;
    end

    assign status_word = {2'b00, cfg_i2c_en, rx_frame_err, i2c_last_nack,
                           i2c_busy, txfifo_empty, rxfifo_full};

    assign heartbeat = i2c_busy;

    // -----------------------------------------------------------------
    // Command interpreter (top-level glue)
    // -----------------------------------------------------------------
    cmd_interp u_cmd (
        .clk           (clk),
        .rst_n         (rst_n),

        .rx_empty      (rxfifo_empty),
        .rx_rd_data    (rxfifo_rd_data),
        .rx_rd_en      (rxfifo_rd_en),

        .tx_full       (txfifo_full),
        .tx_wr_en      (txfifo_wr_en),
        .tx_wr_data    (txfifo_wr_data),

        .reg_wr_en     (reg_wr_en),
        .reg_addr      (reg_addr),
        .reg_wr_data   (reg_wr_data),
        .reg_rd_data   (reg_rd_data),
        .status_bits   (status_word),

        .i2c_cmd       (i2c_cmd),
        .i2c_cmd_valid (i2c_cmd_valid),
        .i2c_tx_data   (i2c_tx_data),
        .i2c_ack_out   (i2c_ack_out),
        .i2c_rx_data   (i2c_rx_data),
        .i2c_ack_in    (i2c_ack_in),
        .i2c_busy      (i2c_busy),
        .i2c_done      (i2c_done)
    );

    wire _unused = &{ena, ui_in[7:3], ui_in[1], uio_in[7:2],
                     cfg_soft_rst, cfg_i2c_stretch_en, 1'b0};

endmodule
