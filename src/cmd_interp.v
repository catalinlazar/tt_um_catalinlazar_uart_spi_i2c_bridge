`default_nettype none
`timescale 1ns/1ps

// Command interpreter: parses UART command frames [OPCODE][LEN][PAYLOAD...],
// drives the SPI/I2C masters and register file, and formats replies back
// onto the UART TX FIFO. See README for the full opcode table.
module cmd_interp (
    input  wire        clk,
    input  wire        rst_n,

    // UART RX FIFO (bytes coming in from the host)
    input  wire        rx_empty,
    input  wire [7:0]  rx_rd_data,
    output reg         rx_rd_en,

    // UART TX FIFO (bytes going back to the host)
    input  wire        tx_full,
    output reg         tx_wr_en,
    output reg  [7:0]  tx_wr_data,

    // Register file
    output reg         reg_wr_en,
    output reg  [3:0]  reg_addr,
    output reg  [7:0]  reg_wr_data,
    input  wire [7:0]  reg_rd_data,
    input  wire [7:0]  status_bits,

    // SPI master
    output reg  [1:0]  spi_cs_sel,
    output reg         spi_start,
    output reg  [7:0]  spi_tx_data,
    input  wire [7:0]  spi_rx_data,
    input  wire        spi_busy,
    input  wire        spi_done,

    // I2C master
    output reg  [1:0]  i2c_cmd,
    output reg         i2c_cmd_valid,
    output reg  [7:0]  i2c_tx_data,
    output reg         i2c_ack_out,
    input  wire [7:0]  i2c_rx_data,
    input  wire        i2c_ack_in,
    input  wire        i2c_busy,
    input  wire        i2c_done
);

    // ---------------- opcodes ----------------
    localparam OP_SPI_WRITE = 8'h01,
               OP_SPI_READ  = 8'h02,
               OP_SPI_XFER  = 8'h03,
               OP_I2C_WRITE = 8'h10,
               OP_I2C_READ  = 8'h11,
               OP_CFG_SPI   = 8'h20,
               OP_CFG_I2C   = 8'h21,
               OP_REG_WRITE = 8'h30,
               OP_REG_READ  = 8'h31,
               OP_STATUS    = 8'hF0,
               OP_RESET     = 8'hFF;

    localparam STATUS_OK   = 8'h00,
               STATUS_NACK = 8'h01,
               STATUS_ERR  = 8'hFF;

    localparam I2C_CMD_START = 2'b00,
               I2C_CMD_STOP  = 2'b01,
               I2C_CMD_WR    = 2'b10,
               I2C_CMD_RD    = 2'b11;

    // fetch-byte destination selector
    localparam D_OPCODE  = 4'd0,
               D_LEN     = 4'd1,
               D_SPI_CS  = 4'd2,
               D_SPI_N   = 4'd3,
               D_SPI_DAT = 4'd4,
               D_I2C_ADDR= 4'd5,
               D_I2C_N   = 4'd6,
               D_I2C_DAT = 4'd7,
               D_REG_ADDR= 4'd8,
               D_REG_DAT = 4'd9,
               D_CFG1    = 4'd10,
               D_CFG2    = 4'd11;

    // ---------------- state ----------------
    localparam
        S_FETCH_ISSUE     = 7'd0,
        S_FETCH_CAP       = 7'd1,
        S_FETCH_WAIT      = 7'd32,
        S_DISPATCH        = 7'd2,
        S_PUSH_BYTE       = 7'd3,
        S_AFTER_OPCODE    = 7'd31,

        S_SPI_AFTER_CS    = 7'd4,
        S_SPI_AFTER_N     = 7'd5,
        S_SPI_LOOP_CHECK  = 7'd6,
        S_SPI_START       = 7'd7,
        S_SPI_WAIT_DONE   = 7'd8,
        S_SPI_INC         = 7'd9,

        S_I2C_AFTER_ADDR  = 7'd10,
        S_I2C_AFTER_N     = 7'd11,
        S_I2C_START_WAIT  = 7'd12,
        S_I2C_ADDR_CMD    = 7'd13,
        S_I2C_ADDR_WAIT   = 7'd14,
        S_I2C_LOOP_CHECK  = 7'd15,
        S_I2C_DATA_CMD    = 7'd16,
        S_I2C_DATA_WAIT   = 7'd17,
        S_I2C_INC         = 7'd18,
        S_I2C_STOP_CMD    = 7'd19,
        S_I2C_STOP_WAIT   = 7'd20,

        S_REGW_AFTER_ADDR = 7'd21,
        S_REGW_DO         = 7'd22,

        S_REGR_DO         = 7'd23,
        S_REGR_PUSH_DATA  = 7'd24,

        S_CFGSPI_AFTER1   = 7'd25,
        S_CFGSPI_DO       = 7'd26,

        S_CFGI2C_AFTER1   = 7'd27,
        S_CFGI2C_DO       = 7'd28,

        S_RESET_DO        = 7'd29,

        S_FINISH          = 7'd30;

    reg [6:0] state, fetch_return, push_return;
    reg [3:0] fetch_dest;
    reg [7:0] push_byte;

    reg [7:0] opcode;
    reg [1:0] cur_cs;
    reg [7:0] n_bytes, byte_idx;
    reg [7:0] spi_tx_byte;

    reg [6:0] cur_addr7;
    reg       cur_rw;         // 0 = write, 1 = read
    reg [7:0] i2c_tx_byte;

    reg [3:0] cur_reg_addr;
    reg [7:0] cur_reg_data;
    reg [7:0] cfg_byte1;
    reg [7:0] cfg_byte2;

    wire i2c_last_byte = (byte_idx == n_bytes - 8'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_FETCH_ISSUE;
            fetch_return  <= S_AFTER_OPCODE;
            fetch_dest    <= D_OPCODE;
            push_return   <= S_FINISH;
            push_byte     <= 8'd0;

            rx_rd_en      <= 1'b0;
            tx_wr_en      <= 1'b0;
            tx_wr_data    <= 8'd0;

            reg_wr_en     <= 1'b0;
            reg_addr      <= 4'd0;
            reg_wr_data   <= 8'd0;

            spi_cs_sel    <= 2'd0;
            spi_start     <= 1'b0;
            spi_tx_data   <= 8'd0;

            i2c_cmd       <= 2'd0;
            i2c_cmd_valid <= 1'b0;
            i2c_tx_data   <= 8'd0;
            i2c_ack_out   <= 1'b0;

            opcode        <= 8'd0;
            cur_cs        <= 2'd0;
            n_bytes       <= 8'd0;
            byte_idx      <= 8'd0;
            spi_tx_byte   <= 8'd0;
            cur_addr7     <= 7'd0;
            cur_rw        <= 1'b0;
            i2c_tx_byte   <= 8'd0;
            cur_reg_addr  <= 4'd0;
            cur_reg_data  <= 8'd0;
            cfg_byte1     <= 8'd0;
        end else begin
            // defaults: pulses de-assert unless explicitly set below
            rx_rd_en      <= 1'b0;
            tx_wr_en      <= 1'b0;
            reg_wr_en     <= 1'b0;
            spi_start     <= 1'b0;
            i2c_cmd_valid <= 1'b0;

            case (state)

                // ---- generic byte fetch from UART RX FIFO ----
                S_FETCH_ISSUE: begin
                    if (!rx_empty) begin
                        rx_rd_en <= 1'b1;
                        state    <= S_FETCH_WAIT;
                    end
                end
                S_FETCH_WAIT: begin
                    // bubble cycle: fifo_sync registers rd_data one cycle
                    // after rd_en, so wait here before capturing it.
                    state <= S_FETCH_CAP;
                end
                S_FETCH_CAP: begin
                    case (fetch_dest)
                        D_OPCODE:   opcode       <= rx_rd_data;
                        D_LEN:      ; // discarded, framing only
                        D_SPI_CS:   cur_cs       <= rx_rd_data[1:0];
                        D_SPI_N:    n_bytes      <= rx_rd_data;
                        D_SPI_DAT:  spi_tx_byte  <= rx_rd_data;
                        D_I2C_ADDR: cur_addr7    <= rx_rd_data[6:0];
                        D_I2C_N:    n_bytes      <= rx_rd_data;
                        D_I2C_DAT:  i2c_tx_byte  <= rx_rd_data;
                        D_REG_ADDR: cur_reg_addr <= rx_rd_data[3:0];
                        D_REG_DAT:  cur_reg_data <= rx_rd_data;
                        D_CFG1:     cfg_byte1    <= rx_rd_data;
                        D_CFG2:     cfg_byte2    <= rx_rd_data;
                        default: ;
                    endcase
                    state <= fetch_return;
                end

                // ---- generic byte push to UART TX FIFO ----
                S_PUSH_BYTE: begin
                    if (!tx_full) begin
                        tx_wr_en   <= 1'b1;
                        tx_wr_data <= push_byte;
                        state      <= push_return;
                    end
                end

                // ---- opcode captured, consume (discard) the LEN byte ----
                S_AFTER_OPCODE: begin
                    fetch_dest   <= D_LEN;
                    fetch_return <= S_DISPATCH;
                    state        <= S_FETCH_ISSUE;
                end

                // ---- opcode + len fetched, dispatch ----
                S_DISPATCH: begin
                    byte_idx <= 8'd0;
                    case (opcode)
                        OP_SPI_WRITE, OP_SPI_READ, OP_SPI_XFER: begin
                            fetch_dest   <= D_SPI_CS;
                            fetch_return <= S_SPI_AFTER_CS;
                            state        <= S_FETCH_ISSUE;
                        end
                        OP_I2C_WRITE, OP_I2C_READ: begin
                            cur_rw       <= (opcode == OP_I2C_READ);
                            fetch_dest   <= D_I2C_ADDR;
                            fetch_return <= S_I2C_AFTER_ADDR;
                            state        <= S_FETCH_ISSUE;
                        end
                        OP_REG_WRITE: begin
                            fetch_dest   <= D_REG_ADDR;
                            fetch_return <= S_REGW_AFTER_ADDR;
                            state        <= S_FETCH_ISSUE;
                        end
                        OP_REG_READ: begin
                            fetch_dest   <= D_REG_ADDR;
                            fetch_return <= S_REGR_DO;
                            state        <= S_FETCH_ISSUE;
                        end
                        OP_CFG_SPI: begin
                            fetch_dest   <= D_CFG1;
                            fetch_return <= S_CFGSPI_AFTER1;
                            state        <= S_FETCH_ISSUE;
                        end
                        OP_CFG_I2C: begin
                            fetch_dest   <= D_CFG1;
                            fetch_return <= S_CFGI2C_AFTER1;
                            state        <= S_FETCH_ISSUE;
                        end
                        OP_STATUS: begin
                            push_byte   <= status_bits;
                            push_return <= S_FINISH;
                            state       <= S_PUSH_BYTE;
                        end
                        OP_RESET: begin
                            state <= S_RESET_DO;
                        end
                        default: begin
                            push_byte   <= STATUS_ERR;
                            push_return <= S_FINISH;
                            state       <= S_PUSH_BYTE;
                        end
                    endcase
                end

                // =========================================================
                // SPI_WRITE / SPI_READ / SPI_XFER
                // =========================================================
                S_SPI_AFTER_CS: begin
                    fetch_dest   <= D_SPI_N;
                    fetch_return <= S_SPI_AFTER_N;
                    state        <= S_FETCH_ISSUE;
                end
                S_SPI_AFTER_N: begin
                    push_byte   <= STATUS_OK;
                    push_return <= S_SPI_LOOP_CHECK;
                    state       <= S_PUSH_BYTE;
                end
                S_SPI_LOOP_CHECK: begin
                    if (byte_idx == n_bytes) begin
                        state <= S_FINISH;
                    end else if (opcode == OP_SPI_READ) begin
                        spi_tx_byte <= 8'd0; // dummy clock-out
                        state       <= S_SPI_START;
                    end else begin // WRITE or XFER: need the data byte
                        fetch_dest   <= D_SPI_DAT;
                        fetch_return <= S_SPI_START;
                        state        <= S_FETCH_ISSUE;
                    end
                end
                S_SPI_START: begin
                    spi_cs_sel  <= cur_cs;
                    spi_tx_data <= spi_tx_byte;
                    spi_start   <= 1'b1;
                    state       <= S_SPI_WAIT_DONE;
                end
                S_SPI_WAIT_DONE: begin
                    if (spi_done) begin
                        if (opcode == OP_SPI_READ || opcode == OP_SPI_XFER) begin
                            push_byte   <= spi_rx_data;
                            push_return <= S_SPI_INC;
                            state       <= S_PUSH_BYTE;
                        end else begin
                            state <= S_SPI_INC;
                        end
                    end
                end
                S_SPI_INC: begin
                    byte_idx <= byte_idx + 8'd1;
                    state    <= S_SPI_LOOP_CHECK;
                end

                // =========================================================
                // I2C_WRITE / I2C_READ
                // =========================================================
                S_I2C_AFTER_ADDR: begin
                    fetch_dest   <= D_I2C_N;
                    fetch_return <= S_I2C_AFTER_N;
                    state        <= S_FETCH_ISSUE;
                end
                S_I2C_AFTER_N: begin
                    i2c_cmd       <= I2C_CMD_START;
                    i2c_cmd_valid <= 1'b1;
                    state         <= S_I2C_START_WAIT;
                end
                S_I2C_START_WAIT: begin
                    if (i2c_done) begin
                        i2c_cmd       <= I2C_CMD_WR;
                        i2c_tx_data   <= {cur_addr7, cur_rw};
                        i2c_cmd_valid <= 1'b1;
                        state         <= S_I2C_ADDR_WAIT;
                    end
                end
                S_I2C_ADDR_WAIT: begin
                    if (i2c_done) begin
                        push_byte   <= i2c_ack_in ? STATUS_NACK : STATUS_OK;
                        push_return <= i2c_ack_in ? S_I2C_STOP_CMD : S_I2C_LOOP_CHECK;
                        state       <= S_PUSH_BYTE;
                    end
                end
                S_I2C_LOOP_CHECK: begin
                    if (byte_idx == n_bytes) begin
                        state <= S_I2C_STOP_CMD;
                    end else if (cur_rw == 1'b0) begin // write: need data byte
                        fetch_dest   <= D_I2C_DAT;
                        fetch_return <= S_I2C_DATA_CMD;
                        state        <= S_FETCH_ISSUE;
                    end else begin // read
                        state <= S_I2C_DATA_CMD;
                    end
                end
                S_I2C_DATA_CMD: begin
                    if (cur_rw == 1'b0) begin
                        i2c_cmd     <= I2C_CMD_WR;
                        i2c_tx_data <= i2c_tx_byte;
                    end else begin
                        i2c_cmd     <= I2C_CMD_RD;
                        i2c_ack_out <= i2c_last_byte; // NACK the final byte
                    end
                    i2c_cmd_valid <= 1'b1;
                    state         <= S_I2C_DATA_WAIT;
                end
                S_I2C_DATA_WAIT: begin
                    if (i2c_done) begin
                        if (cur_rw == 1'b1) begin
                            push_byte   <= i2c_rx_data;
                            push_return <= S_I2C_INC;
                            state       <= S_PUSH_BYTE;
                        end else begin
                            state <= S_I2C_INC; // ignore mid-stream nack, keep simple
                        end
                    end
                end
                S_I2C_INC: begin
                    byte_idx <= byte_idx + 8'd1;
                    state    <= S_I2C_LOOP_CHECK;
                end
                S_I2C_STOP_CMD: begin
                    i2c_cmd       <= I2C_CMD_STOP;
                    i2c_cmd_valid <= 1'b1;
                    state         <= S_I2C_STOP_WAIT;
                end
                S_I2C_STOP_WAIT: begin
                    if (i2c_done) state <= S_FINISH;
                end

                // =========================================================
                // REG_WRITE / REG_READ
                // =========================================================
                S_REGW_AFTER_ADDR: begin
                    fetch_dest   <= D_REG_DAT;
                    fetch_return <= S_REGW_DO;
                    state        <= S_FETCH_ISSUE;
                end
                S_REGW_DO: begin
                    reg_wr_en   <= 1'b1;
                    reg_addr    <= cur_reg_addr;
                    reg_wr_data <= cur_reg_data;
                    push_byte   <= STATUS_OK;
                    push_return <= S_FINISH;
                    state       <= S_PUSH_BYTE;
                end

                S_REGR_DO: begin
                    reg_addr    <= cur_reg_addr;
                    push_byte   <= STATUS_OK;
                    push_return <= S_REGR_PUSH_DATA;
                    state       <= S_PUSH_BYTE;
                end
                S_REGR_PUSH_DATA: begin
                    push_byte   <= reg_rd_data;
                    push_return <= S_FINISH;
                    state       <= S_PUSH_BYTE;
                end

                // =========================================================
                // CFG_SPI / CFG_I2C
                // =========================================================
                S_CFGSPI_AFTER1: begin
                    fetch_dest   <= D_CFG2;
                    fetch_return <= S_CFGSPI_DO;
                    state        <= S_FETCH_ISSUE;
                end
                S_CFGSPI_DO: begin
                    reg_wr_en   <= 1'b1;
                    reg_addr    <= 4'd1;
                    // cfg_byte2 holds mode: bit0=cpol, bit1=cpha
                    reg_wr_data <= {2'b00, cfg_byte2[1], cfg_byte2[0], cfg_byte1[3:0]};
                    push_byte   <= STATUS_OK;
                    push_return <= S_FINISH;
                    state       <= S_PUSH_BYTE;
                end

                S_CFGI2C_AFTER1: begin
                    fetch_dest   <= D_CFG2;
                    fetch_return <= S_CFGI2C_DO;
                    state        <= S_FETCH_ISSUE;
                end
                S_CFGI2C_DO: begin
                    reg_wr_en   <= 1'b1;
                    reg_addr    <= 4'd2;
                    reg_wr_data <= {2'b00, cfg_byte2[0], cfg_byte1[4:0]};
                    push_byte   <= STATUS_OK;
                    push_return <= S_FINISH;
                    state       <= S_PUSH_BYTE;
                end

                // ---- RESET ----
                S_RESET_DO: begin
                    reg_wr_en   <= 1'b1;
                    reg_addr    <= 4'd0;
                    reg_wr_data <= 8'h01; // soft_rst bit, self-clears in regfile
                    push_byte   <= STATUS_OK;
                    push_return <= S_FINISH;
                    state       <= S_PUSH_BYTE;
                end

                S_FINISH: begin
                    fetch_dest   <= D_OPCODE;
                    fetch_return <= S_AFTER_OPCODE;
                    state        <= S_FETCH_ISSUE;
                end

                default: state <= S_FETCH_ISSUE;
            endcase
        end
    end

endmodule
