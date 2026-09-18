`default_nettype none
`timescale 1ns/1ps

// Command interpreter: parses UART command frames [OPCODE][LEN][PAYLOAD...],
// drives the I2C master and register file, and formats replies back
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
    output reg  [2:0]  reg_addr,
    output reg  [7:0]  reg_wr_data,
    input  wire [7:0]  reg_rd_data,
    input  wire [7:0]  status_bits,

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
    localparam OP_I2C_WRITE = 8'h10,
               OP_I2C_READ  = 8'h11,
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

    // fetch-byte destination selector. Every value here has exactly one
    // valid "resume" state (REG_ADDR is split write/read so that stays
    // true), so the resume state is derived combinationally from this
    // tag below (fetch_next) instead of being stored in its own register.
    localparam D_OPCODE    = 4'd0,
               D_LEN       = 4'd1,
               D_I2C_ADDR  = 4'd2,
               D_I2C_N     = 4'd3,
               D_I2C_DAT   = 4'd4,
               D_REGW_ADDR = 4'd5,
               D_REG_DAT   = 4'd6,
               D_CFG1      = 4'd7,
               D_CFG2      = 4'd8,
               D_REGR_ADDR = 4'd9;

    // push-byte destination selector -- same idea as fetch_dest above,
    // for the "resume" state after S_PUSH_BYTE.
    localparam PD_FINISH        = 3'd0,
               PD_I2C_ACK       = 3'd1,
               PD_I2C_NACK      = 3'd2,
               PD_I2C_READ_DATA = 3'd3,
               PD_REGR_STATUS   = 3'd4;

    // ---------------- state ----------------
    localparam
        S_FETCH_ISSUE     = 5'd0,
        S_FETCH_CAP       = 5'd1,
        S_FETCH_WAIT      = 5'd2,
        S_DISPATCH        = 5'd3,
        S_PUSH_BYTE       = 5'd4,
        S_AFTER_OPCODE    = 5'd5,

        S_I2C_AFTER_ADDR  = 5'd6,
        S_I2C_AFTER_N     = 5'd7,
        S_I2C_START_WAIT  = 5'd8,
        S_I2C_ADDR_WAIT   = 5'd9,
        S_I2C_LOOP_CHECK  = 5'd10,
        S_I2C_DATA_CMD    = 5'd11,
        S_I2C_DATA_WAIT   = 5'd12,
        S_I2C_INC         = 5'd13,
        S_I2C_STOP_CMD    = 5'd14,
        S_I2C_STOP_WAIT   = 5'd15,

        S_REGW_AFTER_ADDR = 5'd16,
        S_REGW_DO         = 5'd17,

        S_REGR_DO         = 5'd18,
        S_REGR_PUSH_DATA  = 5'd19,

        S_CFGI2C_AFTER1   = 5'd20,
        S_CFGI2C_DO       = 5'd21,

        S_RESET_DO        = 5'd22,

        S_FINISH          = 5'd23;

    reg [4:0] state;
    reg [3:0] fetch_dest;
    reg [2:0] push_dest;
    reg [7:0] push_byte;

    reg [7:0] opcode;
    reg [7:0] n_bytes, byte_idx;  // max I2C burst: 255 bytes

    // Opcode-specific payload/context fields never coexist -- only one
    // opcode is in flight at a time -- so they share physical storage
    // instead of each getting a dedicated register:
    reg [7:0] i2c_ctx;    // {cur_addr7, cur_rw} for the I2C transaction in flight
    reg [7:0] payload_a;  // i2c_tx_byte / cur_reg_data / cfg_byte1
    reg [7:0] payload_b;  // cur_reg_addr in [2:0] / cfg_byte2

    wire i2c_last_byte = (byte_idx == n_bytes - 8'd1);

    // resume state after a fetch, derived from fetch_dest (see comment above)
    reg [4:0] fetch_next;
    always @(*) begin
        case (fetch_dest)
            D_OPCODE:    fetch_next = S_AFTER_OPCODE;
            D_LEN:       fetch_next = S_DISPATCH;
            D_I2C_ADDR:  fetch_next = S_I2C_AFTER_ADDR;
            D_I2C_N:     fetch_next = S_I2C_AFTER_N;
            D_I2C_DAT:   fetch_next = S_I2C_DATA_CMD;
            D_REGW_ADDR: fetch_next = S_REGW_AFTER_ADDR;
            D_REG_DAT:   fetch_next = S_REGW_DO;
            D_CFG1:      fetch_next = S_CFGI2C_AFTER1;
            D_CFG2:      fetch_next = S_CFGI2C_DO;
            D_REGR_ADDR: fetch_next = S_REGR_DO;
            default:     fetch_next = S_FETCH_ISSUE;
        endcase
    end

    // resume state after a push, derived from push_dest (see comment above)
    reg [4:0] push_next;
    always @(*) begin
        case (push_dest)
            PD_FINISH:        push_next = S_FINISH;
            PD_I2C_ACK:       push_next = S_I2C_LOOP_CHECK;
            PD_I2C_NACK:      push_next = S_I2C_STOP_CMD;
            PD_I2C_READ_DATA: push_next = S_I2C_INC;
            PD_REGR_STATUS:   push_next = S_REGR_PUSH_DATA;
            default:          push_next = S_FINISH;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_FETCH_ISSUE;
            fetch_dest    <= D_OPCODE;
            push_dest     <= PD_FINISH;
            push_byte     <= 8'd0;

            rx_rd_en      <= 1'b0;
            tx_wr_en      <= 1'b0;
            tx_wr_data    <= 8'd0;

            reg_wr_en     <= 1'b0;
            reg_addr      <= 3'd0;
            reg_wr_data   <= 8'd0;

            i2c_cmd       <= 2'd0;
            i2c_cmd_valid <= 1'b0;
            i2c_tx_data   <= 8'd0;
            i2c_ack_out   <= 1'b0;

            opcode        <= 8'd0;
            n_bytes       <= 8'd0;
            byte_idx      <= 8'd0;
            i2c_ctx       <= 8'd0;
            payload_a     <= 8'd0;
            payload_b     <= 8'd0;
        end else begin
            // defaults: pulses de-assert unless explicitly set below
            rx_rd_en      <= 1'b0;
            tx_wr_en      <= 1'b0;
            reg_wr_en     <= 1'b0;
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
                        D_I2C_ADDR: i2c_ctx[7:1] <= rx_rd_data[6:0];
                        D_I2C_N:    n_bytes       <= rx_rd_data;
                        D_I2C_DAT:  payload_a     <= rx_rd_data;
                        D_REGW_ADDR,
                        D_REGR_ADDR: payload_b    <= {5'b0, rx_rd_data[2:0]};
                        D_REG_DAT:  payload_a     <= rx_rd_data;
                        D_CFG1:     payload_a     <= rx_rd_data;
                        D_CFG2:     payload_b     <= rx_rd_data;
                        default: ;
                    endcase
                    state <= fetch_next;
                end

                // ---- generic byte push to UART TX FIFO ----
                S_PUSH_BYTE: begin
                    if (!tx_full) begin
                        tx_wr_en   <= 1'b1;
                        tx_wr_data <= push_byte;
                        state      <= push_next;
                    end
                end

                // ---- opcode captured, consume (discard) the LEN byte ----
                S_AFTER_OPCODE: begin
                    fetch_dest   <= D_LEN;
                    state        <= S_FETCH_ISSUE;
                end

                // ---- opcode + len fetched, dispatch ----
                S_DISPATCH: begin
                    byte_idx <= 8'd0;
                    case (opcode)
                        OP_I2C_WRITE, OP_I2C_READ: begin
                            i2c_ctx[0]   <= (opcode == OP_I2C_READ);
                            fetch_dest   <= D_I2C_ADDR;
                            state        <= S_FETCH_ISSUE;
                        end
                        OP_REG_WRITE: begin
                            fetch_dest   <= D_REGW_ADDR;
                            state        <= S_FETCH_ISSUE;
                        end
                        OP_REG_READ: begin
                            fetch_dest   <= D_REGR_ADDR;
                            state        <= S_FETCH_ISSUE;
                        end
                        OP_CFG_I2C: begin
                            fetch_dest   <= D_CFG1;
                            state        <= S_FETCH_ISSUE;
                        end
                        OP_STATUS: begin
                            push_byte   <= status_bits;
                            push_dest   <= PD_FINISH;
                            state       <= S_PUSH_BYTE;
                        end
                        OP_RESET: begin
                            state <= S_RESET_DO;
                        end
                        default: begin
                            push_byte   <= STATUS_ERR;
                            push_dest   <= PD_FINISH;
                            state       <= S_PUSH_BYTE;
                        end
                    endcase
                end

                // =========================================================
                // I2C_WRITE / I2C_READ
                // =========================================================
                S_I2C_AFTER_ADDR: begin
                    fetch_dest   <= D_I2C_N;
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
                        i2c_tx_data   <= i2c_ctx;
                        i2c_cmd_valid <= 1'b1;
                        state         <= S_I2C_ADDR_WAIT;
                    end
                end
                S_I2C_ADDR_WAIT: begin
                    if (i2c_done) begin
                        push_byte   <= i2c_ack_in ? STATUS_NACK : STATUS_OK;
                        push_dest   <= i2c_ack_in ? PD_I2C_NACK : PD_I2C_ACK;
                        state       <= S_PUSH_BYTE;
                    end
                end
                S_I2C_LOOP_CHECK: begin
                    if (byte_idx == n_bytes) begin
                        state <= S_I2C_STOP_CMD;
                    end else if (i2c_ctx[0] == 1'b0) begin // write: need data byte
                        fetch_dest   <= D_I2C_DAT;
                        state        <= S_FETCH_ISSUE;
                    end else begin // read
                        state <= S_I2C_DATA_CMD;
                    end
                end
                S_I2C_DATA_CMD: begin
                    if (i2c_ctx[0] == 1'b0) begin
                        i2c_cmd     <= I2C_CMD_WR;
                        i2c_tx_data <= payload_a;
                    end else begin
                        i2c_cmd     <= I2C_CMD_RD;
                        i2c_ack_out <= i2c_last_byte; // NACK the final byte
                    end
                    i2c_cmd_valid <= 1'b1;
                    state         <= S_I2C_DATA_WAIT;
                end
                S_I2C_DATA_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_ctx[0] == 1'b1) begin
                            push_byte   <= i2c_rx_data;
                            push_dest   <= PD_I2C_READ_DATA;
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
                    state        <= S_FETCH_ISSUE;
                end
                S_REGW_DO: begin
                    reg_wr_en   <= 1'b1;
                    reg_addr    <= payload_b[2:0];
                    reg_wr_data <= payload_a;
                    push_byte   <= STATUS_OK;
                    push_dest   <= PD_FINISH;
                    state       <= S_PUSH_BYTE;
                end

                S_REGR_DO: begin
                    reg_addr    <= payload_b[2:0];
                    push_byte   <= STATUS_OK;
                    push_dest   <= PD_REGR_STATUS;
                    state       <= S_PUSH_BYTE;
                end
                S_REGR_PUSH_DATA: begin
                    push_byte   <= reg_rd_data;
                    push_dest   <= PD_FINISH;
                    state       <= S_PUSH_BYTE;
                end

                // =========================================================
                // CFG_I2C
                // =========================================================
                S_CFGI2C_AFTER1: begin
                    fetch_dest   <= D_CFG2;
                    state        <= S_FETCH_ISSUE;
                end
                S_CFGI2C_DO: begin
                    reg_wr_en   <= 1'b1;
                    reg_addr    <= 3'd2;
                    reg_wr_data <= {2'b00, payload_b[0], payload_a[4:0]};
                    push_byte   <= STATUS_OK;
                    push_dest   <= PD_FINISH;
                    state       <= S_PUSH_BYTE;
                end

                // ---- RESET ----
                S_RESET_DO: begin
                    reg_wr_en   <= 1'b1;
                    reg_addr    <= 3'd0;
                    reg_wr_data <= 8'h01; // soft_rst bit, self-clears in regfile
                    push_byte   <= STATUS_OK;
                    push_dest   <= PD_FINISH;
                    state       <= S_PUSH_BYTE;
                end

                S_FINISH: begin
                    fetch_dest   <= D_OPCODE;
                    state        <= S_FETCH_ISSUE;
                end

                default: state <= S_FETCH_ISSUE;
            endcase
        end
    end

endmodule
