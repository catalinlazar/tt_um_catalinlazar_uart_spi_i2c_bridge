# How it works

A UART command interpreter drives an I2C master (clock stretching,
7-bit addressing) over a shared 8-bit memory-mapped register file. UART
RX/TX are single-byte buffered, enough to absorb one byte of get-ahead
while `cmd_interp` is mid-transaction, though not a burst. See the
top-level `README.md` for the full opcode table, register map, and
pinout (including the SPI-removal and area history in the "Area"
section).

# How to test

Hold `ui_in[2]` (loopback_test_en) high and issue an `I2C_WRITE`
(opcode `0x10`) command over UART at 9600 baud (default): with no real
I2C slave present, the loopback bus reliably replies with a NACK status
byte, which exercises the START/ADDRESS/STOP sequencing end-to-end.
Connect a real I2C peripheral (with pull-ups on `uio[0]`/`uio[1]`) and
drop `loopback_test_en` low for full end-to-end testing.

See `test/test.py` for an automated cocotb testbench covering register
read/write, I2C NACK detection, and STATUS/RESET.

# External hardware

- Optional: any I2C peripheral on `uio[0]`/`uio[1]`, with external
  pull-up resistors (required — the design only drives these lines
  open-drain low).
- A USB-UART adapter on `ui_in[0]`/`uo_out[0]` for sending commands.
