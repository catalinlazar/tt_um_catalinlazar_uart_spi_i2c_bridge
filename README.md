# UART-SPI-I2C Bridge (Tiny Tapeout, IHP 130nm, 1x2)

A fully digital UART-controlled protocol bridge: send commands over UART
and the chip drives an SPI master (4x chip-select, configurable
CPOL/CPHA) or an I2C master (clock stretching, 7-bit addressing) on your
behalf. Includes a command interpreter, FIFO buffering on every
interface, a memory-mapped register file, and a built-in loopback
self-test mode so the whole design is testable standalone with nothing
but a UART connection.

This is the full original feature set. It didn't fit an IHP 1x1 tile,
so the project moved to a 1x2 tile instead of cutting SPI or narrowing
the protocol permanently — see "Design history" below for how that was
worked out and what was tried along the way.

## Repo layout

```
src/      RTL (see info.yaml source_files for build order)
test/     cocotb testbench (tb.v + test.py) run with Icarus Verilog
docs/     extra documentation
info.yaml Tiny Tapeout project descriptor (pinout, top module, etc.)
```

## Architecture

```
UART RX ─► RX FIFO ─► cmd_interp (FSM) ─► regfile
                             │                │
                             ├──► SPI master ◄┤ (clkdiv/cpol/cpha)
                             │                │
                             └──► I2C master ◄┘ (clkdiv)
UART TX ◄─ TX FIFO ◄─────────┘
```

- `fifo_sync.v` — generic 8-deep synchronous FIFO (used for UART RX/TX)
- `baud_gen.v` — programmable 16x-oversample tick generator
- `uart_rx.v` / `uart_tx.v` — 8N1 UART
- `spi_master.v` — full-duplex SPI, configurable CPOL/CPHA, clock
  divider, 4 independent active-low chip-selects
- `i2c_master.v` — byte-level I2C engine (START/STOP/WR_BYTE/RD_BYTE
  primitives) with clock-stretch support, open-drain SCL/SDA
- `regfile.v` — 8 x 8-bit memory-mapped registers
- `cmd_interp.v` — parses UART frames and sequences SPI/I2C transactions
- `tt_um_catalinlazar_uart_spi_i2c_bridge.v` — top level / pin mapping

## Pinout

| Pin | Direction | Function |
|---|---|---|
| `ui_in[0]` | in | `uart_rx` |
| `ui_in[1]` | in | `spi_miso` |
| `ui_in[2]` | in | `loopback_test_en` — 1 loops SPI mosi→miso internally for standalone self-test |
| `ui_in[7:3]` | in | unused |
| `uo_out[0]` | out | `uart_tx` |
| `uo_out[1]` | out | `spi_sclk` |
| `uo_out[2]` | out | `spi_mosi` |
| `uo_out[6:3]` | out | `spi_cs_n[3:0]` (active low) |
| `uo_out[7]` | out | heartbeat LED (SPI/I2C busy) |
| `uio[0]` | bidir | `i2c_scl` (open-drain, **needs an external pull-up**) |
| `uio[1]` | bidir | `i2c_sda` (open-drain, **needs an external pull-up**) |
| `uio[7:2]` | bidir | unused |

## UART command protocol

Every command frame is `[OPCODE][LEN][PAYLOAD...]`. Every reply starts
with a 1-byte status code (`0x00`=OK, `0x01`=I2C NACK, `0xFF`=error/bad
opcode), followed by any response bytes.

| Opcode | Name | Payload | Reply (after status byte) |
|---|---|---|---|
| `0x01` | SPI_WRITE | `[cs][n][data...]` | — |
| `0x02` | SPI_READ | `[cs][n]` | `n` bytes clocked in |
| `0x03` | SPI_XFER | `[cs][n][data...]` | `n` bytes (full duplex) |
| `0x10` | I2C_WRITE | `[addr7][n][data...]` | — |
| `0x11` | I2C_READ | `[addr7][n]` | `n` bytes (if ACKed) |
| `0x20` | CFG_SPI | `[clkdiv][mode]` (mode bit0=cpol, bit1=cpha) | — |
| `0x21` | CFG_I2C | `[clkdiv][options]` (bit0=stretch_en, informational) | — |
| `0x30` | REG_WRITE | `[addr][data]` | — |
| `0x31` | REG_READ | `[addr]` | 1 byte |
| `0xF0` | STATUS | — | 1 status byte (see register map) |
| `0xFF` | RESET | — | — |

`cs` selects one of the 4 SPI chip-selects (0-3). `n` is 0-255. `LEN` is
present in every frame for host-side framing but the interpreter derives
payload sizes from the opcode-specific fields above.

## Register map (addr, via REG_READ/REG_WRITE)

| Addr | Name | Bits |
|---|---|---|
| 0x00 | CTRL | [0] soft_rst (self-clearing) [1] spi_en [2] i2c_en |
| 0x01 | SPI_CFG | [3:0] clkdiv [4] cpol [5] cpha |
| 0x02 | I2C_CFG | [4:0] clkdiv [5] stretch_en |
| 0x03 | STATUS | read-only: [0] rxfifo_full [1] txfifo_empty [2] spi_busy [3] i2c_busy [4] i2c_last_nack [5] rx_frame_err [6] spi_en [7] i2c_en |
| 0x04 | IRQ (reserved) | — |
| 0x05 | BAUD_LO | UART baud divisor bits [7:0] |
| 0x06 | BAUD_HI | UART baud divisor bits [15:8] |
| 0x07 | scratch | general purpose |

UART bit rate = `clk_hz / (16 * (baud_div + 1))`. Reset default is
`baud_div = 64`, i.e. 9600 baud assuming a 10 MHz project clock — adjust
`clock_hz` in `info.yaml` and the default in `regfile.v` if your test
harness clocks the design differently.

## Standalone self-test (no external SPI/I2C hardware needed)

Hold `ui_in[2]` high. `SPI_XFER` will then echo back exactly what you
send (mosi is looped to miso internally) — a full functional check of
the SPI engine and command interpreter. `I2C_WRITE`/`I2C_READ` will
reliably report NACK (no real slave is present, and the loopback models
an idle pulled-up bus), which exercises START/ADDRESS/STOP sequencing
end-to-end and is easy to verify on a logic analyzer or in simulation.

## Running the testbench

```bash
cd test
pip install -r requirements.txt
make
```

This uses [cocotb](https://www.cocotb.org/) with Icarus Verilog to bit-bang
UART commands into the design and check the replies (register
read/write, SPI loopback echo, I2C NACK detection, STATUS/RESET). View
`test/tb.vcd` in GTKWave to inspect waveforms.

## Hardening / GDS

This repo follows the standard [Tiny Tapeout](https://tinytapeout.com)
flow: `info.yaml` drives the automated OpenLane hardening via Tiny
Tapeout's GitHub Actions (`.github/workflows/`), so no separate
place-and-route config is checked in here. Push to GitHub and check the
Actions tab for the hardening run; see the Tiny Tapeout docs for adding
the standard workflow files if this repo doesn't already have them from
the project template.

## Design history

This design is flop-heavy — the command interpreter's FSM, the FIFOs,
and the register file dominate cell area, not combinational logic — and
the full SPI+I2C feature set does not fit an IHP **1x1** tile
(158.9% global-placement utilization at synthesis). The project went
through several rounds of area-reduction chasing 1x1 before settling on
moving to a **1x2** tile instead, restoring everything cut along the way.
In order:

1. FIFOs trimmed 8-deep → 4-deep → 2-deep; unused IRQ_EN register
   dropped; `cmd_interp`'s opcode-specific payload registers
   consolidated (mutually-exclusive fields like `spi_tx_byte` /
   `i2c_tx_byte` / `cur_reg_data` sharing one physical register instead
   of four, since only one opcode is ever in flight).
2. Even after those cuts, utilization was still ~128%, so **the SPI
   master was dropped entirely** and I2C kept in its place — reasoned
   as the more useful lab-bench interface (2 pins vs. SPI's 7 with 4x
   CS, and a better match for common hobbyist/lab I2C sensor, RTC,
   EEPROM and display breakout boards). This got utilization to ~110%.
3. Clearing the 100% placement-density check on 1x1 still wasn't
   enough: near 100% utilization, `OpenROAD` couldn't honor its target
   density (forced to 1.0, zero slack) and Clock Tree Synthesis's
   post-processing hold-violation repair pass — which needs real spare
   area to insert fix-up buffers — kept failing legalization. Further
   cuts (FIFOs down to a 1-entry skid buffer, I2C burst length narrowed
   from 255 to 15 bytes, baud divider narrowed from 16 to 11 bits,
   `cmd_interp`'s `fetch_return`/`push_return` registers eliminated in
   favor of deriving the resume state combinationally) got utilization
   to ~81% but never fully closed the gap — hold-violation repair
   consistently needed ~30% bonus area that a near-full tile didn't
   have room for. CTS clock-tree tuning (tighter sink clustering,
   buffer spacing) helped some of this but plateaued.
4. Rather than keep trading away capability (I2C burst length, buffer
   depth, and ultimately SPI) for a 1x1 tile that kept failing on hold
   timing regardless, **moved to a 1x2 tile** and restored everything:
   full SPI master, 8-deep FIFOs, 0-255 byte I2C bursts, full 16-bit
   baud divisor. Verified clean: all 80 hardening stages complete,
   DRC/LVS/Antenna pass, and signoff STA reports zero setup, hold,
   slew, or cap violations, at 71.4% utilization with real margin to
   spare.

The internal restructuring from steps 1 and 3 (register sharing between
mutually-exclusive opcode fields, and deriving `cmd_interp`'s FSM resume
states from a small tag instead of storing them) has no protocol or
usability impact either way, so it was kept even after moving to 1x2.

See git history for the measured utilization at each step.
