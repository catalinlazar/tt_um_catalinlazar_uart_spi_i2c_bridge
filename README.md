# UART-I2C Bridge (Tiny Tapeout, IHP 130nm, 1x1)

A fully digital UART-controlled protocol bridge: send commands over UART
and the chip drives an I2C master (clock stretching, 7-bit addressing)
on your behalf. Includes a command interpreter, FIFO buffering on every
interface, a memory-mapped register file, and a built-in loopback
self-test mode so the whole design is testable standalone with nothing
but a UART connection.

An SPI master (4x chip-select, configurable CPOL/CPHA) was part of the
original design; it was dropped post-synthesis to fit the IHP 1x1 tile
area budget. See "Area" below for the history and rationale — I2C was
kept because it's the more useful lab-bench interface (2 pins instead
of SPI's 7 with 4x CS, and far more common on cheap sensor/peripheral
breakout boards).

## Repo layout

```
src/      RTL (see info.yaml source_files for build order)
test/     cocotb testbench (tb.v + test.py) run with Icarus Verilog
docs/     extra documentation
info.yaml Tiny Tapeout project descriptor (pinout, top module, etc.)
```

## Architecture

```
UART RX ─► RX buffer ─► cmd_interp (FSM) ─► regfile
                             │                │
                             └──► I2C master ◄┘ (clkdiv)
UART TX ◄─ TX buffer ◄────────┘
```

- `skid_buffer.v` — single-entry buffer (used for UART RX/TX)
- `baud_gen.v` — programmable 16x-oversample tick generator
- `uart_rx.v` / `uart_tx.v` — 8N1 UART
- `i2c_master.v` — byte-level I2C engine (START/STOP/WR_BYTE/RD_BYTE
  primitives) with clock-stretch support, open-drain SCL/SDA
- `regfile.v` — 8 x 8-bit memory-mapped registers
- `cmd_interp.v` — parses UART frames and sequences I2C transactions
- `tt_um_catalinlazar_uart_spi_i2c_bridge.v` — top level / pin mapping

## Pinout

| Pin | Direction | Function |
|---|---|---|
| `ui_in[0]` | in | `uart_rx` |
| `ui_in[1]` | in | unused (was `spi_miso`) |
| `ui_in[2]` | in | `loopback_test_en` — 1 loops I2C SDA/SCL internally for standalone self-test |
| `ui_in[7:3]` | in | unused |
| `uo_out[0]` | out | `uart_tx` |
| `uo_out[6:1]` | out | unused (was SPI sclk/mosi/cs_n[3:0]), driven low |
| `uo_out[7]` | out | heartbeat LED (I2C busy) |
| `uio[0]` | bidir | `i2c_scl` (open-drain, **needs an external pull-up**) |
| `uio[1]` | bidir | `i2c_sda` (open-drain, **needs an external pull-up**) |
| `uio[7:2]` | bidir | unused |

## UART command protocol

Every command frame is `[OPCODE][LEN][PAYLOAD...]`. Every reply starts
with a 1-byte status code (`0x00`=OK, `0x01`=I2C NACK, `0xFF`=error/bad
opcode), followed by any response bytes.

| Opcode | Name | Payload | Reply (after status byte) |
|---|---|---|---|
| `0x10` | I2C_WRITE | `[addr7][n][data...]` | — |
| `0x11` | I2C_READ | `[addr7][n]` | `n` bytes (if ACKed) |
| `0x21` | CFG_I2C | `[clkdiv][options]` (bit0=stretch_en, informational) | — |
| `0x30` | REG_WRITE | `[addr][data]` | — |
| `0x31` | REG_READ | `[addr]` | 1 byte |
| `0xF0` | STATUS | — | 1 status byte (see register map) |
| `0xFF` | RESET | — | — |

`n` is 0-15 (I2C burst length; narrowed from the original 0-255, then
0-31, to save area). This doesn't cut off any device -- I2C has no
inherent single-transaction size limit, so a bigger read or a
larger-than-15-byte EEPROM page write just becomes multiple back-to-
back `I2C_READ`/`I2C_WRITE` commands from the host (EEPROMs already
impose their own page-size limit that software must respect regardless
of what this bridge allows, so this is adding a second, lower ceiling
on something host software already has to handle). `LEN` is present in
every frame for host-side framing but the interpreter derives payload
sizes from the opcode-specific fields above.

Opcodes `0x01`/`0x02`/`0x03` (SPI_WRITE/READ/XFER) and `0x20` (CFG_SPI)
from the original SPI master are no longer implemented; sending them
now gets the generic `0xFF` error reply like any unrecognized opcode.

## Register map (addr, via REG_READ/REG_WRITE)

| Addr | Name | Bits |
|---|---|---|
| 0x00 | CTRL | [0] soft_rst (self-clearing) [1] reserved [2] i2c_en |
| 0x01 | unimplemented | always reads 0x00, writes ignored (was SPI_CFG) |
| 0x02 | I2C_CFG | [4:0] clkdiv [5] stretch_en |
| 0x03 | STATUS | read-only: [0] rxfifo_full [1] txfifo_empty [2] i2c_busy [3] i2c_last_nack [4] rx_frame_err [5] i2c_en [7:6] reserved |
| 0x04 | IRQ (unimplemented) | always reads 0x00, writes ignored |
| 0x05 | BAUD_LO | UART baud divisor bits [7:0] |
| 0x06 | BAUD_HI | UART baud divisor bits [15:8] |
| 0x07 | scratch | general purpose |

UART bit rate = `clk_hz / (16 * (baud_div + 1))`. Reset default is
`baud_div = 64`, i.e. 9600 baud assuming a 10 MHz project clock — adjust
`clock_hz` in `info.yaml` and the default in `regfile.v` if your test
harness clocks the design differently. BAUD_LO/BAUD_HI still address a
full 16-bit value, but only the low 11 bits actually reach the divider
(saves area); that covers every standard baud rate down to ~305 baud
@ 10 MHz clk, so this is only a limit on unusually low custom rates.

## Standalone self-test (no external I2C hardware needed)

Hold `ui_in[2]` high. `I2C_WRITE`/`I2C_READ` will then reliably report
NACK (no real slave is present, and the loopback models an idle
pulled-up bus), which exercises START/ADDRESS/STOP sequencing
end-to-end and is easy to verify on a logic analyzer or in simulation.

## Running the testbench

```bash
cd test
pip install -r requirements.txt
make
```

This uses [cocotb](https://www.cocotb.org/) with Icarus Verilog to bit-bang
UART commands into the design and check the replies (register
read/write, I2C NACK detection, STATUS/RESET). View
`test/tb.vcd` in GTKWave to inspect waveforms.

## Hardening / GDS

This repo follows the standard [Tiny Tapeout](https://tinytapeout.com)
flow: `info.yaml` drives the automated OpenLane hardening via Tiny
Tapeout's GitHub Actions (`.github/workflows/`), so no separate
place-and-route config is checked in here. Push to GitHub and check the
Actions tab for the hardening run; see the Tiny Tapeout docs for adding
the standard workflow files if this repo doesn't already have them from
the project template.

## Area

This design is flop-heavy — the command interpreter's FSM, the FIFOs,
and the register file dominate cell area, not combinational logic —
and the original SPI+I2C feature set did not fit an IHP 1x1 tile.
Area-reduction history, in order:

1. UART RX/TX FIFOs trimmed from 8-deep to 4-deep, then to 2-deep.
2. Unused IRQ_EN register (addr 0x04) dropped (reads 0x00, writes
   ignored); the scratch register at 0x07 was kept since it's
   exercised by `test/test.py` as a register read/write sanity check.
3. `cmd_interp`'s opcode-specific payload registers consolidated: since
   only one opcode is ever in flight, fields like the old
   `spi_tx_byte`/`i2c_tx_byte`/`cur_reg_data`/`cfg_byte1` never
   coexist and now share one physical register instead of four
   (similarly for the register-address/cfg-byte2 pair, and the SPI
   chip-select/I2C-address+rw pair).
4. **The SPI master was dropped entirely** (`spi_master.v` removed,
   SPI opcodes/pins/register bits gone) — even after (1)-(3),
   utilization was still ~128%, and no further register-sharing trick
   was going to close a gap that large. I2C was kept over SPI because
   it's the more useful lab-bench interface for this chip: 2 pins
   (`uio[0:1]`) vs. SPI's 7 (sclk/mosi/miso + 4x cs_n), and it matches
   far more of what's actually sitting in a hobbyist/lab parts drawer
   (I2C sensor/RTC/EEPROM/display breakout boards, and every
   Raspberry Pi and most MCU dev boards have a native I2C master to
   drive them). SPI is more forgiving electrically (push-pull, no
   pull-up dependency) but that didn't outweigh the pin and ecosystem
   cost here. This alone got utilization down to ~110%.
5. Clearing the 100% placement-density check turned out to not be
   enough on its own: at ~99% utilization, `OpenROAD` couldn't honor
   its target density at all (forced to 1.0, zero slack) and Clock
   Tree Synthesis's legalization step failed outright. Getting real
   routing headroom needed three more cuts, chosen specifically to
   preserve lab usability rather than just chase the number:
   - `fifo_sync.v`'s depth-2 FIFOs replaced by a dedicated single-entry
     `skid_buffer.v`. Not zero buffering — a lab tool that silently
     drops UART bytes is worse than one that's merely bigger — just
     enough to bridge `uart_rx`/`uart_tx`'s one-cycle pulses to
     `cmd_interp`'s FSM, which is the only thing byte-level buffering
     actually does here (the FSM drains bytes orders of magnitude
     faster than any real UART bit period).
   - `cmd_interp`'s `n_bytes`/`byte_idx` narrowed 8→5 bits (31-byte max
     I2C burst instead of 255) — real sensor/EEPROM I2C transactions
     essentially never get close to that.
   - `baud_gen`'s divider narrowed 16→11 bits — still covers every
     standard baud rate down to ~305 baud @ 10 MHz clk.

   That got clean past the density check (~81%) but revealed a
   different failure: `OpenROAD`'s post-CTS hold-violation repair pass
   found 261 hold-violating endpoints and needed ~30% bonus area
   (435 buffers) to fix them, which there wasn't room for --
   legalization failed. This isn't primarily a density problem anymore.
6. `cmd_interp`'s `fetch_return`/`push_return` registers (17 bits)
   removed -- every fetch/push call site has exactly one valid resume
   state (after splitting the one ambiguous case, REG_WRITE vs.
   REG_READ, into two destination codes), so the resume state is now
   derived combinationally from the existing `fetch_dest`/a new smaller
   `push_dest` tag instead of being stored. Same states, same
   transitions, zero protocol impact -- pure internal restructuring.
   Modest win (~490 µm², utilization 83.4%→81.4%) and, as expected, it
   didn't touch the real blocker: still 423 hold buffers needed, still
   failing legalization (106 instances).
7. Tried tightening CTS sink clustering (size 8→4, max diameter
   50→25µm) to reduce clock skew at the source, since that's what's
   really driving the hold-violation count. Measured *worse*: more
   clusters meant more clock leaf buffers (19→76), which competed with
   the hold-fix buffers for the same scarce legal placement area
   (106→164 failing instances). Reverted to TritonCTS's own
   auto-clustering.
8. `cmd_interp`'s `n_bytes`/`byte_idx` narrowed again, 5→4 bits (31→15
   byte max I2C burst). Same reasoning as step 5's narrowing: I2C has
   no inherent per-transaction size limit, so this only means the host
   splits a big read/write into more, smaller commands -- no device
   becomes unreachable.

See git history for the measured utilization at each step.
