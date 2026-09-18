"""
Cocotb testbench for tt_um_catalinlazar_uart_spi_i2c_bridge.

Strategy: drive the UART RX pin (ui_in[0]) by bit-banging at the chip's
default baud rate (9600 baud, assuming a 10 MHz clk, per regfile reset
defaults). A background monitor task continuously watches the UART TX
pin (uo_out[0]) from the moment of reset onward and decodes every byte
the chip transmits into a queue -- this avoids a race where a
per-request receiver started *after* sending a command could miss a
reply that begins before the send even finishes (the interpreter can
reply within a handful of clock cycles, much faster than one more UART
bit period).

ui_in[2] (loopback_test_en) is held high for the whole test so the I2C
engine can be fully exercised with no external device: with no real
slave present, the bus floats and is modeled as pulled high, so every
I2C transaction's address byte should come back NACKed -- this still
exercises START/ADDRESS/STOP sequencing.

(An SPI master was part of the original design but was dropped to fit
the IHP 1x1 tile area budget -- see README "Area" section.)
"""

import asyncio

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Timer, RisingEdge

CLK_PERIOD_NS = 100          # 10 MHz
BAUD_DIV      = 64           # matches regfile reset default -> 9600 baud
BIT_NS        = 16 * (BAUD_DIV + 1) * CLK_PERIOD_NS   # one UART bit period
POLL_NS       = 50           # polling resolution for UART RX sampling

LOOPBACK_BIT = 1 << 2

# Opcodes
OP_I2C_WRITE = 0x10
OP_I2C_READ  = 0x11
OP_CFG_I2C   = 0x21
OP_REG_WRITE = 0x30
OP_REG_READ  = 0x31
OP_STATUS    = 0xF0
OP_RESET     = 0xFF

STATUS_OK   = 0x00
STATUS_NACK = 0x01


async def reset_dut(dut):
    dut.ena.value = 1
    dut.rst_n.value = 0
    dut.ui_in.value = LOOPBACK_BIT | 0x01   # uart_rx idle high, loopback on
    dut.uio_in.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    for _ in range(5):
        await RisingEdge(dut.clk)


async def uart_send_byte(dut, byte):
    """Bit-bang one byte out onto ui_in[0], preserving other ui_in bits."""
    base = int(dut.ui_in.value) & ~0x01

    async def set_bit(b):
        dut.ui_in.value = base | (1 if b else 0)
        await Timer(BIT_NS, unit="ns")

    await set_bit(0)  # start bit
    for i in range(8):
        await set_bit((byte >> i) & 1)
    await set_bit(1)  # stop bit


async def uart_send_bytes(dut, data):
    for b in data:
        await uart_send_byte(dut, b)


async def send_cmd(dut, opcode, payload):
    frame = [opcode, len(payload)] + list(payload)
    await uart_send_bytes(dut, frame)


def start_uart_tx_monitor(dut):
    """Launch a background task that decodes every byte on uo_out[0]
    into an asyncio.Queue, and return that queue. Must be started right
    after reset (while the line is guaranteed idle-high) so the very
    first low sample it ever sees is a genuine start bit."""
    queue = asyncio.Queue()

    async def _monitor():
        line_high = True
        while True:
            await Timer(POLL_NS, unit="ns")
            val = int(dut.uo_out.value) & 0x01
            if line_high and val == 0:
                line_high = False
                # we're within POLL_NS of the true falling edge; move to
                # the middle of the start bit, then sample each data bit
                # one full bit period apart.
                await Timer(BIT_NS // 2, unit="ns")
                bits = []
                for _ in range(8):
                    await Timer(BIT_NS, unit="ns")
                    bits.append(int(dut.uo_out.value) & 0x01)
                await Timer(BIT_NS, unit="ns")  # stop bit, not checked
                byte_val = 0
                for i, b in enumerate(bits):
                    byte_val |= (b << i)
                await queue.put(byte_val)
                line_high = True
            elif val == 1:
                line_high = True

    cocotb.start_soon(_monitor())
    return queue


async def queue_get_timeout(queue, timeout_ns):
    waited = 0
    while True:
        try:
            return queue.get_nowait()
        except asyncio.QueueEmpty:
            await Timer(POLL_NS, unit="ns")
            waited += POLL_NS
            if waited > timeout_ns:
                raise TimeoutError("Timeout waiting for UART reply")


async def recv_bytes(queue, n, timeout_ns):
    return [await queue_get_timeout(queue, timeout_ns) for _ in range(n)]


@cocotb.test()
async def test_reg_write_read(dut):
    clock = Clock(dut.clk, CLK_PERIOD_NS, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    rxq = start_uart_tx_monitor(dut)

    await send_cmd(dut, OP_REG_WRITE, [0x07, 0xA5])
    status = await queue_get_timeout(rxq, BIT_NS * 20)
    assert status == STATUS_OK, f"REG_WRITE status={status:#x}"

    await send_cmd(dut, OP_REG_READ, [0x07])
    reply = await recv_bytes(rxq, 2, BIT_NS * 20)
    assert reply[0] == STATUS_OK, f"REG_READ status={reply[0]:#x}"
    assert reply[1] == 0xA5, f"REG_READ data={reply[1]:#x}, expected 0xA5"


@cocotb.test()
async def test_i2c_no_slave_nacks(dut):
    clock = Clock(dut.clk, CLK_PERIOD_NS, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    rxq = start_uart_tx_monitor(dut)

    await send_cmd(dut, OP_CFG_I2C, [0x02, 0x00])
    status = await queue_get_timeout(rxq, BIT_NS * 20)
    assert status == STATUS_OK

    await send_cmd(dut, OP_I2C_WRITE, [0x50, 0x02, 0xAA, 0xBB])
    status = await queue_get_timeout(rxq, BIT_NS * 60)
    assert status == STATUS_NACK, f"expected NACK with no slave present, got {status:#x}"


@cocotb.test()
async def test_i2c_read_no_slave_nacks(dut):
    clock = Clock(dut.clk, CLK_PERIOD_NS, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    rxq = start_uart_tx_monitor(dut)

    await send_cmd(dut, OP_CFG_I2C, [0x02, 0x00])
    assert await queue_get_timeout(rxq, BIT_NS * 20) == STATUS_OK

    await send_cmd(dut, OP_I2C_READ, [0x50, 0x02])
    status = await queue_get_timeout(rxq, BIT_NS * 60)
    assert status == STATUS_NACK, f"expected NACK with no slave present, got {status:#x}"


@cocotb.test()
async def test_status_and_reset(dut):
    clock = Clock(dut.clk, CLK_PERIOD_NS, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    rxq = start_uart_tx_monitor(dut)

    await send_cmd(dut, OP_STATUS, [])
    _ = await queue_get_timeout(rxq, BIT_NS * 20)  # just check it replies

    await send_cmd(dut, OP_RESET, [])
    status = await queue_get_timeout(rxq, BIT_NS * 20)
    assert status == STATUS_OK
