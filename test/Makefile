SIM ?= icarus
TOPLEVEL_LANG ?= verilog

SRC_DIR = $(PWD)/../src

PROJECT_SOURCES = \
	fifo_sync.v \
	baud_gen.v \
	uart_rx.v \
	uart_tx.v \
	spi_master.v \
	i2c_master.v \
	regfile.v \
	cmd_interp.v \
	tt_um_catalinlazar_uart_spi_i2c_bridge.v

ifeq ($(GATES), yes)
COMPILE_ARGS += -DGL_TEST
COMPILE_ARGS += -DFUNCTIONAL
COMPILE_ARGS += -DUSE_POWER_PINS
COMPILE_ARGS += -DSIM
COMPILE_ARGS += -DUNIT_DELAY=\#1
# ciel (the PDK version manager tt-gds-action's gl_test uses) nests the
# stdcell library under a version-hash directory rather than directly at
# $(PDK_ROOT)/sg13g2_stdcell/..., so locate it instead of hardcoding the path.
STDCELL_V := $(shell find $(PDK_ROOT) -path '*/sg13g2_stdcell/verilog/sg13g2_stdcell.v' 2>/dev/null | head -1)
VERILOG_SOURCES += $(STDCELL_V)
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v
else
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
endif

VERILOG_SOURCES += $(PWD)/tb.v

TOPLEVEL = tb
MODULE = test

include $(shell cocotb-config --makefiles)/Makefile.sim
