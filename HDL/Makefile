# Makefile for pico2-ice complete workflow

# Default project name (can be overridden)
PROJECT ?= audio_processor_transceiver

# Verilog source file
VERILOG1 = $(PROJECT).sv
VERILOG2 = i2s_transmitter.sv
VERILOG3 = spi_receiver.sv
VERILOG4 = signal_processor.sv

# Physical Constraint File
PCF = $(PROJECT).pcf

# Top-level module name
TOP = $(PROJECT)

# MicroPython main file
PYTHON ?= main.py

# Intermediate and output files
JSON = $(TOP).json
ASC  = $(TOP).asc
BIN  = $(TOP).bin

# Default target: build and upload everything
all: upload

# Step 1: Synthesize Verilog to JSON
$(JSON): $(VERILOG)
	@echo "=== Synthesizing Verilog with Yosys ==="
	yosys -p "synth_ice40 -top $(TOP) -json $(JSON)" $(VERILOG1) $(VERILOG2) $(VERILOG3) $(VERILOG4)

# Step 2: Place-and-route with nextpnr
$(ASC): $(JSON) $(PCF)
	@echo "=== Place-and-route with nextpnr ==="
	nextpnr-ice40 --up5k --package sg48 --pcf $(PCF) --json $(JSON) --asc $(ASC)

# Step 3: Pack to FPGA binary
$(BIN): $(ASC)
	@echo "=== Packing ASCII bitstream into .bin ==="
	icepack $(ASC) $(BIN)

# Clean all generated files
clean:
	@echo "=== Cleaning build files ==="
	rm -f $(JSON) $(ASC) $(BIN)
