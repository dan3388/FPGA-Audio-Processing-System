# Makefile for pico2-ice complete workflow

# Default project name (can be overridden)
PROJECT ?= audio_processor_transceiver

# Verilog source file
VERILOG1 = $(PROJECT).sv
VERILOG2 = i2s_transmitter.sv
VERILOG3 = signal_processor.sv
VERILOG4 = spi_receiver.sv

# Physical Constraint File
PCF = $(PROJECT).pcf

# Top-level module name
TOP = $(PROJECT)

# Intermediate and output files
JSON = $(TOP).json
ASC  = $(TOP).asc
BIN  = $(TOP).bin

# Default target: build and upload everything
all: create

# Step 1: Synthesize Verilog to JSON
$(JSON): $(VERILOG1)
	@echo "\n=== Synthesizing Verilog with Yosys ===\n"
	yosys -p "synth_ice40 -top $(TOP) -json $(JSON)" $(VERILOG1) $(VERILOG2) $(VERILOG3) $(VERILOG4)

# Step 2: Place-and-route with nextpnr
$(ASC): $(JSON) $(PCF)
	@echo "\n=== Place-and-route with nextpnr ===\n"
	nextpnr-ice40 --up5k --package sg48 --pcf $(PCF) --json $(JSON) --asc $(ASC)

# Step 3: Pack to FPGA binary
create: $(BIN): $(ASC)
	@echo "\n=== Packing ASCII bitstream into .bin ===\n"
	icepack $(ASC) $(BIN)

# Clean all generated files
clean:
	@echo "\n=== Cleaning build files ===\n"
	rm -f $(JSON) $(ASC) $(BIN)
