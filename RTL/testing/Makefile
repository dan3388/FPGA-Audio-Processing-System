# Makefile for pico2-ice complete workflow

# Default project name (can be overridden)
PROJECT ?= i2s_sound_test

# Verilog source file
VERILOG = $(PROJECT).sv

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
	yosys -p "synth_ice40 -top $(TOP) -json $(JSON)" $(VERILOG)

# Step 2: Place-and-route with nextpnr
$(ASC): $(JSON) $(PCF)
	@echo "=== Place-and-route with nextpnr ==="
	nextpnr-ice40 --up5k --package sg48 --pcf $(PCF) --json $(JSON) --asc $(ASC)

# Step 3: Pack to FPGA binary
$(BIN): $(ASC)
	@echo "=== Packing ASCII bitstream into .bin ==="
	icepack $(ASC) $(BIN)

# Step 4: Upload .bin and main.py to RP2350 and run main.py
upload: $(BIN) $(PYTHON)
	@echo "=== Uploading .bin and main.py to RP2350 ==="
	mpremote cp $(BIN) :
	mpremote cp $(PYTHON) :
	@echo "=== Running main.py on RP2350 ==="
	mpremote run $(PYTHON)
	@echo "=== Done! FPGA should now be programmed ==="

# Clean all generated files
clean:
	@echo "=== Cleaning build files ==="
	rm -f $(JSON) $(ASC) $(BIN)
