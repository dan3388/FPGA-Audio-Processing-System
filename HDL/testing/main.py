import time
from machine import Pin, SoftI2C, ADC, SPI
import ice

file = open("i2s_sound_test.bin", "br")
flash = ice.flash(miso=Pin(4), mosi=Pin(7), sck=Pin(6), cs=Pin(5))
flash.erase(4096) # Optional
flash.write(file)
# Optional
fpga = ice.fpga(cdone=Pin(40), clock=Pin(21), creset=Pin(31), cram_cs=Pin(5), cram_mosi=Pin(4), cram_sck=Pin(6), frequency=48)
fpga.start()

# 2. Define the DAC Reset Pin (Active Low)
# GPIO10 goes to Reset pin on TLV320DAC3100
RESET_PIN = Pin(34)

# -------------------------------------------------------------
RESET_PIN.value(1)
time.sleep(1)
RESET_PIN.value(0)
