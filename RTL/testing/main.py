import time
from machine import Pin, I2C
import ice

DAC_ADDR = 0x18
I2C_SDA_PIN = 2
I2C_SCL_PIN = 3

GREEN_LED = Pin(0, Pin.OUT)
RED_LED = Pin(1, Pin.OUT)
RED_LED.value(1)
GREEN_LED.value(1)

# Initialize I2C
i2c = I2C(1, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=50000) # SCL max clk frequency is 100 kHz in "standard" mode

def init_dac():

    # PAGE 0 REGISTERS:
    i2c.writeto_mem(DAC_ADDR, 0, bytes([0b00000000])) # choose page 0
    time.sleep(0.1)
    # Software Reset 
    i2c.writeto_mem(DAC_ADDR, 1, bytes([0b1]))
    time.sleep(0.1)

    # PLL multiply and divide both = 1
    i2c.writeto_mem(DAC_ADDR, 5, bytes([0b10010001]))

    # power on MDAC and NDAC, set divide/multiply values to 1
    i2c.writeto_mem(DAC_ADDR, 11, bytes([0b10000001]))
    i2c.writeto_mem(DAC_ADDR, 12, bytes([0b10000001]))

    # power on BCLK N-divider, choose N = 1 
    i2c.writeto_mem(DAC_ADDR, 30, bytes([0b10000001]))

    # Select DAC Processing block (PRB_P4 is nothing special and just left channel)
    # i2c.writeto_mem(DAC_ADDR, 60, bytes([0b00000100]))
    i2c.writeto_mem(DAC_ADDR, 60, bytes([0b00011001])) # this is for the beep only

    # Enable left-channel DAC and set left-channel DAC data path = left data
    i2c.writeto_mem(DAC_ADDR, 63, bytes([0b10010000]))

    # Unmute left channel DAC and set independent volume control
    i2c.writeto_mem(DAC_ADDR, 64, bytes([0b00000100]))

    # Set left-channel DAC digital volume control = 23 dB
    i2c.writeto_mem(DAC_ADDR, 65, bytes([0b00101110]))

    # Set the beep!
    i2c.writeto_mem(DAC_ADDR, 71, bytes([0b10000000]))
    i2c.writeto_mem(DAC_ADDR, 73, bytes([0b11111111]))
    i2c.writeto_mem(DAC_ADDR, 74, bytes([0b11111111]))
    i2c.writeto_mem(DAC_ADDR, 75, bytes([0b11111111]))


    # PAGE 1 REGISTERS:
    i2c.writeto_mem(DAC_ADDR, 0, bytes([0b00000001])) # choose page 0
    time.sleep(0.1)
    # Turn on HPL drivers (headphone left), and select 1.65 V common-mode output voltage.
    i2c.writeto_mem(DAC_ADDR, 31, bytes([0b10010100]))

    # Remove sudden audible pop by setting driver ramp-up step time
    i2c.writeto_mem(DAC_ADDR, 33, bytes([0b00111100]))

    # route left DAC to the HPL driver
    i2c.writeto_mem(DAC_ADDR, 35, bytes([0b1000]))

    # send left analog volume control to HPL, and set left-channel analog gain
    i2c.writeto_mem(DAC_ADDR, 36, bytes([0b11111111]))

    # unmute HPL driver
    i2c.writeto_mem(DAC_ADDR, 40, bytes([0b00000111]))


init_dac()


# Define the DAC Reset Pin (Active Low)
# GPIO10 goes to Reset pin on TLV320DAC3100
#RESET_PIN = Pin(34, Pin.OUT)
FPGA_RESET = Pin(23, Pin.OUT)
# -------------------------------------------------------------


file = open("i2s_sound_test.bin", "br")
flash = ice.flash(miso=Pin(4), mosi=Pin(7), sck=Pin(6), cs=Pin(5))
flash.erase(4096) # Optional
flash.write(file)
# Optional
fpga = ice.fpga(cdone=Pin(40), clock=Pin(21), creset=Pin(31), cram_cs=Pin(5), cram_mosi=Pin(4), cram_sck=Pin(6), frequency=12.288)
fpga.start()

time.sleep(1)

FPGA_RESET.value(0)
time.sleep(1)
FPGA_RESET.value(1)


flags = i2c.readfrom_mem(DAC_ADDR, 37, 1)
if flags == bytes([0b10100000]):
    RED_LED.value(0)
    GREEN_LED.value(0)  