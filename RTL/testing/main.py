import time
from machine import Pin, I2C
import ice

DAC_ADDR = 0x18
I2C_SDA_PIN = 2
I2C_SCL_PIN = 3

LED_GREEN = Pin(1, Pin.OUT)
LED_GREEN.value(1)

# Initialize I2C
i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=50000) # SCL max clk frequency is 100 kHz in "standard" mode

def write_reg(page, reg, value):
    """Writes a value to a specific register on a specific page."""
    # 1. Switch to the desired page (Register 0 is always Page Select)
    # [cite: 2838]
    i2c.writeto_mem(DAC_ADDR, 0x00, bytes([page]))
    
    # 2. Write the value to the register
    i2c.writeto_mem(DAC_ADDR, reg, bytes([value]))
    # print(f"Page {page}, Reg {reg} -> 0x{value:02X}")

def init_dac():
    # Software Reset 
    write_reg(0, 1, 0b1)

    # Clock-Gen Muxing
    #
    write_reg(0, 4, 0b0000)

# --- Run ---
try:
    # Scan just to see if device is there
    devices = i2c.scan()
    if DAC_ADDR in devices:
        print(f"Device found at 0x{DAC_ADDR:02X}")
        init_dac()
        LED_GREEN.value(0)
    else:
        print("DAC not found! Check wiring and MCLK.")
except Exception as e:
    print("Error:", e)

test_PIN = Pin(0, Pin.OUT)
test_led_value = 0

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
fpga = ice.fpga(cdone=Pin(40), clock=Pin(21), creset=Pin(31), cram_cs=Pin(5), cram_mosi=Pin(4), cram_sck=Pin(6), frequency=48)
fpga.start()

time.sleep(1)

FPGA_RESET.value(0)
#RESET_PIN.value(0)
time.sleep(1)
FPGA_RESET.value(1)