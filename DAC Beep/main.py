import time
from machine import Pin, I2C
import ice

DAC_ADDR = 0x18

sda_pin = Pin(2, Pin.OUT, Pin.PULL_UP)
scl_pin = Pin(3, Pin.OUT, Pin.PULL_UP)

GREEN_LED = Pin(0, Pin.OUT)
RED_LED = Pin(1, Pin.OUT)
RED_LED.value(1)
GREEN_LED.value(1)

# Initialize I2C
i2c = I2C(1, scl=scl_pin, sda=sda_pin, freq=100000) # SCL max clk frequency is 100 kHz in "standard" mode

print("Scanning I2C bus...")
devices = i2c.scan()

if len(devices) == 0:
    print("No I2C devices found! Check your wiring and pull-up resistors.")
else:
    print(f"I2C devices found: {len(devices)}")
    for device in devices:
        print(f"Decimal address: {device} | Hex address: {hex(device)}")

print("i2c initialized")

# Helper function to write a register
def write_reg(reg, val_bin):
    i2c.writeto_mem(DAC_ADDR, reg, bytes([val_bin]))

def init_dac_beep():
    print("Initializing DAC with PLL for Beep...")
    
    # ==============================
    # PAGE 0: CLOCKS & DIGITAL SETUP
    # ==============================
    write_reg(0, 0b00000000)  # Select Page 0
    write_reg(1, 0b00000001)  # Software Reset [cite: 2840]
    time.sleep(0.01)

    # 1. PLL Configuration (CRITICAL for Beep Gen)
    # We multiply 12.288 MHz -> 98.304 MHz to get enough DSP cycles
    # PLL_CLK = (PLL_CLKIN * R * J.D) / P
    # 98.304 = (12.288 * 1 * 8.0000) / 1
    write_reg(4, 0b00000011)  # PLL_CLKIN = MCLK, CODEC_CLKIN = PLL_CLK [cite: 2850]
    write_reg(6, 0b00001000)  # J = 8 [cite: 2856]
    write_reg(7, 0b00000000)  # D (MSB) = 0 [cite: 2858]
    write_reg(8, 0b00000000)  # D (LSB) = 0 [cite: 2861]
    write_reg(5, 0b10010001)  # PLL Power Up, P=1, R=1 [cite: 2854]
    time.sleep(0.02)    # Wait for PLL to lock

    # 2. Clock Dividers (Target 48kHz)
    # CODEC_CLKIN = 98.304 MHz
    # DAC_FS = 98.304M / (NDAC * MDAC * DOSR)
    # 48k = 98.304M / (4 * 4 * 128)
    # Resource Class Check: (4 * 128) / 32 = 16 (16 >= 12, required for PRB_P25)
    write_reg(11, 0b10000100) # NDAC = 4, Power Up [cite: 2876]
    write_reg(12, 0b10000100) # MDAC = 4, Power Up [cite: 2878]
    write_reg(13, 0b00000000) # DOSR MSB = 0
    write_reg(14, 0b10000000) # DOSR LSB = 128 [cite: 2882]
    
    # 2.5 Turn on Master Mode
    write_reg(27, 0b00001100)
    write_reg(30, 0b10010000)

    # 3. Processing Block Selection
    # [cite_start]PRB_P25 is REQUIRED for Beep Generator [cite: 3012]
    write_reg(60, 0b00011001) # Select PRB_P25

    # 4. DAC Routing & Power
    write_reg(63, 0b11010100) # Left/Right DAC Up, Left uses Left Data [cite: 3023]
    write_reg(64, 0b00000000) # Unmute DACs [cite: 3025]
    write_reg(65, 115) # Left DAC Vol = -68.7 dB [cite: 3028]

    # ==============================
    # BEEP GENERATOR CONFIGURATION
    # ==============================
    
    # 5. Beep Length (~2 seconds at 48kHz)
    # [cite_start]0x017700 = ~96,000 samples [cite: 1770]
    write_reg(73, 0b00000001) 
    write_reg(74, 0b01110111) 
    write_reg(75, 0b00000000)

    # 6. Beep Frequency (1 kHz Sine Wave @ 48kHz)
    # [cite_start]Values derived from Datasheet Table 6-23 [cite: 1782]
    write_reg(76, 0b00010000) # Sine MSB
    write_reg(77, 0b10110101) # Sine LSB
    write_reg(78, 0b01111110) # Cosine MSB
    write_reg(79, 0b11101000) # Cosine LSB

    # ==============================
    # PAGE 1: ANALOG OUTPUT & SPEAKER
    # ==============================
    write_reg(0, 0b00000001)  # Select Page 1
    
    # 8. Power Up Drivers
    # [cite_start]HPL/HPR Power Up (Reg 31) [cite: 3184]
    write_reg(31, 0b11000000) 
    # [cite_start]Class-D Speaker Power Up (Reg 32) [cite: 3186]
    write_reg(32, 0b10000000) 

    # 9. Output Routing (DAC to Mixers)
    # [cite_start]Route DAC_L to Left Mixer, DAC_R to Right Mixer [cite: 3210]
    write_reg(35, 0b01000100) 

    # 10. Route Mixers to Output Pins
    # [cite_start]Reg 36: Route Left Mixer to HPL, 0dB [cite: 3212]
    write_reg(36, 0b10000000) 
    # [cite_start]Reg 38: Route Left Mixer to SPEAKER, 0dB [cite: 3216]
    write_reg(38, 0b10000000) 

    # 11. Unmute Analog Drivers
    # [cite_start]Reg 40: Unmute HPL, 0dB [cite: 3220]
    write_reg(40, 0b00000110) 
    # Reg 42: Unmute Class-D Speaker, 6dB Gain 
    write_reg(42, 0b00000100)
    
    time.sleep(0.1)
    
    # ==============================
    # TRIGGER BEEP (Page 0)
    # ==============================
    write_reg(0, 0b00000000)  # Go back to Page 0
    
    print("Triggering Beep NOW...")
    # 11. Trigger Beep (Enable bit + Volume)
    # Bit 7=1 (Enable), Bits 5-0 = 2dB gain
    # This must be the LAST step!
    write_reg(71, 0b10000000)

    print("DAC Initialized. Beep should be playing on SPEAKER and HEADPHONES.")


FPGA_RESET = Pin(23 , Pin.OUT)

FPGA_RESET.value(0)
time.sleep(1)
FPGA_RESET.value(1)

# Run it
init_dac_beep()



# Define the DAC Reset Pin (Active Low)
# GPIO10 goes to Reset pin on TLV320DAC3100
#RESET_PIN = Pin(34, Pin.OUT)
# -------------------------------------------------------------


file = open("i2s_sound_test.bin", "br")
flash = ice.flash(miso=Pin(4), mosi=Pin(7), sck=Pin(6), cs=Pin(5))
flash.erase(4096) # Optional
flash.write(file)
# Optional
fpga = ice.fpga(cdone=Pin(40), clock=Pin(21), creset=Pin(31), cram_cs=Pin(5), cram_mosi=Pin(4), cram_sck=Pin(6), frequency=12.288)
fpga.start()

#i2c.writeto_mem(DAC_ADDR, 0, bytes([0b00000000])) # choose page 0
#flags = i2c.readfrom_mem(DAC_ADDR, 37, 1)
#print((flags))
#if flags == bytes([0b10100000]):
RED_LED.value(0)
GREEN_LED.value(0)  
