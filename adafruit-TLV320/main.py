import time
from machine import Pin, I2C, ADC, SPI  
import ice

#------------------------------ Pin Definitions -----------------------
DAC_ADDR = 0x18
# I2C for DAC
sda_pin = Pin(2, Pin.OUT, Pin.PULL_UP)
scl_pin = Pin(3, Pin.OUT, Pin.PULL_UP)

# LED Pins
GREEN_LED = Pin(0, Pin.OUT)
RED_LED = Pin(1, Pin.OUT)
RED_LED.value(1)
GREEN_LED.value(1)

# FPGA Reset Function
def reset_fpga():
    FPGA_RESET = Pin(23 , Pin.OUT)
    FPGA_RESET.value(0)
    time.sleep(1)
    FPGA_RESET.value(1)


#-------------------------------- I2C Setup -------------------------
# Initialize I2C
i2c = I2C(1, scl=scl_pin, sda=sda_pin, freq=100000) # SCL max clk frequency is 100 kHz in "standard" mode

def debug_I2C():
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

def init_dac_i2s_slave():
    print("Initializing DAC in I2S SLAVE Mode (MCLK = 12.288 MHz)...")
    
    # ==============================
    # PAGE 0: CLOCKS & DIGITAL SETUP
    # ==============================
    write_reg(0, 0b00000000)  # Select Page 0
    write_reg(1, 0b00000001)  # Software Reset
    time.sleep(0.01)

    # 1. Clock Source (No PLL)
    # We use MCLK directly. 
    # D1-D0 = 00 (CODEC_CLKIN = MCLK)
    write_reg(4, 0b00000000)  

    # Note: We skip Reg 5, 6, 7, 8 because PLL is not used.

    # 2. Clock Dividers (12.288 MHz -> 48 kHz)
    # Target: 48000 Hz
    # Formula: CODEC_CLKIN / (NDAC * MDAC * DOSR)
    # 12.288M / (1 * 2 * 128) = 48k
    
    # NDAC = 1
    # D7=1 (Power Up), D6-D0=1
    write_reg(11, 0b10000001) 
    
    # MDAC = 2
    # D7=1 (Power Up), D6-D0=2
    write_reg(12, 0b10000010) 
    
    # DOSR = 128 (Standard for 48kHz Filter A)
    # DOSR MSB = 0
    write_reg(13, 0b00000000) 
    # DOSR LSB = 128
    write_reg(14, 0b10000000) 
    
    # 3. Interface Control (SLAVE MODE)
    # We accept BCLK and WCLK from external MCU.
    # D7-D6=00 (I2S Mode)
    # D5-D4=00 (16-bit Word Length) 
    # D3=0 (BCLK is INPUT / Slave)
    # D2=0 (WCLK is INPUT / Slave)
    write_reg(27, 0b00000000) 

    # 4. Processing Block Selection
    # PRB_P1 is standard for Stereo Playback (Resource Class 8)
    write_reg(60, 0b00000001) # Select PRB_P1

    # 5. DAC Routing & Power
    write_reg(63, 0b11010100) # Left/Right DAC Up, Left uses Left Data
    write_reg(64, 0b00000000) # Unmute DACs
    write_reg(65, 0b11111111) # Left DAC Vol = 0dB
    write_reg(66, 0b11111111) # Right DAC Vol = 0dB

    # ==============================
    # PAGE 1: ANALOG OUTPUT & SPEAKER
    # ==============================
    write_reg(0, 0b00000001)  # Select Page 1
    
    # 6. Power Up Drivers
    # HPL/HPR Power Up (Reg 31)
    write_reg(31, 0b11000000) 
    # Class-D Speaker Power Up (Reg 32)
    write_reg(32, 0b10000000) 

    # 7. Output Routing (DAC to Mixers)
    # Route DAC_L to Left Mixer, DAC_R to Right Mixer
    write_reg(35, 0b01000100) 

    # 8. Route Mixers to Output Pins
    # Reg 36: Route Left Mixer to HPL, -35.7 dB
    write_reg(36, 71) 
    # Reg 37: Route Right Mixer to HPR, -35.7 dB
    write_reg(37, 71) 
    # Reg 38: Route Left Mixer to SPEAKER, -72.2 dB
    write_reg(38, 60) 

    # 9. Unmute Analog Drivers
    # Reg 40: Unmute HPL, 0dB
    write_reg(40, 0b00000110) 
    # Reg 41: Unmute HPR, 0dB
    write_reg(41, 0b00000110) 
    # Reg 42: Unmute Class-D Speaker, 6dB Gain 
    write_reg(42, 0b00000100) 
    
    print("DAC Configured for I2S Slave. Start your I2S stream now.")

#----------------------------- Pico ADC  -----------------------------
class Pico_ADC:
    def __init__(self, pin_num):
        self.adc = ADC(pin_num)
    
    def read_raw(self):
        return self.adc.read_u16()
    
#----------------------------- Pico SPI ------------------------------
class Pico_SPI:
    def __init__(
        self,
        id=1,
        br: int = 1536000,
        polarity: int = 0,
        phase: int = 0,
        sck:int | None = None,
        mosi:int | None = None,
        miso:int | None = None,
        cs:int | None = None
    ):
        
        self.cs = Pin(cs if cs else 29, Pin.OUT)
        self.cs.value(1)  # Deselect the device

        self.spi = SPI(
            id,
            baudrate=br,
            polarity=polarity,
            phase=phase,
            sck=sck,
            mosi=mosi,
            miso=miso
        )

        # Pre-allocate buffers
        self.data_buffer = bytearray(2) 
        self.silence_buffer = bytearray([0x00, 0x00]) # 16 bits of silence
    
    def __send_data(self, data):
        new_data = self.twos_complement(data)
        high = (new_data >> 8) & 0xFF
        low = new_data & 0xFF

        self.data_buffer[0] = high
        self.data_buffer[1] = low
        
        self.cs.value(0)  # Select the device
        self.spi.write(self.data_buffer)
        self.spi.write(self.silence_buffer)  # Send silence to Right Channel
        self.cs.value(1)  # Deselect the device

    def twos_complement(self, data):
        if data >= (2**15):
            value = data - (1 << 16)
        else: 
            value = data
        return value
        
    def stream(self, adc, sr=48000):
        actual_sr = self.spi.baudrate / 32
        print(f"Starting SPI Stream at ~{actual_sr} Hz (Target: {sr} Hz)")
        while True:
            data = adc.read_raw()
            self.__send_data(data)

#------------------ Flash FPGA with I2S Sound Test Bitfile -----------------------
file = open("i2s_sound_test.bin", "br")
flash = ice.flash(miso=Pin(4), mosi=Pin(7), sck=Pin(6), cs=Pin(5))
flash.erase(4096) # Optional
flash.write(file)
# Optional
fpga = ice.fpga(cdone=Pin(40), clock=Pin(21), creset=Pin(31), cram_cs=Pin(5), cram_mosi=Pin(4), cram_sck=Pin(6), frequency=12.288)
fpga.start()

RED_LED.value(0)
GREEN_LED.value(0)  

#------------------ Start Streaming Audio via I2S ------------------------
adc = Pico_ADC(41)
debug_I2C()
init_dac_i2s_slave()
reset_fpga()
'''
SPI1 Pins:
SCK1: GPIO26 -> ICE21
MOSI1: GPIO28 -> ICE9
MISO1: GPIO27 -> ICE18
CSn1/SSn1: GPIO29 -> ICE11
'''
transmitter = Pico_SPI(id=1, br=3072000, polarity=0, phase=0, sck=26 , mosi=27, miso=28, cs=29)
while(1):
    transmitter.stream(adc, sr=48000)
    time.sleep(1)
