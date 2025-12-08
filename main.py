import time
from machine import Pin, SoftI2C, ADC, SPI

# ------------------ Pin Definitions -----------------------
# 1. Define the I2C pins connected to the DAC
# GPIO2 -> SDA, GPIO3 -> SCL on TLV320DAC3100
I2C_SCL_PIN = Pin(3)
I2C_SDA_PIN = Pin(2)

# 2. Define the DAC Reset Pin (Active Low)
# GPIO10 goes to Reset pin on TLV320DAC3100
RESET_PIN = Pin(34)

# -------------------------------------------------------------
RESET_PIN.value(1)
time.sleep(1)
RESET_PIN.value(0)

# -------------------------------------------------------------
class Pico_ADC:
    def __init__(self, pin_num):
        self.adc = ADC(pin_num)
    
    def read_raw(self):
        return self.adc.read_u16()
    
#--------------------------------------------------------------

class Pico_SPI:
    def __init__(
        self,
        id=1,
        br: int = 1000000,
        polarity: int = 0,
        phase: int = 0,
        sck:int | None = None,
        mosi:int | None = None,
        miso:int | None = None,
        cs:int | None = None
    ):
        
        self.cs = Pin(cs if cs else 29, Pin.OUT)
        self.spi = SPI(
            id,
            baudrate=br,
            polarity=polarity,
            phase=phase,
            sck=sck,
            mosi=mosi,
            miso=miso
        )
        
        self.cs.value(1)
    
    def write_byte(self, data):
        self.cs.value(0) 
        self.spi.write(bytearray([data & 0xFF]))
        self.cs.value(1)
        
    def send_data(self, data):
        high = (data >> 8) & 0xFF
        low = data & 0xFF
        write_byte(high)
        write_byte(low)
        
    def stream(self, adc, sr=8000):
        sample_time = 1000 / sr
        try:
            while True:
                data = adc.read_raw()
                self.write_byte(data)
                time.sleep(int(sample_time))
        except KeyboardInterrupt:
            print("Done cuz you stopped me")
                
    
#--------------------------------------------------------------
adc = Pico_ADC(41)
'''
SPI1 Pins:
SCK1: GPIO26 -> ICE21
MOSI1: GPIO28 -> ICE9
MISO1: GPIO27 -> ICE18
CSn1/SSn1: GPIO29 -> ICE11
'''
transmitter = Pico_SPI(id=1, br=10000000, polarity=0, phase=0, sck=26 , mosi=27, miso=28, cs=29)
while(1):
    transmitter.stream(adc, sr=48000)
    
    
