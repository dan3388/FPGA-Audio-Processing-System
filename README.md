# FPGA-Audio-Processing-System

ENGR433 Project involving FPGA programming

By Daniel Mendoza & Jesse Thompson

ENGR433 - Walla Walla University

Fall Quarter 2025

--- 

## Project Goals

Using the Pico2-Ice development kit, our primary objective was to design a guitar effects pedal using the iCE40 FPGA. This project applies core principles of digital logic and hardware description languages learned in our Digital Design course to real-time signal processing.

Due to time constraints and a burnt out I2S DAC, this project was largely unable to move past test bench verification, although I2S transmission was both verified and tested IRL.


## System Overview

### ADC & SPI (Input Stage)

To process guitar audio within an FPGA, the signal must first be amplified and converted from analog to digital.

1. **Preamplification:** Since guitar pickups produce a low-level signal (ranging from 100mV to 500mV), the audio must be amplified to a level suitable for the ADC.

2. **A–>D Conversion:** We utilized the built-in Analog-to-Digital Converter (ADC) on Pico2-Ice's RP2350B chip. This ADC samples the 0–3.3V analog signal and converts it into a 12-bit digital signal. We converted this unsigned 12-bit signal to a signed 16-bit signal for SPI transmission.

1. **MCU–>FPGA Transmission:** We chose the Serial Peripheral Interface (SPI) protocol to bridge the Pico 2 and the FPGA. SPI provides a simple, high-speed, and reliable method for streaming raw 12-bit samples into the iCE40 for processing.

### I2S & DAC (Output Stage)

Once processed, the digital audio must be converted back into an analog waveform for playback.

1. **Digital Transmission**: We transmitted the audio from the Ice40 by implementing an I2S transmitter in the Ice40.

2. **D–>A Conversion:** We used the TLV320DAC3100 I2S DAC board made by adafruit to convert the I2S transmitted by the FPGA into an analog signal.



## Our implementation
## AI & Sources:

Our use of AI was mainly for debugging within VSCode Github Copilot. There were other cases where asking AI was useful to determine what protocols to use or explaining code found online.

### Prompts: 
> Is sending a digital signal through SPI from Rp2350 to fpga the best option? 

> explain this code to me:

    // Run test I2C commands for the sake of testing the FPGA communication over I2C
    for (uint8_t tx = 0;; tx++) 
    {
        uint8_t buf[1] = {tx};

        printf("i2c scan:");

        for (int i = 0x00; i < 0x7f; i++) {

            ret = i2c_write_blocking(APP_I2C, i, buf, sizeof(buf), false);

            if (ret >= 0) {
                printf(" 0x%02x", i);
            }

            sleep_us(100);
        }

        printf("\n");

        sleep_ms(1);
    } 

## How to compile for the Pico2-Ice
Compilation of this project for the pico2-ice is relatively simple. Below is the series of commands from OSS-CAD-SUITE to compile for the pico2-Ice, but one can also use the make file for "Working Square Wave"
```
yosys -p "synth_ice40 -top i2s_sound_test -json i2s_sound_test.json" i2s_sound_test.sv

nextpnr-ice40 --up5k --package sg48 --pcf $(PCF) --json i2s_sound_test.json --asc i2s_sound_test.asc

icepack i2s_sound_test.asc i2s_sound_test.bin
```

## Conclusion