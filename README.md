# FPGA-Audio-Processing-System

ENGR433 Project involving FPGA programming

By Daniel Mendoza & Jesse Thompson

ENGR433 - Walla Walla University

Fall Quarter 2025

--- 
## Goals of this Project

Using the Pico2-Ice development kit, our initial goal was to use the ICE40 FGPA to create a guitar sounds effect pedal. Using the principles learned in the Digital Design class. 
## Overview

### DPS

The guitar sounds effect pedal is essentially a Digital Sound Processor (DPS). Our DPS would take a signal data, in our case sound coming from a guitar, and alter it with a set function to then be transmitter out and played. 

### ADC+SPI

To get sound from the a guitar and into the FPGA, the data needs to be amplified and converted from analog to digital so that the FPGA can manipulate it. The Raspberry Pi Pico2 (rp2350B) in our dev-kit, has a built-in Analog-to-Digital converter that takes in 0-3.3V analog data and converts it to 12-bit digital. The sound from the guitar would need to be amplified since the pick-up from a guitar ranges from 12mV to 500mV. 

Serial Peripheral Interface (SPI) would the be used to transmit the raw data from the raspberry pi pico2 to the FPGA. For its simplicity and reliability in transmission, SPI is the best option.

### I2S+DAC

The output of our sound will need to be converted back to analog so it can be played. We used the TLV320DAC3100 I2S DAC from Adafruit. This DA, after being programmed using I2C protocol, would take the digital signal from the FPGA using I2S and output it from a speaker or headphone. 

[I2S stuff goes here]





## Our implementation
## AI & Sources:

Our use of AI was mainly for debugging within VSCode Github Copilot. There were other cases where asking AI was useful like what protocols to use or explaining how something works from another code example found online.

### Prompts: 
> Is sending a digital signal through SPI from Rp2350 to fpga the best option? 

>  explain this code to me:

    // Run test I2C commands for the sake of testing the FPGA communication over I2C
    for (uint8_t tx = 0;; tx++) {

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

## Conclusion