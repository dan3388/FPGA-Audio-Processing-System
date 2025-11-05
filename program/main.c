/*
 * MIT License
 *
 * Copyright (c) 2023 tinyVision.ai
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "ice_cram.h"
#include "ice_fpga.h"

#define FPGA_BINARY rgb_blink

int main(void) 
{
    // initialize FPGA
    ice_fpga_init(FPGA_DATA, 48);
    ice_fpga_start(FPGA_DATA);

    // Write the whole bitstream from our SystemVerilog to the FPGA CRAM
    ice_cram_open(FPGA_DATA);
    ice_cram_write(FPGA_BINARY, sizeof(FPGA_BINARY));
    ice_cram_close();

    adc_init();
    adc_gpio_init(41);
    adc_select_input(1); // ADC1 is GPIO41

    while (true)
    {
        // sample and send to iCE40 FPGA for DSP as I2S signal
    }
    return 0;
}
