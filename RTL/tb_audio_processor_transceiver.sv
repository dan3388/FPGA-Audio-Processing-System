`timescale 1ns/1ps // Added timescale for clarity

module tb_audio_processor_transceiver;

    parameter INPUT_CLK_PERIOD = 5; // Creates a 10ns period (100 MHz frequency)
    parameter SERIAL_CLK_PERIOD = 20;

    // inputs
    logic reset_tb;
    logic input_clk_tb;
    logic spi_mosi_tb;
    logic spi_cs_tb;

    // outputs
    logic serial_clk_tb;
    logic i2s_dac_mclk_tb;
    logic i2s_ws_tb;
    logic i2s_sd_tb;
    logic RED_LED_tb;
    logic GREEN_LED_tb;
    logic BLUE_LED_tb;

    // verification specific outputs:
    logic [4:0] i2s_bit_counter_tb;

    audio_processor_transceiver dut
    (
        .reset(reset_tb),
        .input_clk(input_clk_tb),
        .spi_mosi(spi_mosi_tb),
        .spi_cs(spi_cs_tb),
        .serial_clk(serial_clk_tb),
        .i2s_dac_mclk(i2s_dac_mclk_tb),
        .i2s_ws(i2s_ws_tb),
        .i2s_sd(i2s_sd_tb),
        .RED_LED(RED_LED_tb),
        .GREEN_LED(GREEN_LED_tb),
        .BLUE_LED(BLUE_LED_tb),
        .i2s_bit_counter(i2s_bit_counter_tb)
    );

    initial begin
        input_clk_tb = 0;
        forever #INPUT_CLK_PERIOD input_clk_tb = !input_clk_tb;
    end

    initial begin

        reset_tb = 0;
        #SERIAL_CLK_PERIOD; // wait for reset to propogate
        reset_tb = 1;

        spi_mosi_tb = 0;
        spi_cs_tb = 1;

        @(negedge serial_clk_tb); // synchronize
        spi_cs_tb = 0;
        repeat (32) begin
            @(negedge serial_clk_tb);
            spi_mosi_tb = 1;
            @(posedge serial_clk_tb);
        end
        repeat (32) begin
            @(negedge serial_clk_tb);
            spi_mosi_tb = 1;
            @(posedge serial_clk_tb);
        end
        repeat (32) begin
            @(negedge serial_clk_tb);
            spi_mosi_tb = 1;
            @(posedge serial_clk_tb);
        end
        repeat (32) begin
            @(negedge serial_clk_tb);
            spi_mosi_tb = 1;
            @(posedge serial_clk_tb);
        end

        spi_cs_tb = 1;
    end

    initial begin
        wait (spi_cs_tb == 0 && i2s_bit_counter_tb == 0);
        $display("Inputting all ones\n\nNow Monitoring I2S output");
        repeat (136) begin // cycle through the first wave to get stable signals
            @(negedge serial_clk_tb);
            $display("I2S_WS=%b , I2S_Bit_Number=%0d ,  I2S_SD=%b", i2s_ws_tb, i2s_bit_counter_tb, i2s_sd_tb);
        end

        spi_cs_tb = 0;

        $display("Do these match?\n\n=== Testbench complete ===\n\n");
        $finish;
    end

endmodule
