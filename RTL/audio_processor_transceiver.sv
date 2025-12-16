module audio_processor_transceiver
(
    input  logic reset,
    input  logic input_clk,
    
    input  logic spi_chip_select,
    input  logic spi_mosi,

    output logic i2s_serial_clk,
    output logic i2s_dac_mclk,
    output logic i2s_ws,
    output logic i2s_sound_bit_out,

    output logic RED_LED,
    output logic BLUE_LED,
    output logic GREEN_LED
);

    logic [15:0] received_sound;
    logic [15:0] processed_sound;
  
    // THIS IS NOT CURRENT, RECEIVER HASN'T BEEN UPDATED SINCE IT WAS FIRST MADE
    // HASN'T BEEN UPDATED DUE TO NON-CURRENT STATUS
    spi_receiver receiver
    (
        .reset(reset),
        .s_clk(serial_clk),
        .cs(spi_chip_select),
        .mosi(spi_mosi),
        .data_out(received_sound)
    );

    signal_processor DSP
    (
        .input_clk(input_clk),
        .raw_bits(received_sound),
        .processed_bits(processed_sound)
    );

    i2s_transmitter i2s_transmitter
    (
        .reset(reset),
        .input_clk(input_clk),
        .serial_clk(i2s_serial_clk),
        .dac_mclk(i2s_dac_mclk),
        .word_select(i2s_ws),
        .sound_bit_out(i2s_sound_bit_out),
        .RED_LED(RED_LED),
        .BLUE_LED(BLUE_LED),
        .GREEN_LED(GREEN_LED)
    );

endmodule