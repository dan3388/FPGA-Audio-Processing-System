module audio_processor_transceiver
(
    input  logic reset,
    input  logic serial_clk,
    input  logic spi_chip_select,
    input  logic spi_mosi,
    output logic i2s_ws,
    output logic i2s_sound_bit_out
);

    localparam [11:0] sound_of_silence = 12'd2048;
    
    logic [11:0] received_sound;
    logic [11:0] processed_sound;

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
        .s_clk(serial_clk),
        .raw_bits(received_sound),
        .processed_bits(processed_sound)
    );

    i2s_transmitter i2s_transmitter
    (
        .reset(reset),
        .silence(sound_of_silence),
        .s_clk(serial_clk),
        .sound_in(processed_sound),
        .word_select(i2s_ws),
        .sound_bit_out(i2s_sound_bit_out)
    );

endmodule