module tb_audio_processor_transceiver;

parameter CLK_PERIOD = 1000; // Creates a 10ns period (100 MHz frequency)

reg reset;
reg serial_clk;
reg spi_chip_select;
reg spi_mosi;
reg i2s_ws;
reg i2s_sound_bit_out;

audio_processor_transceiver data_out
(
    .reset(reset),
    .serial_clk(serial_clk),
    .spi_chip_select(spi_chip_select),
    .spi_mosi(spi_mosi),
    .i2s_ws(i2s_ws),
    .i2s_sound_bit_out(i2s_sound_bit_out)
);

reset = 0;
#5
reset = 1;





endmodule