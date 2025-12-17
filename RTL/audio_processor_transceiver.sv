module audio_processor_transceiver
(
    input  logic reset,
    input  logic input_clk, // 12.288 MHz

    input  logic spi_miso,
    output logic spi_mosi,
    output logic spi_cs,

    output logic serial_clk, // 3.072 MHz –> 48 kHz sample rate

    output logic i2s_dac_mclk,
    output logic i2s_transmit_ws,
    output logic i2s_transmit_sd,
    output logic [4:0] i2s_transmit_bit_counter, // ONLY FOR TESTBENCH

    output logic RED_LED,
    output logic BLUE_LED,
    output logic GREEN_LED
);

    /*
    input_clk = dac_mclk = 12.288 MHz
    serial_clk = 3.072 MHz –> 4 * serial_clk = input_clk
    sample rate = 48 kHz –> 3.072 MHz / 32 bits / 2 cycles = 48 kHz
    */

    logic [15:0] received_sound;
    logic [15:0] processed_sound;

    logic [1:0] serial_clk_timer;
    // logic [4:0] i2s_transmit_bit_counter; // for real use

    assign i2s_dac_mclk = input_clk; // serial_clk = 46.072 MHz, dac_mclk = 4 * serial_clk = 3.072 MHz

    // Instantiations
    spi_receiver receiver
    (
        .reset(reset),
        .serial_clk(serial_clk),
        .chip_select(spi_cs),
        .miso(spi_miso),
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
        .sound_in(processed_sound),
        .serial_clk(serial_clk),
        .word_select(i2s_transmit_ws),
        .sound_bit_out(i2s_sound_bit_out),
        .bit_counter(i2s_transmit_bit_counter)
    );

    always_ff @(posedge input_clk or negedge reset) begin
        if (!reset) begin
            serial_clk_timer <= 0;
            serial_clk <= 0;
        end 
        else begin
            // Increment timer
            serial_clk_timer <= serial_clk_timer + 1;
            
            // Toggle clocks when timer rolls over
            if (serial_clk_timer == 0) serial_clk <= !serial_clk; 
        end
    end
    
endmodule