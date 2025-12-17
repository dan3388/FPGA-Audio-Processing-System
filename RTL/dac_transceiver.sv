module dac_transceiver
(
    input  logic reset_n,     // Changed to active low to match your dac.sv
    input  logic input_clk,   // 12.288 MHz

    input  logic spi_mosi,
    input  logic spi_cs,

    // NEW DAC OUTPUT
    output logic dac_pdm_out, // Connect to RC Low Pass Filter
    
    output logic RED_LED,
    output logic GREEN_LED,
    output logic BLUE_LED
);

    // Keeping these for internal timing if needed, 
    // but they are no longer external I2S pins
    logic serial_clk;  // 3.072 MHz 

    logic [15:0] received_sound;
    logic [15:0] processed_sound;
    logic [1:0]  serial_clk_timer;

    // 1. SPI Receiver: Samples the ADC
    spi_receiver receiver
    (
        .reset(!reset_n), // Inverting if your spi_rx uses active high
        .serial_clk(serial_clk),
        .chip_select(spi_cs),
        .mosi(spi_mosi),
        .data_out(received_sound)
    );

    // 2. Signal Processor: Volume, EQ, etc.
    signal_processor DSP
    (
        .input_clk(input_clk),
        .raw_bits(received_sound),
        .processed_bits(processed_sound)
    );

    // 3. INTEGRATED DELTA-SIGMA DAC
    // Note: We use input_clk (12.288MHz) instead of serial_clk 
    // to get the highest possible oversampling ratio.
    delta_sigma_dac dac_inst (
        .clk(input_clk),
        .rst_n(reset_n),
        .pcm_in(processed_sound),
        .dac_out(dac_pdm_out)
    );

    // 4. Clock Generation for SPI Timing
    always_ff @(posedge input_clk or negedge reset_n) begin
        if (!reset_n) begin
            serial_clk_timer <= 0;
            serial_clk <= 0;
        end 
        else begin
            serial_clk_timer <= serial_clk_timer + 1;
            if (serial_clk_timer == 2'b01) begin // Divide by 4
                serial_clk <= !serial_clk; 
            end
        end
    end
    
endmodule