`timescale 1ns / 1ps

module tb_dac_transreceiver;

    // Signals
    logic clk_12mhz;
    logic rst_n;
    logic spi_mosi;
    logic spi_cs;
    logic serial_clk;
    logic dac_pdm_out;
    
    // Test data
    logic [15:0] test_vector;

    // Instantiate Top Module
    audio_processor_transceiver dut (
        .reset_n(rst_n),
        .input_clk(clk_12mhz),
        .spi_mosi(spi_mosi),
        .spi_cs(spi_cs),
        .serial_clk(serial_clk),
        .dac_pdm_out(dac_pdm_out),
        .RED_LED(),
        .GREEN_LED(),
        .BLUE_LED()
    );

    // Generate 12.288 MHz Clock 
    // Period is approx 81.38ns -> half period ~40.69ns
    initial begin
        clk_12mhz = 0;
        forever #40.69 clk_12mhz = ~clk_12mhz;
    end

    // Task to simulate the ADC sending a 16-bit word over SPI MOSI
    task send_spi_word(input [15:0] data);
        begin
            @(negedge serial_clk);
            spi_cs = 0; // Select the "ADC"
            for (int i = 15; i >= 0; i--) begin
                spi_mosi = data[i];
                @(negedge serial_clk);
            end
            spi_cs = 1;
            #100; // Gap between samples
        end
    endtask

    // Stimulus
    initial begin
        // Initialize
        rst_n = 0;
        spi_mosi = 0;
        spi_cs = 1;
        test_vector = 16'h0000;

        // Reset system
        #200 rst_n = 1;
        #100;

        // Test 1: Send "Zero" (Mid-scale for audio)
        $display("Sending 0x0000 (Mid-scale)...");
        send_spi_word(16'h0000);
        #5000; 

        // Test 2: Send Positive Peak
        $display("Sending 0x7FFF (Positive Peak)...");
        send_spi_word(16'h7FFF);
        #5000;

        // Test 3: Send Negative Peak
        $display("Sending 0x8000 (Negative Peak)...");
        send_spi_word(16'h8000);
        #5000;

        // Test 4: Send a Sine-wave-like value
        $display("Sending 0x4000...");
        send_spi_word(16'h4000);
        #10000;

        $display("Simulation Finished");
        $finish;
    end

    // Monitor for verification
    initial begin
        $monitor("Time: %t | CS: %b | MOSI: %b | PDM Out: %b", $time, spi_cs, spi_mosi, dac_pdm_out);
    end

endmodule