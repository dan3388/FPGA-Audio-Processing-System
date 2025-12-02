`timescale 1ns/1ps // Added timescale for clarity

module tb_audio_processor_transceiver;

    parameter CLK_PERIOD = 5; // Creates a 10ns period (100 MHz frequency)

    // internal
    logic reset;
    logic serial_clk;
    logic spi_chip_select;
    logic spi_mosi;

    // outputs
    wire i2s_ws;
    wire i2s_sound_bit_out;

    // inputs
    reg [11:0] input_bits;
    reg [3:0] bit_shift;
    reg [4:0] i2s_out_bits_counter;

    audio_processor_transceiver dut
    (
        .reset(reset),
        .serial_clk(serial_clk),
        .spi_chip_select(spi_chip_select),
        .spi_mosi(spi_mosi),
        .i2s_ws(i2s_ws),
        .i2s_sound_bit_out(i2s_sound_bit_out)
    );

    initial begin
        serial_clk = 0;
        forever #CLK_PERIOD serial_clk = !serial_clk;
    end

    initial begin
        input_bits = 12'b111111111111;
        bit_shift = 11;
        i2s_out_bits_counter = 0;

        reset = 0;
        #CLK_PERIOD; // wait for reset to propogate
        #CLK_PERIOD; // wait for reset to propogate
        reset = 1;
        $display("\ninput_bits = %b\n", input_bits);

        @(negedge serial_clk); // synchronize
        spi_chip_select = 0;

        repeat (12) begin
            // spi_mosi = (input_bits >> bit_shift) & 1'b1;
            spi_mosi = 1;
            bit_shift = bit_shift - 1;
            @(negedge serial_clk);
        end

        spi_chip_select = 1;
    end

    initial begin
        i2s_out_bits_counter = 0;
        wait (spi_chip_select == 1);
        $display("Now Monitoring I2S output");
        repeat (26) begin
            @(posedge serial_clk);
            if (i2s_out_bits_counter != 0 && i2s_out_bits_counter != 13) begin
                $display("Bit %0d: %b", i2s_out_bits_counter, i2s_sound_bit_out);
            end
            i2s_out_bits_counter += 1;
        end

        spi_chip_select = 0;

        $display("Do these match?\n\n=== Testbench complete ===\n\n");
        $finish;
    end

endmodule
