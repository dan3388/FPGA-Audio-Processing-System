`timescale 1ns/1ps // Added timescale for clarity

module tb_audio_processor_transceiver;

    parameter CLK_PERIOD = 5; // Creates a 10ns period (100 MHz frequency)

    // internal
    logic reset;
    logic serial_clk;
    logic spi_chip_select = 1;
    logic spi_mosi = 0;

    // outputs
    wire i2s_ws;
    wire i2s_sound_bit_out;
    reg [5:0] i2s_bit_number;

    // inputs
    reg [15:0] input_bits;

    audio_processor_transceiver dut
    (
        .reset(reset),
        .serial_clk(serial_clk),
        .spi_chip_select(spi_chip_select),
        .spi_mosi(spi_mosi),
        .i2s_ws(i2s_ws),
        .i2s_sound_bit_out(i2s_sound_bit_out),
        .i2s_bit_number(i2s_bit_number)
    );

    initial begin
        serial_clk = 0;
        forever #CLK_PERIOD serial_clk = !serial_clk;
    end

    initial begin

        reset = 0;
        #CLK_PERIOD; // wait for reset to propogate
        reset = 1;

        @(negedge serial_clk); // synchronize
        spi_chip_select = 0;

        repeat (34) begin
            @(negedge serial_clk);
            spi_mosi = 1;
            @(posedge serial_clk);
        end
        repeat (34) begin
            @(negedge serial_clk);
            spi_mosi = !spi_mosi;
            @(posedge serial_clk);
        end
        repeat (34) begin
            @(negedge serial_clk);
            spi_mosi = 1;
            @(posedge serial_clk);
        end
        repeat (34) begin
            @(negedge serial_clk);
            spi_mosi = !spi_mosi;
            @(posedge serial_clk);
        end

        spi_chip_select = 1;
    end

    initial begin
        wait (spi_chip_select == 0 && i2s_bit_number == 33);
        $display("Inputting all ones\n\nNow Monitoring I2S output");
        repeat (136) begin // cycle through the first wave to get stable signals
            @(negedge serial_clk);
            $display("I2S_WS=%b , I2S_Bit_Number=%0d ,  I2S_Sound_Bit=%b", i2s_ws, i2s_bit_number, i2s_sound_bit_out);
        end

        spi_chip_select = 0;

        $display("Do these match?\n\n=== Testbench complete ===\n\n");
        $finish;
    end

endmodule
