`timescale 1ns/1ps // Added timescale for clarity

module tb_i2s_sound_test;

    parameter CLK_PERIOD = 5; // Creates a 10ns period (100 MHz frequency)

    logic reset;
    logic input_clk; // 12.288 MHz
    logic serial_clk;
    logic dac_mclk;
    logic word_select;
    logic sound_bit_out;

    logic serial_clk_analyzer;
    logic input_clk_analyzer;
    logic ws_analyzer;
    logic sound_bit_analyzer;

    logic test_LED;
    logic test_LED_R;
    logic test_LED_G;
    logic test_LED_B;

    logic [5:0] bit_counter;

    i2s_sound_test uut
    (
        .reset(reset),
        .input_clk(input_clk),
        .serial_clk(serial_clk),
        .dac_mclk(dac_mclk),
        .word_select(word_select),
        .sound_bit_out(sound_bit_out),
        .serial_clk_analyzer(serial_clk_analyzer),
        .input_clk_analyzer(input_clk_analyzer),
        .test_LED(test_LED),
        .test_LED_R(test_LED_R),
        .test_LED_G(test_LED_G),
        .test_LED_B(test_LED_B),
        .bit_counter(bit_counter)
    );

    initial begin
        input_clk = 0;
        forever #CLK_PERIOD input_clk = !input_clk;
    end

    initial begin
        reset = 0;
        #CLK_PERIOD;
        reset = 1;
        wait (word_select == 1)
        wait (word_select == 0)
        repeat (700) begin
            @(posedge serial_clk);
            $display("I2S_WS=%b , I2S_Bit_Number=%0d ,  I2S_Sound_Bit=%b", word_select, bit_counter, sound_bit_out);
            @(negedge serial_clk);
        end
        $finish;
    end

endmodule
