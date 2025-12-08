`timescale 1ns/1ps // Added timescale for clarity

module tb_i2s_transmitter;

    parameter CLK_PERIOD = 5; // Creates a 10ns period (100 MHz frequency)

    logic rst;
    logic s_clk;
    logic ws;
    logic sound_bit_out;
    reg [5:0] i2s_bit_number;

    i2s_sound_test uut
    (
        .reset(rst),
        .serial_clk(s_clk),
        .word_select(ws),
        .sound_bit_out(sound_bit_out),
        .bit_counter(i2s_bit_number)
    );

    initial begin
        rst = 1;
        s_clk = 0;
        forever #CLK_PERIOD s_clk = !s_clk;
    end

    initial begin
        rst = 0;
        #CLK_PERIOD;
        rst = 1;
        wait (ws == 1)
        wait (ws == 0)
        repeat (68) begin
            @(posedge s_clk);
            $display("I2S_WS=%b , I2S_Bit_Number=%0d ,  I2S_Sound_Bit=%b", ws, i2s_bit_number, sound_bit_out);
            @(negedge s_clk);
        end
        $finish;
    end

endmodule
