`timescale 1ns/1ps // Added timescale for clarity

module tb_i2s_transmitter
(
    input  logic rst,
    input  logic [11:0] sound_of_silence,
    input  logic serial_clk,
    input  logic [11:0] sound_bits_in,
    output logic ws,
    output logic sound_out 
)

    i2s_transmitter uut
    (
        .reset(rst),
        .silence(sound_of_silence),
        .s_clk(serial_clk),
        .sound_in(sound_bits_in),
        .word_select(ws),
        .sound_bit_out(sound_out)
    );

    initial begin
        serial_clk = 0;
        forever #CLK_PERIOD serial_clk = !serial_clk;
    end

endmodule