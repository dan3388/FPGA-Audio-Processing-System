module i2s_transmitter
(
    input  logic reset,
    input  logic [11:0] silence,
    input  logic s_clk,
    input  logic [11:0] sound_in,
    output logic word_select,
    output logic sound_bit_out
);

    reg [11:0] sound_data;
    reg [5:0] bit_counter; // log_2(26) ~= 5 bits

    always_ff @(posedge s_clk or negedge reset) begin
        if (!reset) begin
            bit_counter <= 0;
            sound_data <= silence;
            word_select <= 0;
            sound_bit_out <= 0;
        end
        else begin
            // incrementation
            if      (bit_counter == 25) bit_counter <= 0;
            else                        bit_counter <= bit_counter + 1;

            // left-right channel transitions
            if      (bit_counter == 25) word_select <= 0; // transition to left channel
            else if (bit_counter == 12) word_select <= 1; // transition to right channel

            // take in new sound data if we're at the end of the bit_counter
            if (bit_counter == 25) sound_data <= sound_in; // take new sound data

            // sound output
            if (word_select == 0) begin
                if      (bit_counter == 0)  sound_bit_out <= 0; // I2S has 1 bit delay
                else if (bit_counter < 13) sound_bit_out <= sound_data[11 - bit_counter];
            end
            else begin
                if      (bit_counter == 13)  sound_bit_out <= 0; // I2S has 1 bit delay
                else if (bit_counter <= 26)  sound_bit_out <= silence[11 - bit_counter + 13]; // We need to send something equivalent to silence.
            end
        end
    end 

endmodule