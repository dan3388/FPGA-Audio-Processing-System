module i2s_transmitter
(
    input  logic reset,
    input  logic s_clk,
    input  logic [15:0] sound_in,
    output logic word_select,
    output logic sound_bit_out,
    output logic [5:0] bit_counter // log_2(34) ~= 6 bits
);

    reg [15:0] sound_data;

    always_ff @(posedge s_clk or negedge reset) begin
        if (!reset) begin
            bit_counter <= 33; // 0-33 is 0 through 16 = 16 + 17 more = 0 through 16 + 17 = 33
            sound_data <= 0;
            word_select <= 0;
            sound_bit_out <= 0;
        end
        else begin // ALL NON-BLOCKING ASSIGNMENTS ONLY EFFECT LOGIC FOR NEXT CYCLE
            // incrementation
            if      (bit_counter == 33) bit_counter <= 0;
            else                        bit_counter <= bit_counter + 1; // if at 0, we are still at zero for all below logic due to non-blocking assignment

            // sound output
            /* 
            Because non-blocking assignments effect the next cycle, not this cycle, 
            we want to set the output audio bit for next value of bit_counter, during this cycle.
            */
            if (word_select == 0 && bit_counter < 16) begin 
                sound_bit_out <= sound_data[15 - bit_counter]; 
                // bit-counter ranges from 0-15 for the if condition, giving array index range from 15-0
                // if on bit 0, we set audio output bit for bit 1. If on bit 15, we set audio output bit for bit 16
            end
            else sound_bit_out <= 0; // everything else should be zeros for silence

            // left-right channel transitions
            if      (bit_counter == 33) word_select <= 0; // transition to left channel after bit counter 34, which is the last audio bit
            else if (bit_counter == 16) word_select <= 1; // transition to right channel after bit 16, but counter 17

            // take in new sound data if we're at the end of the bit_counter
            if (bit_counter == 33) sound_data <= sound_in; // take new sound data
        end
    end 

endmodule