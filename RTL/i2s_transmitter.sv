module i2s_transmitter
(
    input  logic reset,
    input  logic input_clk, // 12.288 MHz
    
    output logic serial_clk,
    output logic dac_mclk,
    output logic word_select,
    output logic sound_bit_out,
    output logic [5:0] bit_counter, // log_2(34) ~= 6 bits

    output logic test_LED_R,
    output logic test_LED_G,
    output logic test_LED_B
);

    /*
    input_clk = dac_mclk = 12.288 MHz
    serial_clk = 3.072 MHz –> 4 * serial_clk = input_clk
    sample rate = 48 kHz –> 3.072 MHz / 32 bits / 2 cycles = 48 kHz
    */

    logic [1:0] serial_clk_timer; // 12.288 MHz / 2^2 = 3.072 MHz
    logic [15:0] sound_data;

    assign dac_mclk = input_clk; // serial_clk = 46.072 MHz, dac_mclk = 4 * serial_clk = 3.072 MHz

    always_ff @(posedge input_clk or negedge reset) begin
        if (!reset) begin
            serial_clk_timer <= 0;
            serial_clk <= 0;
        end 
        else begin
            // Increment timer
            serial_clk_timer <= serial_clk_timer + 1;
            
            // Toggle clocks when timer rolls over
            if  (serial_clk_timer == 0)     serial_clk <= !serial_clk; 
        end
    end

    always_ff @(posedge serial_clk or negedge reset) begin
        if (!reset) begin
            bit_counter <= 31; // goes to 31, for 32 bits total, 16 bits each for left and right channels (right is silent in our case)
            sound_data <= 0;
            word_select <= 0;
            sound_bit_out <= 0;
            test_LED_R <= 1;
            test_LED_G <= 1;
            test_LED_B <= 1;
        end
        else begin // ALL NON-BLOCKING ASSIGNMENTS ONLY EFFECT LOGIC FOR NEXT CYCLE
            // incrementation
            if      (bit_counter == 31) bit_counter <= 0;
            else                        bit_counter <= bit_counter + 1; // if at 0, we are still at zero for all below logic due to non-blocking assignment

            // sound output
            /* 
            2 scenarios for left channel audio:
                bit_counter =  0, WS = 0 –> set to send out MSB       send out bit on bit_counter = 1
                bit_counter = 15, WS = 1 –> set to send send out LSB  send out bit on bit_counter = 16
            */
            if (bit_counter < 16) sound_bit_out <= sound_data[15 - bit_counter];
            else sound_bit_out <= sound_data[15 - bit_counter + 16]; // remove this and replace with the else statement for mono audio
            //else                  sound_bit_out <= 0;

            // left-right channel transitions
            // word_select changes 1 bit before the next 16 bit audio sample, thus the LSB of each sample is sent out on the "wrong" word_select value (but this is part of the I2S protocol)
            if (bit_counter == 31) word_select <= 0; // transition to left channel right before LSB of left-channel audio
            if (bit_counter == 15) word_select <= 1; // transition to right channel right before LSB of right-channel audio

            // take in new sound data if we're at the end of the bit_counter
            if (bit_counter == 31) sound_data <= sound_in; // take new sound data

            test_LED_R <= !test_LED_R;
            test_LED_G <= !test_LED_G;
            test_LED_B <= !test_LED_B;
        end
    end 

endmodule