module i2s_receiver
(
    input logic reset,
    input logic serial_clk,
    input logic word_select,
    input logic sound_data,

    output logic [15:0] received_bits,

    output logic warning_LED
);

    logic [4:0] bit_counter;
    logic [15:0] shift_reg;
    logic previous_ws;

    always_ff @(posedge serial_clk or negedge reset) begin
        if (!reset) begin
            bit_counter <= 31;
            shift_reg <= 0;
            previous_ws <= 0;
        end
        else begin
            // Incrementation
            bit_counter <= bit_counter + 1; // no need to set back to zero because of roll-over

            // bit_counter sync
            if (word_select == 0 && previous_ws == 1) begin
                bit_counter <= 31; // MSB is next cycle, during which we arbitrarily decide bit_counter should equal 0
            end

            // save this cycle's word_select
            previous_ws <= word_select;

            // data reception
            if (previous_ws == 0 && word_select == 1 && bit_counter < 16) begin // LSB
                shift_reg <= {shift_reg[14:0], sound_data};
                if (bit_counter != 15) warning_LED <= 0; // bit_counter is out of sync if this is true
                received_bits <= shift_reg;
            end
            else if (word_select == 0 && bit_counter < 16) shift_reg <= {shift_reg[14:0], sound_data};
        end
    end

endmodule
