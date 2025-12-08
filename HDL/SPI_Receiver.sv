module spi_receiver
(
    input  logic reset,
    input  logic s_clk, // serial clock
    input  logic cs,   // chip-select
    input  logic mosi, // master out, slave in (we receive only)
    output logic [15:0] data_out
);

    reg [15:0] shift_reg;
    reg [5:0] bit_count;

    always_ff @(posedge s_clk or negedge reset) begin
        if (!reset) begin
            // reset received data
            shift_reg <= 0;
            bit_count <= 0;
            data_out <= 0;
        end
        else begin
            if (cs) begin
                bit_count <= 0;
                shift_reg <= 0;
            end
            else begin
                if (bit_count < 15) begin
                    shift_reg <= {shift_reg[14:0], mosi};
                    bit_count <= bit_count + 1;
                end
                else if (bit_count == 15) begin
                    /*
                    shift_reg <= {shift_reg[14:0], mosi}; // added when getting test bench to show the right thing
                    data_out <= shift_reg;
                    bit_count <= bit_count + 1;
                    */
                    shift_reg <= {shift_reg[14:0], mosi};
                    data_out <= {shift_reg[14:0], mosi}; // capture the newly shifted-in bit too
                    bit_count <= bit_count + 1;
                end
                else if (bit_count < 34) begin
                    bit_count <= bit_count + 1;
                end
                else begin
                    bit_count <= 0;
                    shift_reg <= 0;
                end
            end
        end
    end

endmodule