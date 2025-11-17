module spi_receiver
(
    input  logic reset,
    input  logic s_clk, // serial clock
    input  logic cs,   // chip-select
    input  logic mosi, // master out, slave in (we receive only)
    output logic [11:0] data_out
);

    reg [11:0] shift_reg;
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
                if (bit_count < 12) begin
                    shift_reg <= {shift_reg[10:0], mosi};
                    bit_count <= bit_count + 1;
                end
                else if (bit_count == 12) begin
                    data_out <= shift_reg;
                    bit_count <= bit_count + 1;
                end
                else if (bit_count < 25) begin
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