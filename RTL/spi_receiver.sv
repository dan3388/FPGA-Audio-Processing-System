module spi_receiver 
(
    input  logic reset,
    input  logic serial_clk,
    input  logic chip_select,
    input logic mosi,
    output logic [15:0] data_out,
    output logic [15:0] shift_reg_out // this is only for verification purposes
);

    reg [15:0] shift_reg;
    reg [5:0] bit_count;

    assign shift_reg_out = shift_reg;

    always_ff @(posedge serial_clk or negedge reset) begin
        if (!reset) begin
            // reset received data
            shift_reg <= 0;
            bit_count <= 0;
            data_out <= 0;
        end
        else begin
            // Active low chip select
            if (chip_select) begin
                bit_count <= 0;
                shift_reg <= 0;
            end
            else begin
                if (bit_count < 15) begin
                    shift_reg <= {shift_reg[14:0], mosi};
                    bit_count <= bit_count + 1;
                end
                else if (bit_count == 15) begin
                    
                    shift_reg <= {shift_reg[14:0], mosi}; // added when getting test bench to show the right thing
                    data_out <= shift_reg;
                    bit_count <= bit_count + 1;
                
                    // shift_reg <= {shift_reg[14:0], miso};
                    // data_out <= {shift_reg[14:0], miso}; // capture the newly shifted-in bit too
                    // bit_count <= bit_count + 1;
                end
                else if (bit_count < 31) begin
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