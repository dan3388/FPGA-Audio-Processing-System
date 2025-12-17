module tb_spi_receiver;

    // Testbench signals
    logic reset;
    logic serial_clk;
    logic chip_select;
    logic mosi;
    logic [15:0] data_out;
    logic [15:0] shift_reg_out;

    // Instantiate the spi_receiver module
    spi_receiver uut (
        .reset(reset),
        .serial_clk(serial_clk),
        .chip_select(chip_select),
        .mosi(mosi),
        .data_out(data_out),
        .shift_reg_out(shift_reg_out)
    );

    // Clock generation
    initial begin
        serial_clk = 0;
        forever #5 serial_clk = ~serial_clk; // 100 MHz clock
    end

    // Test sequence
    initial begin
        // Initialize signals
        reset = 0;
        chip_select = 1; // Inactive
        mosi = 0;

        // Apply reset
        #10;
        reset = 1;

        // Start SPI communication
        #10;
        chip_select = 0; // Active

        // Send 16 bits of data (example: 0xA5A5)
        for (int i = 15; i >= 0; i--) begin
            mosi = (16'hA5A5 >> i) & 1;
            #10; // Wait for one clock cycle
            $display("shift_reg=%0h", shift_reg_out);
        end

        // Deactivate chip select
        #10;
        chip_select = 1; // Inactive

        // Wait and finish simulation
        #50;
        $finish;
    end
endmodule