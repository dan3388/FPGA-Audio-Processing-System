// filepath: c:\Users\Daniel\Documents\WWU_F25\ENGR433\FPGA-Audio-Processing-System\FPGA-Audio-Processing-System\RTL\tb_dac.sv
module tb_delta_sigma_dac;

    logic clk;
    logic rst_n;
    logic [15:0] pcm_in;
    logic dac_out;

    // Instantiate DUT
    delta_sigma_dac dut (
        .clk(clk),
        .rst_n(rst_n),
        .pcm_in(pcm_in),
        .dac_out(dac_out)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #10 clk = ~clk; // 50MHz clock
    end

    // Test stimulus
    initial begin
        // Reset
        rst_n = 0;
        pcm_in = 16'h0000;
        #20 rst_n = 1;

        // Test 1: Zero input
        pcm_in = 16'h0000;
        #100;

        // Test 2: Positive signal (mid-range)
        pcm_in = 16'h4000;
        #100;

        // Test 3: Larger positive signal
        pcm_in = 16'h7FFF;
        #100;

        // Test 4: Negative signal (offset binary)
        pcm_in = 16'h8000;
        #100;

        // Test 5: Small negative signal
        pcm_in = 16'hC000;
        #100;

        $finish;
    end

    // Optional: Monitor output
    initial begin
        $monitor("Time: %t | pcm_in: %h | dac_out: %b", $time, pcm_in, dac_out);
    end

endmodule