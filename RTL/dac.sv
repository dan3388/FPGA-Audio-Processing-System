module delta_sigma_dac 
(
    input  logic clk,         // Fast System Clock (e.g. 50-100MHz)
    input  logic rst_n,
    input  logic [15:0] pcm_in, // Signed 16-bit input
    output logic dac_out      // 1-bit PDM output
);

    logic [16:0] accumulator; // 1 bit wider than input
    logic [15:0] unsigned_in;

    // Convert Signed PCM (-32768 to 32767) to Unsigned (0 to 65535)
    // We invert the MSB (offset binary conversion)
    assign unsigned_in = {~pcm_in[15], pcm_in[14:0]};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accumulator <= 17'd0;
            dac_out     <= 1'b0;
        end else begin
            // First-order Delta-Sigma Modulator
            // Accumulate the input. When it overflows, output a pulse.
            accumulator <= accumulator[15:0] + unsigned_in;
            dac_out     <= accumulator[16];
        end
    end
endmodule