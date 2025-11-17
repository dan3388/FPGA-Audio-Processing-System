module signal_processor
(
    input  logic s_clk, // serial clock from SPI, but we use this clock for everything to prevent domain crossing
    input  logic [11:0] raw_bits, // the bits we got from SPI from RP2350B ADC
    output logic [11:0] processed_bits // processed bits
);
    // pass-through; figure out DSP once we get a working prototype
    assign processed_bits = raw_bits;
endmodule