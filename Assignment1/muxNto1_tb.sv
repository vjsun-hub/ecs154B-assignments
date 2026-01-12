`timescale 1ns / 1ps

module muxNto1_tb;

    // Parameters for this specific test
    localparam int TEST_N = 5;      // Let's test with 5 inputs
    localparam int TEST_WIDTH = 8;  // 8-bit data width

    // Testbench Signals
    logic [TEST_N-1:0][TEST_WIDTH-1:0] in_bus;
    logic [$clog2(TEST_N)-1:0]         sel;
    logic [TEST_WIDTH-1:0]             out;

    muxNto1 #(
        .N(TEST_N),
        .WIDTH(TEST_WIDTH)
    ) dut (
        .in_bus(in_bus),
        .sel(sel),
        .out(out)
    );

    // Waveform Generation (GTKWave)
    initial begin
        $dumpfile("mux_test.vcd");
        $dumpvars(0, muxNto1_tb);
    end

    // Stimulus
    initial begin
        // Initialize the input bus with unique values for each port
        // Port 0 = 10, Port 1 = 11, Port 2 = 12, etc.
        for (int i = 0; i < TEST_N; i++) begin
            in_bus[i] = i + 10; 
        end

        $display("Starting Mux Test for N=%0d...", TEST_N);
        $display("------------------------------------");

        // Loop through every possible selection
        for (int j = 0; j < TEST_N; j++) begin
            sel = j;
            #10; // Wait for logic to propagate
            
            $display("Time: %t | Sel: %0d | Expected: %0d | Got: %0d", 
                     $time, sel, in_bus[j], out);
            
            if (out !== in_bus[j]) begin
                $display("ERROR: Mismatch at selection %0d", j);
            end
        end

        #10;
        $display("------------------------------------");
        $display("Test Complete.");
        $finish;
    end

endmodule