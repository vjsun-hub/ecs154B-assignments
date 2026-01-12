`timescale 1ns / 1ps

module vending_machine_tb;

    logic clk;
    logic rst_n;
    logic nickel;
    logic dime;
    logic dispense;
    logic change;

    vending_machine dut (
        .clk(clk),
        .rst_n(rst_n),
        .nickel(nickel),
        .dime(dime),
        .dispense(dispense),
        .change(change)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        $dumpfile("vending_machine.vcd");
        $dumpvars(0, vending_machine_tb);
    end

    task reset_dut();
        rst_n = 0;
        nickel = 0;
        dime = 0;
        #15 rst_n = 1;
        $display("--- System Reset ---");
    endtask

    initial begin
        reset_dut();
        
        // Scenario 1: 30 cents (3 Dimes)
        $display("Scenario 1: Testing 3 Dimes...");
        insert_coin(0, 1, 0, 0); // Dime, expect 0
        insert_coin(0, 1, 0, 0); // Dime, expect 0
        insert_coin(0, 1, 1, 0); // Dime, expect Dispense=1, Change=0
        
        #20;

        // Scenario 2: 30 cents (2 Dimes, 2 Nickels)
        $display("Scenario 2: Testing 2 Dimes, 2 Nickels...");
        insert_coin(0, 1, 0, 0); // Dime
        insert_coin(0, 1, 0, 0); // Dime
        insert_coin(1, 0, 0, 0); // Nickel
        insert_coin(1, 0, 1, 0); // Nickel, expect Dispense=1
        
        #20;

        // Scenario 3: 35 cents (25c + Dime)
        $display("Scenario 3: Testing 35 cents (25c then Dime)...");
        insert_coin(0, 1, 0, 0); // 10
        insert_coin(0, 1, 0, 0); // 20
        insert_coin(1, 0, 0, 0); // 25
        insert_coin(0, 1, 1, 1); // 35, expect Dispense=1, Change=1

        #50;
        $display("All tests finished.");
        $finish;
    end

    // Helper Task to insert a coin and check Mealy outputs
    task insert_coin(input logic n, input logic d, input logic exp_disp, input logic exp_chg);
        @(posedge clk);
        #1; 
        nickel = n;
        dime = d;
        
        // Wait just before the next clock edge to sample Mealy outputs
        // Clock period is 10ns. We are at (posedge + 1ns).
        // Waiting #8 takes us to (posedge + 9ns), just before the next edge.
        #8; 
        
        if (dispense !== exp_disp || change !== exp_chg) begin
            $display("ERROR: Time %0t | In: N=%b D=%b | Got: Disp=%b Chg=%b | Exp: %b %b", 
                     $time, n, d, dispense, change, exp_disp, exp_chg);
        end else if (exp_disp || exp_chg) begin
            $display("PASS:  Time %0t | Dispensed successfully", $time);
        end

        @(posedge clk);
        #1;
        nickel = 0;
        dime = 0;
    endtask

endmodule