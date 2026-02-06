module vending_machine (
    input  logic clk,
    input  logic rst_n,    // Active low reset
    input  logic nickel,   // 5 cents
    input  logic dime,     // 10 cents
    output logic dispense, // Merchandise out
    output logic change    // 5 cent nickel change
);

    // TODO: Implement the vending machine logic here
    /*
    Implement a mealy model of a vending machine. 
    The vending machine accepts nickel and dime only. 
    When a total of 30 cents is inserted, the machine dispenses 
    a product and returns any change if necessary.
    */

    localparam [2:0] 
        S0 = 3'd0, // 0 cents
        S5 = 3'd1, // 5 cents
        S10 = 3'd2, 
        S15 = 3'd3, 
        S20 = 3'd4, 
        S25 = 3'd5;

    logic [2:0] curr_state, next_state;

    always @(*) begin
        next_state = curr_state;
        dispense = 1'b0;
        change = 1'b0; //equal to 0

        case (curr_state)
            S0: begin
                if (nickel) next_state = S5;
                else if (dime) next_state = S10;
            end
            S5: begin
                if (nickel) next_state = S10;
                else if (dime) next_state = S15;
            end
            S10: begin
                if (nickel) next_state = S15;
                else if (dime) next_state = S20;
            end
            S15: begin
                if (nickel) next_state = S20;
                else if (dime) next_state = S25;
            end
            S20: begin
                if (nickel) next_state = S25;
                else if (dime) begin
                    dispense = 1'b1; //dispense the product, equal to 1
                    next_state = S0;
                end
            end
            S25: begin
                if (nickel) begin
                    dispense = 1'b1;
                    next_state = S0;
                end
                else if (dime) begin
                    dispense = 1'b1;
                    change = 1'b1; //returns a nickel as change
                    next_state = S0;
                end
            end

            default: begin
                next_state = S0;
            end

        endcase
    end

    //state transitions logic, for active low reset
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            curr_state <= S0;
        end else begin
            curr_state <= next_state;
        end
    end

endmodule
