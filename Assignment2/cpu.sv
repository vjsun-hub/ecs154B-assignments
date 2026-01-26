module cpu(
    input logic clk,
    input logic reset
);

    // Program Counter
    logic [7:0] PC;

    // Instruction Register
    logic [7:0] IR;

    // Registers (2 registers: r0 and r1)
    logic [7:0] registers [0:1];

    // Memory (256 bytes)
    logic [7:0] memory [0:255];

    // Control signals
    logic [3:0] opcode;
    logic ds, s;
    logic [1:0] extra;
    logic [2:0] immediate;
    logic [3:0] offset;

    // ALU signals
    logic [7:0] alu_a, alu_b, alu_out;
    logic alu0, alu1;

    // State
    logic halted;

    // TODO: Decode instruction fields from IR
    // Hint: Extract opcode, ds, s, immediate, offset based on instruction format

    // TODO: Implement ALU
    // Hint: Use alu0 and alu1 to determine operation
    // Add (11), Sub (10), Mult (01), Nand (00)
    always_comb begin
        // TODO: Implement ALU logic based on alu0 and alu1
        alu_out = 8'b0;
    end

    // TODO: Implement main CPU logic
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // TODO: Initialize PC, halted flag, registers, and memory
        end else if (!halted) begin
            // TODO: Fetch instruction from memory[PC]

            // TODO: Decode and Execute based on opcode
            // Implement each instruction:
            // - nand (0000)
            // - add (0001)
            // - addm (0010)
            // - addi (0011)
            // - sub (0100)
            // - mult (0101)
            // - lw (0110)
            // - sw (0111)
            // - beq (1000)
            // - jmp (1001)
            // - halt (1111)

            // TODO: Update PC appropriately for each instruction
        end
    end

endmodule
