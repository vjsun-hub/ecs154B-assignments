// ============================================================================
// MODULE: Memory (Single-Port RAM)
// ============================================================================
module memory_module (
    input  logic       clk,
    input  logic [7:0] addr,          // Memory address
    input  logic       write_en,      // Write enable
    input  logic [7:0] data_in,       // Data to write
    output logic [7:0] data_out       // Data read
);
    // Memory storage - exposed for testbench initialization
    logic [7:0] mem [0:255];

    // Initialize memory to 0
    initial begin
        for (int i=0; i<256; i++) mem[i] = 0;
    end

    // Asynchronous read
    assign data_out = mem[addr];

    // Synchronous write
    always_ff @(posedge clk) begin
        if (write_en) begin
            mem[addr] <= data_in;
        end
    end
endmodule

// ============================================================================
// MODULE: ALU
// ============================================================================
module alu (
    input  logic [7:0] a,
    input  logic [7:0] b,
    input  logic [1:0] func, // 00: NAND, 01: MULT, 10: SUB, 11: ADD
    output logic [7:0] result,
    output logic       zero // Indicates if result is zero
);
    // TODO: Implement ALU operations
    always_comb begin
        result = 8'b0000_0000;

        case (func)
            2'b00: result = ~(a & b); //NAND
            2'b01: result = a * b; // MULT
            2'b10: result = a - b;  //SUB
            2'b11: result = a + b;  //ADD
            default: result = 8'b0000_0000;
        endcase

        zero = (result == 8'b0000_0000); // set zero flag if result is zero
    end
    
endmodule

// ============================================================================
// MODULE: Control Unit
// ============================================================================
module control_unit (
    input  logic       clk,
    input  logic       reset,
    input  logic [3:0] opcode,
    input  logic       alu_zero,  // From ALU, for BEQ
    
    // Status
    output logic       halted,
    
    // Control Signals
    // TODO: Add more control signals as needed
    output logic       mem_write, //SW
    output logic       reg_write, //register file write enable
    output logic       ir_write, //latch IR <= mem_data_out during FETCH
    output logic [1:0] alu_func, //selects ALU operation
    output logic [1:0] alu_src_a_sel, // ALU input A
    output logic [1:0] alu_src_b_sel, // ALU input B
    output logic       pc_inc,        //FETCH: PC = PC+1
    output logic       pc_add_offset, // BEQ/JMP: PC <= PC+offset
    output logic       pc_offset_is_jmp, // choose offset field for BEQ vs JMP
    output logic       wb_sel_mem,      // LW: write back from mem
    output logic       mem_addr_is_pc // FETCH uses PC, data ops use rs
    
);
    
    typedef enum logic [2:0] {
        // TODO: Define states
        FETCH = 3'b000,
        EXEC = 3'b001,
        ADDM = 3'b010, //extra cycle for ADDM (use mem_data_out as ALU input)
        HALT = 3'b011
    } state_t;

    // TODO: Implement state machine and control signal generation
    state_t curr_state, next_state;

    always_comb begin
        next_state = curr_state;

        case (curr_state)
            FETCH: next_state = EXEC;

            EXEC: begin
                case (opcode)
                    4'b1111: next_state = HALT; 
                    4'b0010: next_state = ADDM;
                    default: next_state = FETCH; 
                endcase
            end

            ADDM: next_state = FETCH; //fetch next inst after 2nd cycle of ADDM
            HALT: next_state = HALT;
            default: next_state = FETCH;

        endcase
    end


    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            curr_state <= FETCH;
        end else begin
            curr_state <= next_state;
        end
    end

    // Control outputs 
    always_comb begin
        halted = 1'b0;
        ir_write = 1'b0;
        pc_inc = 1'b0;
        pc_add_offset = 1'b0;
        pc_offset_is_jmp = 1'b0;
        mem_addr_is_pc = 1'b0;
        mem_write = 1'b0;
        reg_write = 1'b0;
        wb_sel_mem = 1'b0;

        alu_src_a_sel = 2'b00;
        alu_src_b_sel = 2'b00;
        alu_func = 2'b11; 

        case (curr_state)
            FETCH: begin
                mem_addr_is_pc = 1'b1; // mem_addr mux selects PC
                ir_write = 1'b1; //IR <= mem_data_out
                pc_inc = 1'b1;   // PC <= PC + 1
            end

            EXEC: begin
                case (opcode)
                    // R-type: rds = f(rds, rs)
                    4'b0000: begin //NAND
                        alu_func = 2'b00;
                        alu_src_a_sel = 2'b00; // A = rds
                        alu_src_b_sel = 2'b00; // B = rs
                        reg_write = 1'b1;  // registers[rds] <= ALU result (wb_data)
                    end
                    4'b0001: begin //ADD
                        alu_func = 2'b11;
                        alu_src_a_sel = 2'b00;
                        alu_src_b_sel = 2'b00;
                        reg_write = 1'b1;
                    end
                    4'b0100: begin //SUB
                        alu_func = 2'b10;
                        alu_src_a_sel = 2'b00;
                        alu_src_b_sel = 2'b00;
                        reg_write = 1'b1;
                    end
                    4'b0101: begin //MULT
                        alu_func = 2'b01;
                        alu_src_a_sel = 2'b00;
                        alu_src_b_sel = 2'b00;
                        reg_write = 1'b1;
                    end

                    // ADDI: rds = rds + imm
                    4'b0011: begin
                        alu_func = 2'b11; //ADD
                        alu_src_a_sel = 2'b00; //rds
                        alu_src_b_sel = 2'b01; //imm_ext
                        reg_write = 1'b1;
                    end

                    // LW: rds = mem[rs]
                    4'b0110: begin
                        mem_addr_is_pc = 1'b0; // mem_addr mux selects rs
                        reg_write = 1'b1; //write into rds
                        wb_sel_mem = 1'b1; // wb_data = mem_data_out
                    end

                    // SW: mem[rs] = rds
                    4'b0111: begin
                        mem_addr_is_pc = 1'b0; 
                        mem_write = 1'b1;
                    end

                    // BEQ: if (r0 == r1) PC = PC + offset
                    4'b1000: begin
                        alu_func = 2'b10; // SUB
                        alu_src_a_sel = 2'b01; //r0
                        alu_src_b_sel = 2'b11; //r1
                        if (alu_zero) begin
                            pc_add_offset = 1'b1; // PC <= PC + off_beq
                            pc_offset_is_jmp = 1'b0; 
                        end
                    end

                    // JMP: PC = PC + offset
                    4'b1001: begin
                        pc_add_offset = 1'b1; // PC <= PC + off_jmp
                        pc_offset_is_jmp = 1'b1; 
                    end

                    // ADDM: rds = rds + mem[rs]
                    // cycle 1: put rs on mem_addr bus so mem_data_out = mem[rs]
                    // add happens in next cycle
                    4'b0010: begin
                        mem_addr_is_pc = 1'b0; 
                    end

                    // HALT
                    4'b1111: begin
                        halted = 1'b1;
                    end

                    default: begin end
                endcase
            end

            // ADDM cycle 2
            ADDM: begin
                alu_func = 2'b11; // ADD
                alu_src_a_sel = 2'b00; // rds
                alu_src_b_sel = 2'b10; // mem_data_out (from mem[rs])
                reg_write = 1'b1; // registers[rds] <= ALU result
                wb_sel_mem = 1'b0;   //wb_data = alu_result
            end

            HALT: begin
                halted = 1'b1;
            end

            default: begin end
        endcase
    end

endmodule

// ============================================================================
// MODULE: CPU (Datapath Top Level)
// ============================================================================
module cpu (
    input logic clk,
    input logic reset
);

    // -- Storage --
    // Registers kept here so the testbench can access 'dut.registers'
    logic [7:0] registers [0:1];
    
    // -- Architectural Registers --
    logic [7:0] PC;
    logic [7:0] IR;

    // TODO: Define other necessary signals
    logic [3:0] opcode;
    logic       ds_idx, s_idx; // only 2 regs exist => 1-bit indices

    logic [7:0] imm_ext;
    logic [7:0] off_beq;
    logic [7:0] off_jmp;

    logic [7:0] mem_addr;
    logic       mem_write;
    logic [7:0] mem_data_out;
    logic [7:0] reg_r1; // rds
    logic [7:0] reg_r2; // rs

    logic       reg_write;
    logic       wb_sel_mem;
    logic [7:0] wb_data;
    logic [7:0] alu_in_a, alu_in_b;
    logic [1:0] alu_src_a_sel, alu_src_b_sel;
    logic [1:0] alu_func;
    logic [7:0] alu_result;
    logic       alu_zero;
    logic       ir_write, pc_inc, pc_add_offset, pc_offset_is_jmp, mem_addr_is_pc;
    logic       halted;

    // ------------------------------------------------------------------------
    // 1. Instruction Decoding
    // ------------------------------------------------------------------------
    assign opcode = IR[7:4];
    assign ds_idx = IR[3];
    assign s_idx  = IR[2];

    // assign imm_ext = // TODO
    // assign off_beq = // TODO
    // assign off_jmp = // TODO
    assign imm_ext = {5'b00000, IR[2:0]}; // zero-extend 3-bit imm to 8 bits
    assign off_beq = {{4{IR[3]}}, IR[3:0]}; //sign-extend from bit[3]
    assign off_jmp = {{4{IR[3]}}, IR[3:0]}; // sign-extend from bit[3]


    // ------------------------------------------------------------------------
    // 2. Memory Module Instantiation & Address Multiplexing
    // ------------------------------------------------------------------------
    // TODO: Define memory address logic
    always_comb begin
        mem_addr = 8'b0000_0000;

        if (mem_addr_is_pc)
            mem_addr = PC; //FETCH
        else
            mem_addr = reg_r2;
    end


    // Memory module instantiation
    memory_module mem_inst (
        .clk(clk),
        .addr(mem_addr),
        .write_en(mem_write),
        .data_in(reg_r1),       // Data to write (from rds register)
        .data_out(mem_data_out) // Data read from memory
    );

    // ------------------------------------------------------------------------
    // 3. Register File Access
    // ------------------------------------------------------------------------
    assign reg_r1 = registers[ds_idx]; // Read Port 1
    assign reg_r2 = registers[s_idx];  // Read Port 2

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            registers[0] <= 0;
            registers[1] <= 0;
        end else if (reg_write) begin
            registers[ds_idx] <= wb_data;
        end
    end

    // ------------------------------------------------------------------------
    // 4. ALU & Datapath Muxes
    // ------------------------------------------------------------------------
    
    // TODO: Define ALU input multiplexing logic
    
    // ALU input A
    always_comb begin
        case (alu_src_a_sel)
            2'b00: alu_in_a = reg_r1; // 00 -> rds (reg_r1)
            2'b01: alu_in_a = registers[0]; // 01 -> r0  (for beq compare)
            default: alu_in_a = reg_r1;
        endcase
    end

    // ALU input B
    always_comb begin
        case (alu_src_b_sel)
            2'b00: alu_in_b = reg_r2; // 00 -> rs (reg_r2)
            2'b01: alu_in_b = imm_ext; // 01 -> imm_ext (ADDI)
            2'b10: alu_in_b = mem_data_out; // 10 -> mem_data_out (ADDM cycle 2)
            2'b11: alu_in_b = registers[1]; // 11 -> r1 (for beq compare)
            default: alu_in_b = reg_r2;
        endcase
    end

    alu cpu_alu (
        .a(alu_in_a),
        .b(alu_in_b),
        .func(alu_func),
        .result(alu_result),
        .zero(alu_zero)
    );

    // ------------------------------------------------------------------------
    // 5. Control Unit Instance
    // ------------------------------------------------------------------------
    control_unit cu (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .alu_zero(alu_zero),
        .halted(halted),
        // TODO: Connect other control signals
        .mem_write(mem_write),
        .reg_write(reg_write),
        .ir_write(ir_write),
        .alu_func(alu_func),
        .alu_src_a_sel(alu_src_a_sel),
        .alu_src_b_sel(alu_src_b_sel),
        .pc_inc(pc_inc),
        .pc_add_offset(pc_add_offset),
        .pc_offset_is_jmp(pc_offset_is_jmp),
        .wb_sel_mem(wb_sel_mem),
        .mem_addr_is_pc(mem_addr_is_pc)
    );

    // TODO
    always_comb begin
        if (wb_sel_mem)
            wb_data = mem_data_out; //LW path
        else
            wb_data = alu_result; //ALU path
    end

    // PC/IR update logic
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            PC <= 8'b0000_0000;
            IR <= 8'b0000_0000;
        end else if (!halted) begin

            // During FETCH, control asserts ir_write so the CPU latches the
            // inst currently addressed by mem_addr (which is PC in FETCH)
            if (ir_write) IR <= mem_data_out;

            if (pc_inc) PC <= PC + 8'b0000_0001; // FETCH increments PC

            // BEQ/JMP applies a signed offset relative to current PC
            if (pc_add_offset) begin
                if (pc_offset_is_jmp)
                    PC <= PC + off_jmp;
                else
                    PC <= PC + off_beq;
            end
        end
    end

endmodule
