// ============================================================================
// MODULE: Memory (Single-Port RAM)
// ============================================================================
module memory_module (
    input  logic       clk,
    input  logic [7:0] addr,
    input  logic       write_en,
    input  logic [7:0] data_in,
    output logic [7:0] data_out
);
    logic [7:0] mem [0:255];

    initial begin
        for (int i=0; i<256; i++) mem[i] = 0;
    end

    assign data_out = mem[addr];

    always_ff @(posedge clk) begin
        if (write_en)
            mem[addr] <= data_in;
    end
endmodule


// ============================================================================
// MODULE: ALU
// ============================================================================
module alu (
    input  logic [7:0] a,
    input  logic [7:0] b,
    input  logic [1:0] func,
    output logic [7:0] result,
    output logic       zero
);
    always_comb begin
        case (func)
            2'b00: result = ~(a & b);
            2'b01: result = a * b;
            2'b10: result = a - b;
            2'b11: result = a + b;
            default: result = 8'h00;
        endcase
    end

    assign zero = (result == 0);
endmodule


// ============================================================================
// MODULE: Control Unit (MULTICYCLE FSM)
// ============================================================================
module control_unit (
    input  logic       clk,
    input  logic       reset,
    input  logic [3:0] opcode,
    input  logic       alu_zero,

    output logic       halted,
    output logic       mem_write,
    output logic       reg_write,
    output logic       ir_write,
    output logic [1:0] alu_func,
    output logic [1:0] alu_src_a_sel,
    output logic [1:0] alu_src_b_sel,
    output logic       pc_inc,
    output logic       pc_add_offset,
    output logic       pc_offset_is_jmp,
    output logic       wb_sel_mem,
    output logic       mem_addr_is_pc
);

    typedef enum logic [1:0] {
        FETCH,
        EXEC,
        ADDM,
        HALT
    } state_t;

    state_t curr_state, next_state;

    // State transition
    always_comb begin
        next_state = curr_state;
        case (curr_state)
            FETCH: next_state = EXEC;
            EXEC: begin
                if (opcode == 4'b1111)      next_state = HALT;
                else if (opcode == 4'b0010) next_state = ADDM;
                else                        next_state = FETCH;
            end
            ADDM: next_state = FETCH;
            HALT: next_state = HALT;
        endcase
    end

    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            curr_state <= FETCH;
        else
            curr_state <= next_state;
    end

    // Control outputs
    always_comb begin
        halted = 0;
        mem_write = 0;
        reg_write = 0;
        ir_write = 0;
        pc_inc = 0;
        pc_add_offset = 0;
        pc_offset_is_jmp = 0;
        wb_sel_mem = 0;
        mem_addr_is_pc = 0;

        alu_func = 2'b11;
        alu_src_a_sel = 2'b00;
        alu_src_b_sel = 2'b00;

        case (curr_state)
            FETCH: begin
                mem_addr_is_pc = 1;
                ir_write = 1;
                pc_inc = 1;
            end

            EXEC: begin
                case (opcode)
                    4'b0000: begin alu_func=2'b00; reg_write=1; end
                    4'b0001: begin alu_func=2'b11; reg_write=1; end
                    4'b0100: begin alu_func=2'b10; reg_write=1; end
                    4'b0101: begin alu_func=2'b01; reg_write=1; end

                    4'b0011: begin
                        alu_func=2'b11;
                        alu_src_b_sel=2'b01;
                        reg_write=1;
                    end

                    4'b0110: begin
                        mem_addr_is_pc=0;
                        reg_write=1;
                        wb_sel_mem=1;
                    end

                    4'b0111: begin
                        mem_addr_is_pc=0;
                        mem_write=1;
                    end

                    4'b1000: begin
                        alu_func=2'b10;
                        if (alu_zero)
                            pc_add_offset=1;
                    end

                    4'b1001: begin
                        pc_add_offset=1;
                        pc_offset_is_jmp=1;
                    end

                    4'b1111: halted=1;
                endcase
            end

            ADDM: begin
                alu_func=2'b11;
                alu_src_b_sel=2'b10;
                reg_write=1;
            end

            HALT: halted=1;
        endcase
    end
endmodule


// ============================================================================
// MODULE: CPU
// ============================================================================
module cpu (
    input logic clk,
    input logic reset
);

    logic [7:0] registers [0:1];
    logic [7:0] PC, IR;

    logic [3:0] opcode;
    logic ds_idx, s_idx;
    assign opcode = IR[7:4];
    assign ds_idx = IR[3];
    assign s_idx  = IR[2];

    logic [7:0] imm_ext;
    logic signed [7:0] off_ext;
    assign imm_ext = {5'b0, IR[2:0]};
    assign off_ext = {{4{IR[3]}}, IR[3:0]};

    logic [7:0] reg_r1, reg_r2;
    assign reg_r1 = registers[ds_idx];
    assign reg_r2 = registers[s_idx];

    logic [7:0] mem_addr, mem_data_out;
    logic mem_write;

    memory_module mem_inst (
        .clk(clk),
        .addr(mem_addr),
        .write_en(mem_write),
        .data_in(reg_r1),
        .data_out(mem_data_out)
    );

    logic [7:0] alu_in_a, alu_in_b, alu_result;
    logic [1:0] alu_func;
    logic alu_zero;

    alu cpu_alu (
        .a(alu_in_a),
        .b(alu_in_b),
        .func(alu_func),
        .result(alu_result),
        .zero(alu_zero)
    );

    logic halted, reg_write, wb_sel_mem, ir_write;
    logic pc_inc, pc_add_offset, pc_offset_is_jmp, mem_addr_is_pc;
    logic [1:0] alu_src_a_sel, alu_src_b_sel;

    control_unit cu (
        .clk(clk), .reset(reset), .opcode(opcode), .alu_zero(alu_zero),
        .halted(halted), .mem_write(mem_write), .reg_write(reg_write),
        .ir_write(ir_write), .alu_func(alu_func),
        .alu_src_a_sel(alu_src_a_sel), .alu_src_b_sel(alu_src_b_sel),
        .pc_inc(pc_inc), .pc_add_offset(pc_add_offset),
        .pc_offset_is_jmp(pc_offset_is_jmp),
        .wb_sel_mem(wb_sel_mem), .mem_addr_is_pc(mem_addr_is_pc)
    );

    always_comb begin
        mem_addr = mem_addr_is_pc ? PC : reg_r2;
        alu_in_a = reg_r1;
        case (alu_src_b_sel)
            2'b01: alu_in_b = imm_ext;
            2'b10: alu_in_b = mem_data_out;
            default: alu_in_b = reg_r2;
        endcase
    end

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            PC<=0; IR<=0; registers[0]<=0; registers[1]<=0;
        end else if (!halted) begin
            if (ir_write) IR <= mem_data_out;
            if (reg_write)
                registers[ds_idx] <= wb_sel_mem ? mem_data_out : alu_result;

            if (pc_add_offset)
                PC <= PC + off_ext;
            else if (pc_inc)
                PC <= PC + 1;
        end
    end
endmodule
