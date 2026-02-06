module cpu_tb;

    logic clk;
    logic reset;

    cpu dut (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation: 10ns period
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // Waves
    initial begin
        $dumpfile("cpu_extra.vcd");
        $dumpvars(0, cpu_tb);
    end

    // ------------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------------

    // A-type: Opcode[7:4] | ds[3] | s[2] | extra[1:0]
    function automatic [7:0] encA(input [3:0] op, input bit ds, input bit s, input [1:0] extra);
        encA = {op, ds, s, extra};
    endfunction

    // B-type: Opcode[7:4] | ds[3] | imm[2:0]
    function automatic [7:0] encB(input [3:0] op, input bit ds, input [2:0] imm);
        encB = {op, ds, imm};
    endfunction

    // C-type: Opcode[7:4] | offset[3:0]
    function automatic [7:0] encC(input [3:0] op, input [3:0] off);
        encC = {op, off};
    endfunction

    task automatic clear_mem;
        int i;
        begin
            for (i = 0; i < 256; i++) dut.mem_inst.mem[i] = 8'b0000_0000;
        end
    endtask

    task automatic reset_cpu;
        begin
            reset = 1'b1;
            #12;          // straddle a clock edge
            reset = 1'b0;
            #8;           // let signals settle
        end
    endtask

    task automatic load_instr(input int addr, input [7:0] inst);
        begin
            dut.mem_inst.mem[addr] = inst;
        end
    endtask

    // Run until halt with per-test timeout (NO fork/join: avoids Icarus vvp crash)
    task automatic run_until_halt(input int max_cycles);
        int cycles;
        begin
            cycles = 0;

            while (!dut.halted && cycles < max_cycles) begin
                @(posedge clk);
                cycles++;
            end

            if (!dut.halted) begin
                $display("\nERROR: timeout waiting for HALT after %0d cycles (PC=%0d IR=%b r0=%0d r1=%0d)",
                        max_cycles, dut.PC, dut.IR, dut.registers[0], dut.registers[1]);
                $finish;
            end

            // settle one more cycle
            @(posedge clk);
        end
    endtask


    task automatic check_reg(input string name, input int idx, input [7:0] exp);
        begin
            if (dut.registers[idx] !== exp) begin
                $display("FAIL %s: r%0d=%0d (0x%02h) expected %0d (0x%02h)",
                         name, idx, dut.registers[idx], dut.registers[idx], exp, exp);
                $finish;
            end
        end
    endtask

    task automatic check_mem(input string name, input int addr, input [7:0] exp);
        begin
            if (dut.mem_inst.mem[addr] !== exp) begin
                $display("FAIL %s: mem[%0d]=%0d (0x%02h) expected %0d (0x%02h)",
                         name, addr, dut.mem_inst.mem[addr], dut.mem_inst.mem[addr], exp, exp);
                $finish;
            end
        end
    endtask

    // Optional monitor (comment out if too chatty)
    initial begin
        $monitor("t=%0t PC=%0d IR=%b halted=%b r0=%0d r1=%0d",
                 $time, dut.PC, dut.IR, dut.halted, dut.registers[0], dut.registers[1]);
    end

    // ------------------------------------------------------------------------
    // Tests
    // ------------------------------------------------------------------------
    initial begin
        $display("Starting extended CPU tests...");

        // ================================================================
        // TEST 1: Provided program (baseline)
        // ================================================================
        clear_mem();

        // Load program + initial data first
        load_instr(0, 8'b00110101);
        load_instr(1, 8'b00111111);
        load_instr(2, 8'b00010100);
        load_instr(3, 8'b01101000);
        load_instr(4, 8'b10000001);
        load_instr(5, 8'b11110000);
        load_instr(6, 8'b00100000);
        load_instr(7, 8'b00111111);
        load_instr(8, 8'b01000100);
        load_instr(9, 8'b01110100);
        load_instr(10,8'b11110000);

        // Data
        dut.mem_inst.mem[12] = 8'd12;

        // Now start CPU
        reset_cpu();

        run_until_halt(200);
        check_reg("TEST1", 0, 8'd5);
        check_reg("TEST1", 1, 8'd19);
        check_mem("TEST1", 19, 8'd5);
        $display("PASS TEST1\n");


        // ================================================================
        // TEST 2: BEQ NOT taken (should NOT skip)
        // ================================================================
        clear_mem();
        reset_cpu();

        // addi r0,1
        // addi r1,2
        // beq +1      (not taken)
        // addi r0,1   (executes => r0 becomes 2)
        // halt
        load_instr(0, encB(4'b0011, 1'b0, 3'b001)); // addi r0,1
        load_instr(1, encB(4'b0011, 1'b1, 3'b010)); // addi r1,2
        load_instr(2, encC(4'b1000, 4'b0001));      // beq +1
        load_instr(3, encB(4'b0011, 1'b0, 3'b001)); // addi r0,1
        load_instr(4, 8'b11110000);                 // halt

        run_until_halt(200);
        check_reg("TEST2", 0, 8'd2);
        check_reg("TEST2", 1, 8'd2);
        $display("PASS TEST2\n");

        // ================================================================
        // TEST 3: BEQ taken with NEGATIVE offset (sign-extension test)
        // Pattern: JMP skips over HALT, then BEQ branches backward to HALT.
        // ================================================================
        clear_mem();
        reset_cpu();

        // addi r0,0
        // addi r1,0
        // jmp +1      -> goes to beq
        // halt        -> target of negative beq
        // beq -2      -> from PC=4 branches back to PC=3 (halt)
        load_instr(0, encB(4'b0011, 1'b0, 3'b000)); // addi r0,0
        load_instr(1, encB(4'b0011, 1'b1, 3'b000)); // addi r1,0
        load_instr(2, encC(4'b1001, 4'b0001));      // jmp +1
        load_instr(3, 8'b11110000);                 // halt
        load_instr(4, encC(4'b1000, 4'b1110));      // beq -2 (0b1110 = -2)

        run_until_halt(200);
        check_reg("TEST3", 0, 8'd0);
        check_reg("TEST3", 1, 8'd0);
        $display("PASS TEST3\n");

        // ================================================================
        // TEST 4: JMP forward skips an instruction
        // ================================================================
        clear_mem();
        reset_cpu();

        // addi r0,1
        // jmp +1      -> skip next addi
        // addi r0,7   (skipped)
        // halt
        load_instr(0, encB(4'b0011, 1'b0, 3'b001)); // addi r0,1
        load_instr(1, encC(4'b1001, 4'b0001));      // jmp +1
        load_instr(2, encB(4'b0011, 1'b0, 3'b111)); // addi r0,7 (skipped)
        load_instr(3, 8'b11110000);                 // halt

        run_until_halt(200);
        check_reg("TEST4", 0, 8'd1);
        $display("PASS TEST4\n");

        // ================================================================
        // TEST 5: NAND + MULT + SUB/ADD sanity
        // ================================================================
        clear_mem();
        reset_cpu();

        // r0=3, r1=5
        // nand r0,r1 => r0 = ~(3&5) = ~(1) = 254
        // mult r1,r0 => r1 = 5*254 = 1270 mod256 = 246
        // sub  r1,r0 => r1 = 246 - 254 = -8 mod256 = 248
        // add  r0,r1 => r0 = 254 + 248 = 502 mod256 = 246
        // halt
        load_instr(0, encB(4'b0011, 1'b0, 3'b011)); // addi r0,3
        load_instr(1, encB(4'b0011, 1'b1, 3'b101)); // addi r1,5
        load_instr(2, encA(4'b0000, 1'b0, 1'b1, 2'b00)); // nand r0,r1
        load_instr(3, encA(4'b0101, 1'b1, 1'b0, 2'b00)); // mult r1,r0
        load_instr(4, encA(4'b0100, 1'b1, 1'b0, 2'b00)); // sub  r1,r0
        load_instr(5, encA(4'b0001, 1'b0, 1'b1, 2'b00)); // add  r0,r1
        load_instr(6, 8'b11110000); // halt

        run_until_halt(200);
        check_reg("TEST5", 0, 8'd246);
        check_reg("TEST5", 1, 8'd248);
        $display("PASS TEST5\n");

        // ================================================================
        // TEST 6: SW then LW (store/load roundtrip)
        // ================================================================
        clear_mem();
        reset_cpu();

        // r0=6
        // r1=10
        // sw r0,r1 -> mem[10]=6
        // lw r0,r1 -> r0=mem[10]=6
        // halt
        load_instr(0, encB(4'b0011, 1'b0, 3'b110)); // addi r0,6
        load_instr(1, encB(4'b0011, 1'b1, 3'b010)); // addi r1,2
        load_instr(2, encB(4'b0011, 1'b1, 3'b111)); // addi r1,7 => r1=9
        load_instr(3, encB(4'b0011, 1'b1, 3'b001)); // addi r1,1 => r1=10
        load_instr(4, encA(4'b0111, 1'b0, 1'b1, 2'b00)); // sw r0,r1
        load_instr(5, encA(4'b0110, 1'b0, 1'b1, 2'b00)); // lw r0,r1
        load_instr(6, 8'b11110000); // halt

        run_until_halt(200);
        check_reg("TEST6", 0, 8'd6);
        check_mem("TEST6", 10, 8'd6);
        $display("PASS TEST6\n");

        // ================================================================
        // TEST 7: ADDM uses mem[rs] and requires 2 cycles
        // ================================================================
        clear_mem();
        reset_cpu();

        // r0=4
        // mem[4]=20
        // addm r0,r0 => r0 = 4 + 20 = 24
        // halt
        load_instr(0, encB(4'b0011, 1'b0, 3'b100)); // addi r0,4
        load_instr(1, encA(4'b0010, 1'b0, 1'b0, 2'b00)); // addm r0,r0
        load_instr(2, 8'b11110000); // halt

        dut.mem_inst.mem[4] = 8'd20;

        run_until_halt(200);
        check_reg("TEST7", 0, 8'd24);
        $display("PASS TEST7\n");

        $display("ALL EXTENDED TESTS PASSED ✅");


        // ================================================================
        // TEST 8: 8-bit arithmetic wraparound
        // ================================================================
        clear_mem();
        reset_cpu();

        // r0 = 7
        // r1 = 7
        // mult r0,r1 => 49
        // add r0,r0 => 98
        // add r0,r0 => 196
        // add r0,r0 => 392 mod256 = 136
        // sub r0,r1 => 136 - 7 = 129
        // halt
        load_instr(0, encB(4'b0011, 1'b0, 3'b111)); // addi r0,7
        load_instr(1, encB(4'b0011, 1'b1, 3'b111)); // addi r1,7
        load_instr(2, encA(4'b0101, 1'b0, 1'b1, 2'b00)); // mult r0,r1
        load_instr(3, encA(4'b0001, 1'b0, 1'b0, 2'b00)); // add r0,r0
        load_instr(4, encA(4'b0001, 1'b0, 1'b0, 2'b00)); // add r0,r0
        load_instr(5, encA(4'b0001, 1'b0, 1'b0, 2'b00)); // add r0,r0
        load_instr(6, encA(4'b0100, 1'b0, 1'b1, 2'b00)); // sub r0,r1
        load_instr(7, 8'b11110000); // halt

        run_until_halt(200);
        check_reg("TEST8", 0, 8'd129);
        $display("PASS TEST8\n");

        // ================================================================
        // TEST 9: memory boundary at address 255
        // ================================================================
        clear_mem();
        reset_cpu();

        // r0 = 3
        // r1 = 7 + 7 + 7 + 7 + 7 + 7 + 7 + 7 = 56
        // keep adding until r1=255
        // sw r0,r1
        // lw r0,r1
        // halt

        load_instr(0, encB(4'b0011, 1'b0, 3'b011)); // addi r0,3
        load_instr(1, encB(4'b0011, 1'b1, 3'b111)); // addi r1,7

        // build r1 toward 255
        load_instr(2, encA(4'b0001, 1'b1, 1'b1, 2'b00)); // add r1,r1 => 14
        load_instr(3, encA(4'b0001, 1'b1, 1'b1, 2'b00)); // 28
        load_instr(4, encA(4'b0001, 1'b1, 1'b1, 2'b00)); // 56
        load_instr(5, encA(4'b0001, 1'b1, 1'b1, 2'b00)); // 112
        load_instr(6, encA(4'b0001, 1'b1, 1'b1, 2'b00)); // 224
        load_instr(7, encB(4'b0011, 1'b1, 3'b111));      // +7 => 231
        load_instr(8, encB(4'b0011, 1'b1, 3'b111));      // +7 => 238
        load_instr(9, encB(4'b0011, 1'b1, 3'b111));      // +7 => 245
        load_instr(10,encB(4'b0011, 1'b1, 3'b111));      // +7 => 252
        load_instr(11,encB(4'b0011, 1'b1, 3'b011));      // +3 => 255

        load_instr(12,encA(4'b0111, 1'b0, 1'b1, 2'b00)); // sw r0,r1
        load_instr(13,encA(4'b0110, 1'b0, 1'b1, 2'b00)); // lw r0,r1
        load_instr(14,8'b11110000); // halt

        run_until_halt(200);
        check_mem("TEST9", 255, 8'd3);
        check_reg("TEST9", 0, 8'd3);
        $display("PASS TEST9\n");


        // ================================================================
        // ================================================================
        // TEST 10: negative BEQ offset (terminating)
        // ================================================================
        clear_mem();

        // Program:
        // 0: addi r0,1          ; r0 = 1
        // 1: addi r1,0          ; r1 = 0
        // 2: addi r1,1          ; r1 = 1
        // 3: beq  -1            ; if r0==r1, jump back to PC=3? (bad: infinite)
        // Better:
        // Use BEQ -2 to jump back to "addi r1,1" exactly once, then mismatch r0
        //
        // 0: addi r0,1          ; r0=1
        // 1: addi r1,1          ; r1=1
        // 2: beq  -1            ; infinite (bad)
        //
        // Let's do this instead:
        // 0: addi r0,1          ; r0=1
        // 1: addi r1,1          ; r1=1
        // 2: addi r0,1          ; r0=2   (make mismatch)
        // 3: beq  -2            ; compare r0 vs r1 (2 vs 1) => NOT taken, fall through
        // 4: halt
        load_instr(0, encB(4'b0011, 1'b0, 3'b001)); // addi r0,1  => 1
        load_instr(1, encB(4'b0011, 1'b1, 3'b001)); // addi r1,1  => 1
        load_instr(2, encB(4'b0011, 1'b0, 3'b001)); // addi r0,1  => 2
        load_instr(3, encC(4'b1000, 4'b1110));      // beq -2 (to PC=2) but should NOT be taken
        load_instr(4, 8'b11110000);                 // halt

        reset_cpu();

        run_until_halt(200);
        check_reg("TEST10", 0, 8'd2);
        check_reg("TEST10", 1, 8'd1);
        $display("PASS TEST10\n");




        $finish;
    end



endmodule
