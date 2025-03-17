module riscv_core_tb;
    reg clk;
    reg rst;
    integer i;
    reg simulation_done;

    // Instantiate the RISC-V core
    riscv_core dut (
        .clk(clk),
        .rst(rst)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Test stimulus
    initial begin
        // Initialize waveform dump
        $dumpfile("riscv_core_tb.vcd");
        $dumpvars(0, riscv_core_tb);

        // Initialize signals
        simulation_done = 0;
        rst = 1;
        #20;
        rst = 0;

        // Monitor important signals
        $monitor("Time=%0t pc=%0h inst=%0h rd=%0d rs1=%0d rs2=%0d",
                 $time, dut.pc, dut.instruction, 
                 dut.instruction[11:7],  // rd
                 dut.instruction[19:15], // rs1
                 dut.instruction[24:20]  // rs2
                );

        // Let the processor run for some cycles
        repeat(20) @(posedge clk);

        // Check register values
        if (dut.reg_file.registers[2] != 5) begin
            $display("ERROR: x2 should be 5, got %0d", dut.reg_file.registers[2]);
        end else begin
            $display("PASS: x2 = 5");
        end

        if (dut.reg_file.registers[3] != 3) begin
            $display("ERROR: x3 should be 3, got %0d", dut.reg_file.registers[3]);
        end else begin
            $display("PASS: x3 = 3");
        end

        // Add more test cases for branch instructions
        // Wait for branch instructions to execute
        repeat(10) @(posedge clk);

        // Check if branches were taken correctly
        $display("\nBranch Test Results:");
        $display("PC=%h", dut.pc);
        $display("Branch=%b", dut.branch);
        $display("Is Jump=%b", dut.is_jump);

        // End simulation
        #100;
        simulation_done = 1;
        $finish;
    end

    // Timeout watchdog
    initial begin
        #10000; // Adjust timeout value as needed
        if (!simulation_done) begin
            $display("ERROR: Simulation timeout!");
            $finish;
        end
    end

    // Debug: Monitor branch-related signals
    always @(posedge clk) begin
        if (dut.branch || dut.is_jump) begin
            $display("Time=%0t BRANCH/JUMP: pc=%h inst=%h",
                     $time, dut.pc, dut.instruction);
            $display("  opcode=%b funct3=%b", 
                     dut.instruction[6:0],   // opcode
                     dut.instruction[14:12]  // funct3
                    );
        end
    end

endmodule
