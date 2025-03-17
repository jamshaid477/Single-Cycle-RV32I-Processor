module riscv_core(
    input clk,
    input rst
);

    // Program Counter
    reg [31:0] pc;
    wire [31:0] next_pc;
    wire [31:0] pc_plus_4;
    wire [31:0] branch_target;
    
    // Instruction Memory
    wire [31:0] instruction;
    
    // Register File
    wire [4:0] rs1;
    wire [4:0] rs2;
    wire [4:0] rd;
    wire [31:0] reg_write_data;
    wire [31:0] reg_read_data1;
    wire [31:0] reg_read_data2;
    
    // ALU
    wire [31:0] alu_operand1;
    wire [31:0] alu_operand2;
    wire [31:0] alu_result;
    wire branch_condition;
    
    // Control Unit
    wire [3:0] alu_control;
    wire mem_read;
    wire mem_to_reg;
    wire mem_write;
    wire alu_src;
    wire reg_write;
    wire branch;
    wire is_jump;
    wire branch_taken;
    
    // Immediate Generator
    wire [31:0] immediate;
    
    // Data Memory
    wire [31:0] mem_read_data;
    
    // Instruction fields
    assign rs1 = instruction[19:15];
    assign rs2 = instruction[24:20];
    assign rd = instruction[11:7];
    
    // Program Counter Logic
    assign pc_plus_4 = pc + 4;
    assign branch_target = pc + immediate;
    assign next_pc = branch_taken ? branch_target : pc_plus_4;
    
    always @(posedge clk or posedge rst) begin
        if (rst)
            pc <= 32'h0;
        else
            pc <= next_pc;
    end
    
    // Instantiate Instruction Memory
    instruction_memory imem (
        .pc(pc),
        .instruction(instruction)
    );
    
    // Instantiate Register File
    register_file reg_file (
        .clk(clk),
        .rst(rst),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .write_data(reg_write_data),
        .reg_write(reg_write),
        .read_data1(reg_read_data1),
        .read_data2(reg_read_data2)
    );
    
    // Instantiate Control Unit
    control_unit ctrl (
        .opcode(instruction[6:0]),
        .funct3(instruction[14:12]),
        .funct7(instruction[31:25]),
        .branch_condition(branch_condition),
        .alu_control(alu_control),
        .mem_read(mem_read),
        .mem_to_reg(mem_to_reg),
        .mem_write(mem_write),
        .alu_src(alu_src),
        .reg_write(reg_write),
        .branch(branch),
        .is_jump(is_jump),
        .branch_taken(branch_taken)
    );
    
    // Instantiate Immediate Generator
    immediate_gen imm_gen (
        .instruction(instruction),
        .imm_out(immediate)
    );
    
    // ALU Input Muxing
    assign alu_operand1 = reg_read_data1;
    assign alu_operand2 = alu_src ? immediate : reg_read_data2;
    
    // Instantiate ALU
    alu alu_unit (
        .operand1(alu_operand1),
        .operand2(alu_operand2),
        .alu_control(alu_control),
        .result(alu_result),
        .branch_condition(branch_condition)
    );
    
    // Instantiate Data Memory
    data_memory dmem (
        .clk(clk),
        .address(alu_result),
        .write_data(reg_read_data2),
        .write_enable(mem_write),
        .read_enable(mem_read),
        .read_data(mem_read_data)
    );
    
    // Write Back Muxing
    assign reg_write_data = is_jump ? pc_plus_4 :
                           mem_to_reg ? mem_read_data : 
                           alu_result;
    
    // Debug
    always @(posedge clk) begin
        $display("PC: %h, Instruction: %h", pc, instruction);
        if (reg_write && rd != 0)
            $display("Register Write: x%d = %h", rd, reg_write_data);
        if (branch_taken)
            $display("Branch Taken: PC = %h -> %h", pc, branch_target);
    end

endmodule
module instruction_memory(
    input [31:0] pc,
    output reg [31:0] instruction
);

    // Internal memory
    reg [31:0] mem [0:63];  // 64 words of memory
    
    // Initialize with test program
    initial begin
        // Basic arithmetic and logic test sequence
        mem[0]  = 32'h00500113;    // ADDI x2, x0, 5      # x2 = 5
        mem[1]  = 32'h00300193;    // ADDI x3, x0, 3      # x3 = 3
        mem[2]  = 32'h00318233;    // ADD  x4, x3, x3     # x4 = x3 + x3 = 6
        mem[3]  = 32'h402182B3;    // SUB  x5, x3, x2     # x5 = x3 - x2 = -2
        
        // Test branch instructions
        mem[4]  = 32'h00320463;    // BEQ  x4, x3, 8      # Skip next if x4 == x3
        mem[5]  = 32'h00200113;    // ADDI x2, x0, 2      # x2 = 2 (should execute)
        mem[6]  = 32'h00418A63;    // BEQ  x3, x4, 20     # Branch if x3 == x4
        mem[7]  = 32'h00100113;    // ADDI x2, x0, 1      # x2 = 1 (should execute)
        mem[8]  = 32'hFE320AE3;    // BEQ  x4, x3, -12    # Branch back if x4 == x3
        
        // Test JAL and JALR
        mem[9]  = 32'h00C000EF;    // JAL  x1, 12         # Jump to PC+12, save PC+4 in x1
        mem[10] = 32'h00100113;    // ADDI x2, x0, 1      # (should be skipped)
        mem[11] = 32'h00200113;    // ADDI x2, x0, 2      # (should be skipped)
        mem[12] = 32'h00300113;    // ADDI x2, x0, 3      # x2 = 3
        mem[13] = 32'h000080E7;    // JALR x1, x1, 0      # Return to saved address
        
        // Test more branch conditions
        mem[14] = 32'h0041C463;    // BLT  x3, x4, 8      # Branch if x3 < x4
        mem[15] = 32'h00500113;    // ADDI x2, x0, 5      # (should be skipped)
        mem[16] = 32'h00600113;    // ADDI x2, x0, 6      # x2 = 6
        
        // End program with infinite loop
        mem[17] = 32'h0000006F;    // JAL  x0, 0          # Infinite loop
        
        $display("Instruction memory initialized with test program");
    end
    
    // Read instruction
    always @(*) begin
        instruction = mem[pc[31:2]];  // Word aligned
        $display("Fetch: pc=%h inst=%h", pc, instruction);
    end

endmodule

module register_file(
    input clk,
    input rst,
    input [4:0] rs1,
    input [4:0] rs2,
    input [4:0] rd,
    input [31:0] write_data,
    input reg_write,
    output reg [31:0] read_data1,
    output reg [31:0] read_data2
);

    // Register file
    reg [31:0] registers [0:31];
    integer i;

    // Initialize registers
    initial begin
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 32'b0;
    end

    // Read ports are combinational
    always @(*) begin
        // x0 is hardwired to 0
        read_data1 = (rs1 == 0) ? 32'b0 : registers[rs1];
        read_data2 = (rs2 == 0) ? 32'b0 : registers[rs2];
    end

    // Write port is synchronous
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 32'b0;
        end
        else if (reg_write && rd != 0) begin // x0 cannot be written
            registers[rd] <= write_data;
            $display("Register Write: x%0d = %0h", rd, write_data);
        end
    end

endmodule

module control_unit(
    input [6:0] opcode,
    input [2:0] funct3,
    input [6:0] funct7,
    input branch_condition,
    output reg [3:0] alu_control,
    output reg mem_read,
    output reg mem_to_reg,
    output reg mem_write,
    output reg alu_src,
    output reg reg_write,
    output reg branch,
    output reg is_jump,
    output reg branch_taken
);

    // Debug signals
    integer debug_cycle = 0;
    initial debug_cycle = 0;
    always @(*) debug_cycle = debug_cycle + 1;

    // Opcode definitions
    parameter R_TYPE    = 7'b0110011;
    parameter I_TYPE    = 7'b0010011;
    parameter STORE     = 7'b0100011;
    parameter LOAD      = 7'b0000011;
    parameter BRANCH    = 7'b1100011;
    parameter JAL       = 7'b1101111;
    parameter JALR      = 7'b1100111;
    parameter LUI      = 7'b0110111;
    parameter AUIPC    = 7'b0010111;

    // ALU control codes
    parameter ALU_ADD  = 4'b0000;
    parameter ALU_SUB  = 4'b0001;
    parameter ALU_AND  = 4'b0010;
    parameter ALU_OR   = 4'b0011;
    parameter ALU_XOR  = 4'b0100;
    parameter ALU_SLL  = 4'b0101;
    parameter ALU_SRL  = 4'b0110;
    parameter ALU_SRA  = 4'b0111;
    parameter ALU_SLT  = 4'b1000;
    parameter ALU_SLTU = 4'b1001;
    parameter ALU_BEQ  = 4'b1010;
    parameter ALU_BNE  = 4'b1011;
    parameter ALU_BLT  = 4'b1100;
    parameter ALU_BGE  = 4'b1101;
    parameter ALU_BLTU = 4'b1110;
    parameter ALU_BGEU = 4'b1111;

    always @(*) begin
        // Default values
        mem_read = 0;
        mem_to_reg = 0;
        mem_write = 0;
        alu_src = 0;
        reg_write = 0;
        branch = 0;
        is_jump = 0;
        branch_taken = 0;
        alu_control = ALU_ADD;

        case(opcode)
            R_TYPE: begin
                reg_write = 1;
                case(funct3)
                    3'b000: alu_control = (funct7[5]) ? ALU_SUB : ALU_ADD;
                    3'b001: alu_control = ALU_SLL;
                    3'b010: alu_control = ALU_SLT;
                    3'b011: alu_control = ALU_SLTU;
                    3'b100: alu_control = ALU_XOR;
                    3'b101: alu_control = (funct7[5]) ? ALU_SRA : ALU_SRL;
                    3'b110: alu_control = ALU_OR;
                    3'b111: alu_control = ALU_AND;
                endcase
            end
            
            I_TYPE: begin
                reg_write = 1;
                alu_src = 1;
                case(funct3)
                    3'b000: alu_control = ALU_ADD;  // ADDI
                    3'b001: alu_control = ALU_SLL;  // SLLI
                    3'b010: alu_control = ALU_SLT;  // SLTI
                    3'b011: alu_control = ALU_SLTU; // SLTIU
                    3'b100: alu_control = ALU_XOR;  // XORI
                    3'b101: alu_control = (funct7[5]) ? ALU_SRA : ALU_SRL;
                    3'b110: alu_control = ALU_OR;   // ORI
                    3'b111: alu_control = ALU_AND;  // ANDI
                endcase
            end
            
            STORE: begin
                mem_write = 1;
                alu_src = 1;
                alu_control = ALU_ADD;
            end
            
            LOAD: begin
                mem_read = 1;
                mem_to_reg = 1;
                reg_write = 1;
                alu_src = 1;
                alu_control = ALU_ADD;
            end
            
            BRANCH: begin
                branch = 1;
                case(funct3)
                    3'b000: alu_control = ALU_BEQ;  // BEQ
                    3'b001: alu_control = ALU_BNE;  // BNE
                    3'b100: alu_control = ALU_BLT;  // BLT
                    3'b101: alu_control = ALU_BGE;  // BGE
                    3'b110: alu_control = ALU_BLTU; // BLTU
                    3'b111: alu_control = ALU_BGEU; // BGEU
                endcase
                branch_taken = branch_condition;
                $display("Debug: Branch ALU control: funct3=%b, control=%b", funct3, alu_control);
            end
            
            JAL: begin
                is_jump = 1;
                reg_write = 1;
                branch_taken = 1;
            end
            
            JALR: begin
                is_jump = 1;
                reg_write = 1;
                alu_src = 1;
                branch_taken = 1;
            end
            
            LUI: begin
                reg_write = 1;
                alu_src = 1;
                alu_control = ALU_ADD;  // Pass immediate directly
            end
            
            AUIPC: begin
                reg_write = 1;
                alu_src = 1;
                alu_control = ALU_ADD;  // Add immediate to PC
            end
            
            default: begin
                mem_read = 0;
                mem_to_reg = 0;
                mem_write = 0;
                alu_src = 0;
                reg_write = 0;
                branch = 0;
                is_jump = 0;
                branch_taken = 0;
                alu_control = ALU_ADD;
            end
        endcase
    end

endmodule

module immediate_gen(
    input [31:0] instruction,
    output reg [31:0] imm_out
);

    wire [6:0] opcode = instruction[6:0];
    wire [2:0] funct3 = instruction[14:12];
    
    always @(*) begin
        case(opcode)
            7'b0010011: // I-type
                imm_out = {{20{instruction[31]}}, instruction[31:20]};
                
            7'b0000011: // Load
                imm_out = {{20{instruction[31]}}, instruction[31:20]};
                
            7'b0100011: // Store
                imm_out = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
                
            7'b1100011: begin // Branch
                imm_out = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
                $display("Branch Imm Gen: inst=%h imm=%h", instruction, imm_out);
            end
            
            7'b1101111: begin // JAL
                imm_out = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
                $display("JAL Imm Gen: inst=%h imm=%h", instruction, imm_out);
            end
            
            7'b1100111: // JALR
                imm_out = {{20{instruction[31]}}, instruction[31:20]};
                
            7'b0110111: // LUI
                imm_out = {instruction[31:12], 12'b0};
                
            7'b0010111: // AUIPC
                imm_out = {instruction[31:12], 12'b0};
                
            default:
                imm_out = 32'b0;
        endcase
    end

endmodule

module alu(
    input [31:0] operand1,
    input [31:0] operand2,
    input [3:0] alu_control,
    output reg [31:0] result,
    output reg branch_condition
);

    // ALU Control codes
    parameter ALU_ADD  = 4'b0000;
    parameter ALU_SUB  = 4'b0001;
    parameter ALU_AND  = 4'b0010;
    parameter ALU_OR   = 4'b0011;
    parameter ALU_XOR  = 4'b0100;
    parameter ALU_SLL  = 4'b0101;
    parameter ALU_SRL  = 4'b0110;
    parameter ALU_SRA  = 4'b0111;
    parameter ALU_SLT  = 4'b1000;
    parameter ALU_SLTU = 4'b1001;
    parameter ALU_BEQ  = 4'b1010;
    parameter ALU_BNE  = 4'b1011;
    parameter ALU_BLT  = 4'b1100;
    parameter ALU_BGE  = 4'b1101;
    parameter ALU_BLTU = 4'b1110;
    parameter ALU_BGEU = 4'b1111;

    wire signed [31:0] signed_op1;
    wire signed [31:0] signed_op2;
    assign signed_op1 = operand1;
    assign signed_op2 = operand2;

    always @(*) begin
        // Default values
        result = 32'b0;
        branch_condition = 1'b0;

        case(alu_control)
            ALU_ADD:  result = operand1 + operand2;
            ALU_SUB:  result = operand1 - operand2;
            ALU_AND:  result = operand1 & operand2;
            ALU_OR:   result = operand1 | operand2;
            ALU_XOR:  result = operand1 ^ operand2;
            ALU_SLL:  result = operand1 << operand2[4:0];
            ALU_SRL:  result = operand1 >> operand2[4:0];
            ALU_SRA:  result = signed_op1 >>> operand2[4:0];
            ALU_SLT:  result = (signed_op1 < signed_op2) ? 32'b1 : 32'b0;
            ALU_SLTU: result = (operand1 < operand2) ? 32'b1 : 32'b0;
            
            // Branch operations
            ALU_BEQ:  branch_condition = (operand1 == operand2);
            ALU_BNE:  branch_condition = (operand1 != operand2);
            ALU_BLT:  branch_condition = (signed_op1 < signed_op2);
            ALU_BGE:  branch_condition = (signed_op1 >= signed_op2);
            ALU_BLTU: branch_condition = (operand1 < operand2);
            ALU_BGEU: branch_condition = (operand1 >= operand2);
            
            default: begin
                result = 32'b0;
                branch_condition = 1'b0;
            end
        endcase
    end

endmodule

module data_memory(
    input clk,
    input [31:0] address,
    input [31:0] write_data,
    input write_enable,
    input read_enable,
    output reg [31:0] read_data
);

    reg [31:0] memory [0:1023];  // 4KB memory
    
    // Initialize memory
    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1)
            memory[i] = 32'b0;
    end
    
    // Read operation
    always @(*) begin
        if (read_enable)
            read_data = memory[address[31:2]];  // Word aligned
        else
            read_data = 32'b0;
    end
    
    // Write operation
    always @(posedge clk) begin
        if (write_enable) begin
            memory[address[31:2]] <= write_data;
            $display("Memory Write: addr=%h data=%h", address, write_data);
        end
    end

endmodule


