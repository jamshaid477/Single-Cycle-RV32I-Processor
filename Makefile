# Makefile for RISC-V Single-Cycle Processor

# Compiler and tools
RV_GCC = riscv32-unknown-elf-gcc
RV_OBJCOPY = riscv32-unknown-elf-objcopy
IVERILOG = iverilog
VVP = vvp
GTKWAVE = gtkwave

# Files
SRC = complete_rv32i.v
TB = riscv_core_tb.v
HEX = program.hex
C_SRC = program.c
ASM = program.s
ELF = program.elf

default: compile run

# Compile Verilog files
compile:
	$(IVERILOG) -o processor.out $(SRC) $(TB)

# Run simulation
run:
	$(VVP) processor.out
...
    "README.md": 
