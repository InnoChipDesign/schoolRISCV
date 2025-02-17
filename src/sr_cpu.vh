/*
 * schoolRISCV - small RISC-V CPU 
 *
 * originally based on Sarah L. Harris MIPS CPU 
 *                   & schoolMIPS project
 * 
 * Copyright(c) 2017-2020 Stanislav Zhelnio 
 *                        Aleksandr Romanov 
 */ 

//ALU commands
`define ALU_ADD     3'd0
`define ALU_OR      3'd1
`define ALU_SRL     3'd2
`define ALU_SLTU    3'd3
`define ALU_SUB     3'd4
`define ALU_AND     3'd5
`define ALU_SLL     3'd6

// instruction opcode
`define RVOP_ADDI   7'b0010011
`define RVOP_BEQ    7'b1100011
`define RVOP_LUI    7'b0110111
`define RVOP_BNE    7'b1100011
`define RVOP_ADD    7'b0110011
`define RVOP_OR     7'b0110011
`define RVOP_AND    7'b0110011
`define RVOP_SRL    7'b0110011
`define RVOP_SLL    7'b0110011
`define RVOP_SLTU   7'b0110011
`define RVOP_SUB    7'b0110011
`define RVOP_LOAD   7'b0000011
`define RVOP_STORE  7'b0100011
`define RVOP_JALR   7'b1100111

// instruction funct3
`define RVF3_ADDI   3'b000
`define RVF3_BEQ    3'b000
`define RVF3_BNE    3'b001
`define RVF3_ADD    3'b000
`define RVF3_OR     3'b110
`define RVF3_AND    3'b111
`define RVF3_SRL    3'b101
`define RVF3_SLL    3'b001
`define RVF3_SLTU   3'b011
`define RVF3_SUB    3'b000
`define RVF3_WORD   3'b010 // load/store word
`define RVF3_JALR   3'b000
`define RVF3_ANY    3'b???

// instruction funct7
`define RVF7_ADD    7'b0000000
`define RVF7_OR     7'b0000000
`define RVF7_AND    7'b0000000
`define RVF7_SRL    7'b0000000
`define RVF7_SLL    7'b0000000
`define RVF7_SLTU   7'b0000000
`define RVF7_SUB    7'b0100000
`define RVF7_ANY    7'b???????

