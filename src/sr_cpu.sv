/*
 * schoolRISCV - small RISC-V CPU 
 *
 * originally based on Sarah L. Harris MIPS CPU 
 *                   & schoolMIPS project
 * 
 * Copyright(c) 2017-2020 Stanislav Zhelnio 
 *                        Aleksandr Romanov 
 */ 

`include "sr_cpu.vh"

module sr_cpu
(
    input           clk,        // clock
    input           rst_n,      // reset
    input   [ 4:0]  regAddr,    // debug access reg address
    output  [31:0]  regData,    // debug access reg data
    output  [31:0]  imAddr,     // instruction memory address
    input   [31:0]  imData,     // instruction memory data
    output  [31:0]  memAddr,    // general memory address
    output          memWriteEnable, // do write in memory
    inout   [31:0]  memData     // data from general memory
);
    // program counter
    wire [31:0] pc;
    wire [31:0] pcNextDefault  = pc + 4;
    sm_register r_pc(clk, rst_n, pcNextDefault, pc);

    // program memory access
    assign imAddr = pc >> 2;

    // #####################################
    // ######## INSTURCTION DECODE #########
    // #####################################

    // ID stage registers
    logic [ 4:0] rd_D;
    logic [ 4:0] rs1_D;
    logic [ 4:0] rs2_D;
    logic [31:0] immI_D;
    logic [31:0] immS_D;
    logic [31:0] immB_D;
    logic [31:0] immU_D;
    logic        branch_D;
    logic        condZero_D;
    logic        regWrite_D;
    logic        memWrite_D;
    logic        aluSrc_D;
    logic [1:0]  wdSrc_D;
    logic [2:0]  aluControl_D;
    logic        vld_D;
    logic [31:0] pc_D;

    // decoded values
    wire [ 6:0] cmdOp;
    wire [ 2:0] cmdF3;
    wire [ 6:0] cmdF7;
    wire [ 4:0] rd;
    wire [ 4:0] rs1;
    wire [ 4:0] rs2;
    wire [31:0] immI;
    wire [31:0] immS;
    wire [31:0] immB;
    wire [31:0] immU;

    instruction_decode id(
        .instr      ( imData ),
        .cmdOp      ( cmdOp  ), .cmdF3      ( cmdF3  ), .cmdF7      ( cmdF7  ),
        .rd         ( rd     ), .rs1        ( rs1    ), .rs2        ( rs2    ),
        .immI       ( immI   ), .immS       ( immS   ), .immB       ( immB   ),
        .immU       ( immU   ) 
    );

    // control values
    wire       branch;
    wire       condZero;
    wire       regWrite; 
    wire       memWrite;
    wire       aluSrc;
    wire [1:0] wdSrc;
    wire [2:0] aluControl;

    instruction_control ic(
        .cmdOp      ( cmdOp      ), .cmdF3      ( cmdF3      ), .cmdF7      ( cmdF7      ),
        .branch     ( branch     ), .condZero   ( condZero   ), .regWrite   ( regWrite   ),
        .memWrite   ( memWrite   ), .aluSrc     ( aluSrc     ), .wdSrc      ( wdSrc      ),
        .aluControl ( aluControl )
    );

    always_ff @ (posedge clk) begin
        vld_D         <=  rst_n ? '1 : '0;
        rd_D          <=  rd;
        rs1_D         <=  rs1;
        rs2_D         <=  rs2;
        immI_D        <=  immI;
        immS_D        <=  immS;
        immB_D        <=  immB;
        immU_D        <=  immU;
        branch_D      <=  branch;
        condZero_D    <=  condZero;
        regWrite_D    <=  regWrite;
        memWrite_D    <=  memWrite;
        aluSrc_D      <=  aluSrc;
        wdSrc_D       <=  wdSrc;
        aluControl_D  <=  aluControl;
        pc_D          <=  pc;
    end

    // #####################################
    // ######## INSTURCTION EXECUTE ########
    // #####################################

    // EX stage registers
    logic        regWrite_E;
    logic        memWrite_E;
    logic        wdSrc_E;
    logic [31:0] wData_E;
    logic [31:0] memAddr_E;
    logic [31:0] memData_E;
    logic [31:0] pcNext_E;
    logic [ 4:0] rd_E;
    logic        vld_E;

    // register file wires
    wire [31:0] rd1;
    wire [31:0] rd2;

    // execution values
    wire [31:0] aluResult;
    wire        aluZero;
    wire [31:0] srcB        = aluSrc_D ? immI_D : rd2;
    wire [31:0] pcBranch    = pc_D + immB_D;
    wire [31:0] pcPlus4     = pc_D + 4;
    wire        pcSrc       = branch_D & (aluZero == condZero_D);
    wire [31:0] pcNext_new  = pcSrc ? pcBranch : pcPlus4;

    // ALU
    sr_alu alu (
        .srcA       ( rd1          ),
        .srcB       ( srcB         ),
        .oper       ( aluControl   ),
        .zero       ( aluZero      ),
        .result     ( aluResult    ) 
    );

    always_ff @ (posedge clk) begin
        vld_E         <=  rst_n ? vld_D : 1'b0;  
        regWrite_E    <=  regWrite_D;
        memWrite_E    <=  memWrite_D;
        wdSrc_E       <=  wdSrc_D[1]; // is_from_mem
        wData_E       <=  wdSrc_D[0] ? immU_D : aluResult;
        memAddr_E     <=  rd1 + (memWrite_D ? immS_D : immI_D);
        memData_E     <=  rd2;
        pcNext_E      <=  pcNext_new;
        rd_E          <=  rd_D;
    end

    // #####################################
    // ########### MEMORY ACCESS ###########
    // #####################################

    // MEM stage registers
    logic           regWrite_M;
    logic   [31:0]  pcNext_M;
    logic   [31:0]  wData_M;
    logic   [ 4:0]  rd_M;
    logic           vld_M;
    
    // memory access values
    wire [31:0] wData = wdSrc_E ? memData : wData_E;

    assign memAddr = memAddr_E;
    assign memWriteEnable = memWrite_E & vld_E;
    assign memData = memWriteEnable ? memData_E : 'bz;
    
    always_ff @ (posedge clk) begin
        vld_M           <=  rst_n ? vld_E : 1'b0;
        wData_M         <=  wData;
        regWrite_M      <=  regWrite_E;
        pcNext_M        <=  pcNext_E;
        rd_M            <=  rd_E;
    end

    // #####################################
    // ######## REGISTER WRITE BACK ########
    // #####################################

    // debug register access
    wire [31:0] rd0;
    assign regData = (regAddr != 0) ? rd0 : pc;
    sm_register_file rf (
        .clk        ( clk                ),
        .a0         ( regAddr            ),
        .a1         ( rs1_D              ),
        .a2         ( rs2_D              ),
        .a3         ( rd_M               ),
        .rd0        ( rd0                ),
        .rd1        ( rd1                ),
        .rd2        ( rd2                ),
        .wd3        ( wData_M            ),
        .we3        ( regWrite_M & vld_M )
    );
endmodule

module instruction_decode
(
    input   [31:0]  instr,
    output  [ 6:0]  cmdOp,
    output  [ 4:0]  rd,
    output  [ 2:0]  cmdF3,
    output  [ 4:0]  rs1,
    output  [ 4:0]  rs2,
    output  [ 6:0]  cmdF7,
    output  [31:0]  immI,
    output  [31:0]  immS,
    output  [31:0]  immB,
    output  [31:0]  immU
);
    assign cmdOp   =   instr[ 6: 0];
    assign rd      =   instr[11: 7];
    assign cmdF3   =   instr[14:12];
    assign rs1     =   instr[19:15];
    assign rs2     =   instr[24:20];
    assign cmdF7   =   instr[31:25];
    assign immI    =   { { 21{ instr[31] } }, instr[30:20] };
    assign immS    =   { { 21{ instr[31] } }, instr[30:25], instr[11:7] };
    assign immB    =   { { 20{ instr[31] } }, instr[7], instr[30:25], instr[11:8], 1'b0 };
    assign immU    =   { instr[31:12], 12'b0 };
endmodule

module instruction_control
(
    input           [ 6:0]  cmdOp,
    input           [ 2:0]  cmdF3,
    input           [ 6:0]  cmdF7,
    output logic            branch,
    output logic            condZero,
    output logic            regWrite, 
    output logic            memWrite,
    output logic            aluSrc,
    output logic    [1:0]   wdSrc,
    output logic    [2:0]   aluControl
);
    always @ (*) begin
        branch      = 1'b0;
        condZero    = 1'b0;
        regWrite    = 1'b0;
        aluSrc      = 1'b0;
        wdSrc       = 2'b00;
        memWrite    = 1'b0;
        aluControl  = `ALU_ADD;

        casez( {cmdF7, cmdF3, cmdOp} )
            { `RVF7_ADD,  `RVF3_ADD,  `RVOP_ADD  } : begin regWrite = 1'b1; aluControl = `ALU_ADD;  end
            { `RVF7_OR,   `RVF3_OR,   `RVOP_OR   } : begin regWrite = 1'b1; aluControl = `ALU_OR;   end
            { `RVF7_SRL,  `RVF3_SRL,  `RVOP_SRL  } : begin regWrite = 1'b1; aluControl = `ALU_SRL;  end
            { `RVF7_SLTU, `RVF3_SLTU, `RVOP_SLTU } : begin regWrite = 1'b1; aluControl = `ALU_SLTU; end
            { `RVF7_SUB,  `RVF3_SUB,  `RVOP_SUB  } : begin regWrite = 1'b1; aluControl = `ALU_SUB;  end

            { `RVF7_ANY,  `RVF3_ADDI, `RVOP_ADDI } : begin regWrite = 1'b1; aluSrc = 1'b1; aluControl = `ALU_ADD; end
            { `RVF7_ANY,  `RVF3_ANY,  `RVOP_LUI  } : begin regWrite = 1'b1; wdSrc[0]  = 1'b1; end

            { `RVF7_ANY,  `RVF3_BEQ,  `RVOP_BEQ  } : begin branch = 1'b1; condZero = 1'b1; aluControl = `ALU_SUB; end
            { `RVF7_ANY,  `RVF3_BNE,  `RVOP_BNE  } : begin branch = 1'b1; aluControl = `ALU_SUB; end

            { `RVF7_ANY,  `RVF3_WORD, `RVOP_LOAD } : begin regWrite = 1'b1; wdSrc[1] = 1'b1; end
            { `RVF7_ANY,  `RVF3_WORD, `RVOP_STORE } : begin memWrite = 1'b1; end
        endcase
    end
endmodule

module sr_alu
(
    input  [31:0] srcA,
    input  [31:0] srcB,
    input  [ 2:0] oper,
    output        zero,
    output reg [31:0] result
);
    always @ (*) begin
        case (oper)
            default   : result = srcA + srcB;
            `ALU_ADD  : result = srcA + srcB;
            `ALU_OR   : result = srcA | srcB;
            `ALU_SRL  : result = srcA >> srcB [4:0];
            `ALU_SLTU : result = (srcA < srcB) ? 1 : 0;
            `ALU_SUB : result = srcA - srcB;
        endcase
    end

    assign zero   = (result == 0);
endmodule

module sm_register_file
(
    input         clk,
    input  [ 4:0] a0,
    input  [ 4:0] a1,
    input  [ 4:0] a2,
    input  [ 4:0] a3,
    output [31:0] rd0,
    output [31:0] rd1,
    output [31:0] rd2,
    input  [31:0] wd3,
    input         we3
);
    reg [31:0] rf [31:0];

    assign rd0 = (a0 != 0) ? rf [a0] : 32'b0;
    assign rd1 = (a1 != 0) ? rf [a1] : 32'b0;
    assign rd2 = (a2 != 0) ? rf [a2] : 32'b0;

    always @ (posedge clk)
        if(we3) rf [a3] <= wd3;
endmodule
