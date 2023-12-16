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
    // instruction decode wires
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

    // program counter
    wire [31:0] pc;
    wire [31:0] pcPlus4  = pc + 4;
    sm_register r_pc(clk, rst_n, pcPlus4, pc);

    // program memory access
    assign imAddr = pc >> 2;

    // register file
    wire [31:0] rd0;
    wire [31:0] rd1;
    wire [31:0] rd2;
    wire [31:0] wd3;
    // debug register access
    assign regData = (regAddr != 0) ? rd0 : pc;

    // instructsion decode
    decode_stage id (
        .clk        ( clk          ),
        .rst        ( ~rst_n       ),
        .instr      ( imData       ),
        .pc_in      ( pc           ),
        .inVld      ( 1'b1         ),
        .outVld     ( vld_D        ),
        .rd         ( rd_D         ),
        .rs1        ( rs1_D        ),
        .rs2        ( rs2_D        ),
        .immI       ( immI_D       ),
        .immS       ( immS_D       ),
        .immB       ( immB_D       ),
        .immU       ( immU_D       ),
        .branch     ( branch_D     ),
        .condZero   ( condZero_D   ),
        .regWrite   ( regWrite_D   ),
        .memWrite   ( memWrite_D   ),
        .aluSrc     ( aluSrc_D     ),
        .wdSrc      ( wdSrc_D      ),
        .aluControl ( aluControl_D ),
        .pc_out     ( pc_D         )
    );

    logic        regWrite_E;
    logic        memWrite_E;
    logic        wdSrc_E;
    logic [31:0] wData_E;
    logic [31:0] memAddr_E;
    logic [31:0] memData_E;
    logic [31:0] pcNext_E;
    logic [ 4:0] rd_E;
    logic        vld_E;

    execute_stage ex (
        .clk             ( clk          ),
        .rst             ( ~rst_n       ),
        .inVld           ( vld_D        ),
        .outVld          ( vld_E        ),
        .pc              ( pc_D         ),
        .rd1             ( rd1          ),
        .rd2             ( rd2          ),
        .rd_in           ( rd_D         ),
        .aluSrc          ( aluSrc_D     ),
        .aluControl      ( aluControl_D ),
        .branch          ( branch_D     ),
        .condZero        ( condZero_D   ),
        .immI            ( immI_D       ),
        .immS            ( immS_D       ),
        .immB            ( immB_D       ),
        .immU            ( immU_D       ),
        .regWrite_in     ( regWrite_D   ),
        .memWrite_in     ( memWrite_D   ),
        .wdSrc_in        ( wdSrc_D      ),
        .regWrite_out    ( regWrite_E   ),
        .memWrite_out    ( memWrite_E   ),
        .wdSrc_out       ( wdSrc_E      ),
        .wData           ( wData_E      ),
        .memAddr         ( memAddr_E    ),
        .memData         ( memData_E    ),
        .pcNext          ( pcNext_E     ),
        .rd_out          ( rd_E         )
    );

    logic   [31:0]  regWrite_M;
    logic   [31:0]  pcNext_M;
    logic   [ 4:0]  rd_M;
    logic           vld_M;

    assign memAddr = memAddr_E;
    assign memWriteEnable = memWrite_E & vld_E;
    assign memData = memWriteEnable ? memData_E : 'bz;

    memory_access_stage ma (
        .clk             ( clk           ),
        .rst             ( ~rst_n        ),
        .inVld           ( vld_E         ),
        .outVld          ( vld_M         ),
        .regWrite_in     ( regWrite_E    ),
        .wData_in        ( wData_E       ),
        .pcNext_in       ( pcNext_E      ),
        .wdSrc           ( wdSrc_E       ),
        .memData         ( memData       ),
        .rd_in           ( rd_E          ),
        .regWrite_out    ( regWrite_M    ),
        .wData_out       ( wd3           ), // connect to register file
        .pcNext          ( pcNext_M      ),
        .rd_out          ( rd_M          )
    );

    wire        regWrite = regWrite_M & vld_M;

    sm_register_file rf (
        .clk        ( clk          ),
        .a0         ( regAddr      ),
        .a1         ( rs1_D        ),
        .a2         ( rs2_D        ),
        .a3         ( rd_M         ),
        .rd0        ( rd0          ),
        .rd1        ( rd1          ),
        .rd2        ( rd2          ),
        .wd3        ( wd3          ),
        .we3        ( regWrite     )
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

module decode_stage
(
    input                   clk,
    input                   rst,
    input           [31:0]  instr,
    input           [31:0]  pc_in,
    input                   inVld,
    output logic            outVld,
    output logic    [ 4:0]  rd,
    output logic    [ 4:0]  rs1,
    output logic    [ 4:0]  rs2,
    output logic    [31:0]  immI,
    output logic    [31:0]  immS,
    output logic    [31:0]  immB,
    output logic    [31:0]  immU,
    output logic            branch,
    output logic            condZero,
    output logic            regWrite, 
    output logic            memWrite,
    output logic            aluSrc,
    output logic    [1:0]   wdSrc,
    output logic    [2:0]   aluControl,
    output logic    [31:0]  pc_out
);
    logic [ 6:0] cmdOp;
    logic [ 2:0] cmdF3;
    logic [ 6:0] cmdF7;

    logic [ 4:0] rd_new;
    logic [ 4:0] rs1_new;
    logic [ 4:0] rs2_new;
    logic [31:0] immI_new;
    logic [31:0] immS_new;
    logic [31:0] immB_new;
    logic [31:0] immU_new;

    instruction_decode id(
        .instr      ( instr      ),
        .cmdOp      ( cmdOp      ),
        .cmdF3      ( cmdF3      ),
        .cmdF7      ( cmdF7      ),
        .rd         ( rd_new     ),
        .rs1        ( rs1_new    ),
        .rs2        ( rs2_new    ),
        .immI       ( immI_new   ),
        .immS       ( immS_new   ),
        .immB       ( immB_new   ),
        .immU       ( immU_new   ) 
    );
    
    logic       branch_new;
    logic       condZero_new;
    logic       regWrite_new; 
    logic       memWrite_new;
    logic       aluSrc_new;
    logic [1:0] wdSrc_new;
    logic [2:0] aluControl_new;

    instruction_control ic(
        .cmdOp      ( cmdOp          ),
        .cmdF3      ( cmdF3          ),
        .cmdF7      ( cmdF7          ),
        .branch     ( branch_new     ),
        .condZero   ( condZero_new   ),
        .regWrite   ( regWrite_new   ),
        .memWrite   ( memWrite_new   ),
        .aluSrc     ( aluSrc_new     ),
        .wdSrc      ( wdSrc_new      ),
        .aluControl ( aluControl_new )
    );

    always_ff @ (posedge clk) begin
        outVld      <=  rst ? 1'b0 : inVld;
        rd          <=  rd_new;
        rs1         <=  rs1_new;
        rs2         <=  rs2_new;
        immI        <=  immI_new;
        immS        <=  immS_new;
        immB        <=  immB_new;
        immU        <=  immU_new;
        branch      <=  branch_new;
        condZero    <=  condZero_new;
        regWrite    <=  regWrite_new;
        memWrite    <=  memWrite_new;
        aluSrc      <=  aluSrc_new;
        wdSrc       <=  wdSrc_new;
        aluControl  <=  aluControl_new;
        pc_out      <=  pc_in;
    end
endmodule

module execute_stage
(
    input                   clk,
    input                   rst,
    input                   inVld,
    output logic            outVld,

    input logic     [31:0]  pc,
    input logic     [31:0]  rd1,
    input logic     [31:0]  rd2,
    input logic             aluSrc,
    input logic     [2:0]   aluControl,
    input logic             branch,
    input logic             condZero,
    input logic     [31:0]  immI,
    input logic     [31:0]  immS,
    input logic     [31:0]  immB,
    input logic     [31:0]  immU,

    input logic     [ 4:0]  rd_in,
    input logic             regWrite_in, 
    input logic             memWrite_in,
    input logic     [1:0]   wdSrc_in,

    output logic            regWrite_out, 
    output logic            memWrite_out,
    output logic            wdSrc_out,
    output logic    [ 4:0]  rd_out,

    output logic    [31:0]  wData,
    output logic    [31:0]  memAddr,
    output logic    [31:0]  memData,
    output logic    [31:0]  pcNext
);
    wire [31:0] srcB = aluSrc ? immI : rd2;
    wire [31:0] aluResult;
    logic aluZero;

    wire [31:0] pcBranch    = pc + immB;
    wire [31:0] pcPlus4     = pc + 4;
    wire        pcSrc       = branch & (aluZero == condZero);
    wire [31:0] pcNext_new  = pcSrc ? pcBranch : pcPlus4;

    sr_alu alu (
        .srcA       ( rd1          ),
        .srcB       ( srcB         ),
        .oper       ( aluControl   ),
        .zero       ( aluZero      ),
        .result     ( aluResult    ) 
    );

    always_ff @ (posedge clk) begin
        outVld          <=  rst ? 1'b0 : inVld;;  
        regWrite_out    <=  regWrite_in;
        memWrite_out    <=  memWrite_in;
        wdSrc_out       <=  wdSrc_in[1]; // is_from_mem
        wData           <=  wdSrc_in[0] ? immU : aluResult;
        memAddr         <=  rd1 + (memWrite_in ? immS : immI);
        memData         <=  rd2;      
        pcNext          <=  pcNext_new;
        rd_out          <=  rd_in;
    end
endmodule

module memory_access_stage
(
    input                   clk,
    input                   rst,
    input                   inVld,
    output logic            outVld,

    input logic             regWrite_in,
    input logic    [31:0]   wData_in,
    input logic    [31:0]   pcNext_in,
    input logic    [31:0]   memData,
    input logic    [ 4:0]   rd_in,

    input logic             wdSrc,

    output logic   [31:0]   regWrite_out,
    output logic   [31:0]   wData_out,
    output logic   [31:0]   pcNext,
    output logic   [ 4:0]   rd_out
);
    wire    [31:0] wData = wdSrc ? memData : wData_in;
    
    always_ff @ (posedge clk) begin
        outVld          <=  rst ? 1'b0 : inVld;;
        regWrite_out    <=  regWrite_in;
        wData_out       <=  wData;
        pcNext          <=  pcNext_in;
        rd_out          <=  rd_in;
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
