/*
 * schoolRISCV - small RISC-V CPU 
 *
 * originally based on Sarah L. Harris MIPS CPU 
 *                   & schoolMIPS project
 * 
 * Copyright(c) 2017-2020 Stanislav Zhelnio 
 *                        Aleksandr Romanov 
 */ 

// hardware top level module
module sm_top
(
    input           clkIn,
    input           rst_n,
    input   [ 3:0 ] clkDivide,
    input           clkEnable,
    output          clk,
    input   [ 4:0 ] regAddr,
    output  [31:0 ] regData,
    output  [31:0 ] busAddr,
    output          ioWriteEnable,
    inout   [31:0 ] ioData
);
    // metastability input filters
    wire    [ 3:0 ] divide;
    wire            enable;
    wire    [ 4:0 ] addr;

    sm_debouncer #(.SIZE(4)) f0(clkIn, clkDivide, divide);
    sm_debouncer #(.SIZE(1)) f1(clkIn, clkEnable, enable);
    sm_debouncer #(.SIZE(5)) f2(clkIn, regAddr,   addr  );

    // cores
    // clock divider
    sm_clk_divider #(.shift(16)) sm_clk_divider
    (
        .clkIn      ( clkIn     ),
        .rst_n      ( 1'b1      ),
        .divide     ( divide    ),
        .enable     ( enable    ),
        .clkOut     ( clk       )
    );

    // instruction memory
    wire    [31:0]  imAddr;
    wire    [31:0]  imData;
    sm_rom #(.SIZE(128)) reset_rom(imAddr, imData);

    wire    [31:0]  busData;
    wire            busWriteEnable;
    data_bus dbus(.clk(clk), .addr(busAddr), .writeEnable(busWriteEnable), .data(busData), .ioWriteEnable(ioWriteEnable), .ioData(ioData));

    sr_cpu sm_cpu
    (
        .clk        ( clk       ),
        .rst_n      ( rst_n     ),
        .regAddr    ( addr      ),
        .regData    ( regData   ),
        .imAddr     ( imAddr    ),
        .imData     ( imData    ),
        .memAddr    ( busAddr   ),
        .memWriteEnable(busWriteEnable),
        .memData    ( busData   )
    );

endmodule

// metastability input debouncer module
module sm_debouncer
#(
    parameter SIZE = 1
)
(
    input                      clk,
    input      [ SIZE - 1 : 0] d,
    output reg [ SIZE - 1 : 0] q
);
    reg        [ SIZE - 1 : 0] data;

    always @ (posedge clk) begin
        data <= d;
        q    <= data;
    end

endmodule

// tunable clock divider
module sm_clk_divider
#(
    parameter shift  = 16,
              bypass = 0
)
(
    input           clkIn,
    input           rst_n,
    input   [ 3:0 ] divide,
    input           enable,
    output          clkOut
);
    wire [31:0] cntr;
    wire [31:0] cntrNext = cntr + 1;
    sm_register_we r_cntr(clkIn, rst_n, enable, cntrNext, cntr);

    assign clkOut = bypass ? clkIn 
                           : cntr[shift + divide];
endmodule
