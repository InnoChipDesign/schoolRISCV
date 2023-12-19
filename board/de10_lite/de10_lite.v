

module de10_lite(

    input               ADC_CLK_10,
    input               MAX10_CLK1_50,
    input               MAX10_CLK2_50,

    output  [  7:0 ]    HEX0,
    output  [  7:0 ]    HEX1,
    output  [  7:0 ]    HEX2,
    output  [  7:0 ]    HEX3,
    output  [  7:0 ]    HEX4,
    output  [  7:0 ]    HEX5,

    input   [  1:0 ]    KEY,

    output  [  9:0 ]    LEDR,

    input   [  9:0 ]    SW,

    inout   [ 35:0 ]    GPIO
);

    // wires & inputs
    wire          clk;
    wire          clkIn     =  MAX10_CLK1_50;
    wire          enableCtl =  SW[9];
    wire          clkEnable =  enableCtl ? (SW[8] | ~KEY[1]) : 1'b1;
    wire          rst_n     =  enableCtl ? KEY[0] : 1'b1;
    wire [  3:0 ] clkDivide =  enableCtl ? { 1'b0, SW [7:5] } : 4'b0;
    wire [  4:0 ] regAddr   =  SW [4:0];
    wire [ 31:0 ] regData;

    // IO wires
    wire [ 31:0 ] busAddr;
    wire [ 31:0 ] busData;
    wire          writeEnable;

    wire [  9:0 ] ledr_driven;

    wire [ 47:0 ] hex_driven;
    
    // IO bus
    io_bus iobus 
    (
        .clk(clkIn), 
        .addr(busAddr), 
        .data(busData), 
        .writeEnable(writeEnable), 
        .switches(SW),
        .btns(KEY),
        .hex(hex_driven),
        .leds(ledr_driven) 
    );

    //cores
    sm_top sm_top
    (
        .clkIn      ( clkIn     ),
        .rst_n      ( rst_n     ),
        .clkDivide  ( clkDivide ),
        .clkEnable  ( clkEnable ),
        .clk        ( clk       ),
        .regAddr    ( regAddr   ),
        .regData    ( regData   ),
        .busAddr    ( busAddr   ),
        .ioWriteEnable ( writeEnable ),
        .ioData     ( busData   )
    );

    // outputs
    assign LEDR[0]   = enableCtl ? clk : ledr_driven[0];
    assign LEDR[9:1] = ledr_driven[9:1];

    wire [ 31:0 ] h7segment = regData;

    assign HEX0 [7] = enableCtl ? 1'b1 : hex_driven[7];
    assign HEX1 [7] = enableCtl ? 1'b1 : hex_driven[15];
    assign HEX2 [7] = enableCtl ? 1'b1 : hex_driven[23];
    assign HEX3 [7] = enableCtl ? 1'b1 : hex_driven[31];
    assign HEX4 [7] = enableCtl ? 1'b1 : hex_driven[39];
    assign HEX5 [7] = enableCtl ? 1'b1 : hex_driven[47];

    wire [6:0] h7_out0;
    wire [6:0] h7_out1;
    wire [6:0] h7_out2;
    wire [6:0] h7_out3;
    wire [6:0] h7_out4;
    wire [6:0] h7_out5;

    // wire [ 47:0 ] hex_driven = { hex5_driven, hex4_driven, hex3_driven, hex2_driven, hex1_driven, hex0_driven };

    assign HEX0 [6:0] = enableCtl ? h7_out0 : hex_driven[6:0];
    assign HEX1 [6:0] = enableCtl ? h7_out1 : hex_driven[14:8];
    assign HEX2 [6:0] = enableCtl ? h7_out2 : hex_driven[22:16];
    assign HEX3 [6:0] = enableCtl ? h7_out3 : hex_driven[30:24];
    assign HEX4 [6:0] = enableCtl ? h7_out4 : hex_driven[38:32];
    assign HEX5 [6:0] = enableCtl ? h7_out5 : hex_driven[46:40];

    sm_hex_display digit_5 ( h7segment [23:20] , h7_out5 );
    sm_hex_display digit_4 ( h7segment [19:16] , h7_out4 );
    sm_hex_display digit_3 ( h7segment [15:12] , h7_out3 );
    sm_hex_display digit_2 ( h7segment [11: 8] , h7_out2 );
    sm_hex_display digit_1 ( h7segment [ 7: 4] , h7_out1 );
    sm_hex_display digit_0 ( h7segment [ 3: 0] , h7_out0 );

endmodule
