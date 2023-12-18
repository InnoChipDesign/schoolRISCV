module io_bus
#(
    parameter LED_COUNT = 10,  // on-off LEDs
    parameter HEX_COUNT = 6,   // 7-segment displays
    parameter BTN_COUNT = 2,   // buttons
    parameter SW_COUNT  = 10,  // switches
    parameter GPIO_COUNT = 36, // GPIO ports
    parameter SIZE = 8
)
(
    input         clk,
    input  [31:0] addr,
    input         writeEnable,
    inout  [31:0] data,
    output [LED_COUNT-1:0] leds,
    input  [SW_COUNT -1:0] switches
);
    wire actualAddr = addr >> 2;
    logic [31:0] regs [0 : SIZE - 1];

    assign leds = regs[0][9:0];

    // [ 0 - 1 ] : LEDS
    // [ 2 - 3 ] : Switches

    wire [31:0] swData = { 22'b0, switches };
    wire [31:0] actualData = (addr == 32'd8) ? swData : regs[actualAddr];
    assign data = writeEnable ? 'z : actualData;

    always_ff @ (posedge clk)
        if (writeEnable) 
            regs[actualAddr] <= data;

endmodule
