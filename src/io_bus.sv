module io_bus
#(
    parameter LED_COUNT = 10,  // on-off LEDs
    parameter HEX_COUNT = 6,   // 7-segment displays
    parameter BTN_COUNT = 2,   // buttons
    parameter SW_COUNT  = 10,  // switches
    parameter GPIO_COUNT = 36, // GPIO ports
    parameter SIZE = 16
)
(
    input         clk,
    input  [31:0] addr,
    input         writeEnable,
    inout  [31:0] data,
    input  [SW_COUNT -1:0] switches,
    input  [BTN_COUNT-1:0] btns,
    output [HEX_COUNT*8-1:0] hex,
    output [LED_COUNT-1:0] leds
);
    wire [31:0] actualAddr = addr >> 2;
    logic [31:0] regs [0 : SIZE - 1];

    assign leds = regs[1][9:0];
    assign hex  = { regs[8][15:0], regs[9] };

    // [ 0 - 1 ] : LEDS     (00, 04)
    // [ 2 - 3 ] : Switches (08, 12)
    // [ 4 - 5 ] : Buttons  (16, 20)
    // [ 6 - 9 ] : Hex      (24, 28, 32, 36)

    wire [31:0] keyData = { 30'b0, btns };
    wire [31:0] swData = { 22'b0, switches };
    wire [31:0] actualData = (addr == 32'd12) ? swData : (addr == 32'd20 ? keyData : regs[actualAddr]);
    assign data = writeEnable ? 'z : actualData;

    always_ff @ (posedge clk)
        if (writeEnable) 
            regs[actualAddr] <= data;

endmodule
