module data_bus
#(
    parameter IO_BIT_COUNT = 10
)
(
    input         clk,
    input  [31:0] addr,
    input         writeEnable,
    inout  [31:0] data,
    output        ioWriteEnable,
    inout  [31:0] ioData
);
    wire isIO     = addr[31:IO_BIT_COUNT] == '0;
    wire writeRam = writeEnable & ~isIO;
    assign ioWriteEnable = writeEnable & isIO;

    wire [31:0] ramData;
    int_ram cpu_ram(.clk(clk), .addr(addr), .writeEnable(writeRam), .data(ramData));

    assign data = writeEnable ? 'z : (isIO ? ioData : ramData);
    assign ramData = writeRam ? data : 'z;
    assign ioData  = ioWriteEnable  ? data : 'z;

endmodule
