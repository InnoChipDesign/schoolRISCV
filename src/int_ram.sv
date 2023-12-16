module int_ram
#(
    parameter SIZE = 64
)
(
    input         clk,
    input  [31:0] addr,
    input         writeEnable,
    inout  [31:0] data
);
    logic [31:0] ram_contents [SIZE - 1:0];
    wire  [31:0] actualAddr = addr >> 2;

    assign data = writeEnable ? 'bz : ram_contents[actualAddr];

    always_ff @ (posedge clk)
        if (writeEnable) ram_contents[actualAddr] <= data;

endmodule
