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

    assign data = writeEnable ? 'bz : ram_contents[addr];

    always_ff @ (posedge clk)
        if (writeEnable) ram_contents[addr] <= data;

endmodule
