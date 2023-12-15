module sm_ram
#(
    parameter SIZE = 64
)
(
    input         clk,
    input  [31:0] addr,
    input  [31:0] value,
    input         writeEnable,
    output [31:0] result
);
    reg [31:0] ram_contents [SIZE - 1:0];
    assign result = ram_contents[addr];

    always @ (posedge clk)
        if (writeEnable)
            ram_contents[addr] = value;

endmodule
