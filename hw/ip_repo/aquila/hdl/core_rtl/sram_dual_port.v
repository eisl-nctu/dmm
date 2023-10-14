//
// This module show you how to infer an initialized SRAM block
// in your circuit using the standard Verilog code.  The initial
// values of the SRAM cells is defined in the text file "image.dat"
// Each line defines a cell value. The number of data in image.dat
// must match the size of the sram block exactly.

module sram_dual_port #(parameter RAM_WIDTH = 40, RAM_ADDR_BITS = 16)(
input clka, 
input clkb, 
input we_A,
input we_B,
input en_A,  
input en_B,
input  [RAM_ADDR_BITS-1 : 0] addr_A, 
input  [RAM_ADDR_BITS-1 : 0] addr_B,
input  [RAM_WIDTH-1 : 0] data_iA,
input  [RAM_WIDTH-1 : 0] data_iB,
output reg [RAM_WIDTH-1 : 0] data_oA,
output reg [RAM_WIDTH-1 : 0] data_oB);
reg [0:RAM_WIDTH-1] RAM [(2**RAM_ADDR_BITS) -1:0];


// ------------------------------------
// BRAM Port-A read operation
// ------------------------------------
always@(posedge clka)
begin
  if (en_A & we_A)
    data_oA <= data_iA;
  else
    data_oA <= RAM[addr_A];
end

// ------------------------------------
// BRAM Port-B read operation
// ------------------------------------
always@(posedge clkb)
begin
  if (en_B & we_B)
    data_oB <= data_iB;
  else
    data_oB <= RAM[addr_B];
end

// ------------------------------------
// BRAM Port-A write operation
// ------------------------------------
always@(posedge clka)
begin
  if (en_A & we_A)
    RAM[addr_A] <= data_iA;
end

// ------------------------------------
// BRAM Port-B write operation
// ------------------------------------
always@(posedge clkb)
begin
  if (en_B & we_B)
    RAM[addr_B] <= data_iB;
end


endmodule
