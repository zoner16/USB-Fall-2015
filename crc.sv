module crc
  (input bit_in,
   input clk, rst_b,
   output bit_out,
   output [4:0] remainder);

   always_ff @(posedge clk) begin
	if (~rst_b) begin
	remainder[0] <= 1;
	remainder[1] <= 1;
	remainder[2] <= 1;
	remainder[3] <= 1;
	remainder[4] <= 1;	  	
	end
	else begin
	
	end
   end 
