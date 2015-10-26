module bitstream
   (input logic clk,rst_b,
    input logic bit_in,
    input logic stop_stream,
    output logic bit_out,
    output logic NRZ_start,
    output logic stream_begin,
    output logic stream_done);

   always_ff @(posedge clk, negedge rst_b)
   if (~rst_b) begin
	state <= Hold;
	count <= 0;
   end
   else if (~stop_stream) begin
	nextState <= state;
	count <= newCount;
   end

   always_comb begin
	nextstate = state;
	NRZ_start = 0;
	case(state)
	Hold: if (init) begin
		nextState = Send;
		NRZ_start = 1;
	end
	Send: begin
		newCount = count + 1;
		if (count ==
	end
	endcase
   end
    




endmodule: bitstream
