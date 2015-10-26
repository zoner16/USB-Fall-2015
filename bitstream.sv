module bitstream
  (input logic clk,rst_b,
   input logic 	bit_in,
   input logic 	stop_stream,
   input logic EOP,
   output logic bit_out,
   output logic NRZ_start,
   output logic stream_begin,
   output logic stream_done);

   logic [5:0] 	count;

   enum 	logic [0:0] {Hold = 1'd0, Send = 1'd1} state, nextState;
   
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
      stream_begin = 0;
      stream_done = 0;
      newCount = count;
      NRZ_start = 0;
      case(state)
	Hold: if (init) begin
	   nextState = Send;
	   NRZ_start = 1;
	end
	Send: begin
	   newCount = count + 1;
	   bit_out = bit_in;
	   if (count == 6'd15) stream_begin = 1;
	   else if (EOP) begin // EOP is 1 when D+ and D- are both 0
	      stream_done = 1;
	      nextState = Hold;
	   end
      endcase
   end
   
endmodule: bitstream
