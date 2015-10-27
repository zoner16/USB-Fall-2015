module bitstream
  (input logic clk, rst_b,
   input logic [7:0] pid,
   input logic [6:0] addr,
   input logic [3:0] endp,
   input logic init,
   input logic 	stop_stream,
   output logic bit_out,
   output logic NRZ_start,
   output logic stream_begin,
   output logic stream_done);

   logic [5:0] count;
   logic [7:0] npid;
   logic [7:0] sync;	

   assign npid = ~pid;
   assign sync = 8'd10000000;

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
	   newCount = 0;
	   nextState = Send;
	   NRZ_start = 1;
	end
	Send: begin
	   newCount = count + 1;
	   if (count < 6'd8) begin
	   bit_out = sync[count];
	   end
	   else if (count < 6'd12) bit_out = pid[count - 6'd8];
	   else if (count < 6'd16) bit_out = npid[count - 6'd12];
	   else if (count < 6'd23) begin
		if (count == 6'd16) stream_begin = 1;
		bit_out = addr[count - 6'd16];
	   end
	   else if (count < 6'd27) begin
		bit_out = endp[count - 6'd23];
		if (count == 6'd26) begin 
			stream_done = 1;
			nextState = Hold;
		end
	   end
	end
      endcase
   end
   
endmodule: bitstream
