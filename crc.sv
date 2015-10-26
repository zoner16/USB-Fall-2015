module crc
  (input logic bit_in,
   input logic 	clk, rst_b,
   input logic 	stream_begin, strea_done,
   input logic 	stop_stream,
   output logic bit_out,
   output logic out_remain);
   
   logic [2:0] 	count;
   always_ff @(posedge clk, negedge rst_b) begin
      if (~rst_b) begin
	 remainder[0] <= 1;
	 remainder[1] <= 1;
	 remainder[2] <= 1;
	 remainder[3] <= 1;
	 remainder[4] <= 1;
	 count <= 3'd4;
	 state <= Standby; 	
      end
      else if (~stop_stream) begin
	 remainder[0] <= in_flop[0];
	 remainder[1] <= in_flop[1];
	 remainder[2] <= in_flop[2];
	 remainder[3] <= in_flop[3];
	 remainder[4] <= in_flop[4];
	 state <= nextState;
	 count <= newCount;
      end
   end
   
   logic [4:0] in_flop;
   
   always_comb begin
      in_flop[0] = remainder[4] ^ bit_in;
      in_flop[1] = remainder[0];
      in_flop[2] = remainder[1] ^ bit_in;
      in_flop[3] = remainder[2];
      in_flop[4] = remainder[3];
   end
   
   always_comb begin
      nextState = state;
      newCount = count;
      case(state)
	Standby: begin
	   if (stream_begin) nextState = CRC;
	end
	CRC: begin
	   bit_out = bit_in;
	   if (stream_done) nextState = Send;
	end
	Send: begin
	   if (count == 0) begin
	      nextState = Standby;
	   end
	   else newCount = count - 1;
	   out_remain = ~remainder[count];
	end
      endcase
   end
   

endmodule: crc
