module readWriteFSM
 (input bit clk, rst_b,
  input bit [15:0] FSMmempage,
  input bit [63:0] data_in,
  input bit read,
  input bit write,
  output bit [63:0] valueRead,
  output bit done);


	always_ff @(posedge clk, negedge rst_b) begin
		if (~rst_b) state <= Hold;
		else begin
			case(state)
                Hold: 
                     if (read) state <= Read;
                     else if (write) state <= Write;
                ReadOut: // out trans then in
                    outTrans <= 1;
                    protocolMemPage <= FSMmempage;
                    if (readOutDone) state <= ReadIn;
                ReadIn:
                Write: // out trans then out
		    endcase
	end
  end 

endmodule: readWriteFSM
