module readWriteFSM
  (input bit clk, rst_b,
   input bit [15:0] FSMmempage,
   input bit [63:0] data_from_OS, data_from_device,
   input bit read,
   input bit write,
   output bit [63:0] data_to_device, data_to_host,
   output bit in_trans, out_trans,
   output bit isValueReadCorrect,
   output bit read_write_FSM_done);
   
   
   always_ff @(posedge clk, negedge rst_b) begin
      if (~rst_b) begin
	 state <= Hold;
	 isValueReadCorrect <= 0;
	 read_write_FSM_done <= 0;
	 in_trans <= 0;
	 out_trans <= 0;
      end
      else begin
	 case(state)
           Hold: begin
	      isValueReadCorrect <= 0;
	      read_write_FSM_done <= 0;
	      in_trans <= 0;
	      out_trans <= 0;
              if (read || write) state <= ReadOut;
	   end
           Out: begin // out trans then in
              out_trans <= 1;
              data_to_device <= {48'd0,FSMmempage}; //addr gets sent out as data
	      if (success) begin
		 state <= ReadIn;
	      end
	      else if (failure) begin
		 state <= Hold;
		 read_write_FSM_done <= 1;
		 isValueReadCorrect <= 0;
	      end
	      if (read) state <= ReadIn;
	      else if (write) state <= WriteOut;
	   end
           ReadIn: begin
	      in_trans <= 1;
	      data_to_OS <= data_from_device;
	      if (success) begin
		 state <= Hold;
		 read_write_FSM_done <= 1;
		 isValueReadCorrect <= 1;
	      end
	      else if (failure) begin
		 state <= Hold;
		 read_write_FSM_done <= 1;
		 isValueReadCorrect <= 0;
	      end
	   end // case: ReadIn
	   WriteOut: begin
	      out_trans <= 1;
	      data_to_device <= data_from_OS;
	      if (success) begin
		 state <= Hold;
		 read_write_FSM_done <= 1;
		 isValueReadCorrect <= 1;
	      end
	      else if (failure) begin
		 state <= Hold;
		 read_write_FSM_done <= 1;
		 isValueReadCorrect <= 0;
	      end
	   end
	 endcase
      end
   end 

endmodule: readWriteFSM
