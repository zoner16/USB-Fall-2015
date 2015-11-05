module readWriteFSM
  (input bit clk, rst_b, read, write, failure, success,
   input bit [15:0] FSMmempage,
   input bit [63:0] data_from_OS, data_from_device,
   output bit [63:0] data_to_device, data_to_OS,
   output bit in_trans, out_trans, isValueReadCorrect, read_write_FSM_done);
   
    enum logic [1:0] {Hold = 2'b00, Out = 2'b01, ReadIn = 2'b10, WriteOut = 2'b11} state;
   
   
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
                    if (read || write) begin
                       state <= Out;
		       out_trans <= 1;
                       data_to_device <= {48'd0, FSMmempage};
                    end
                end
                Out: begin // out trans then in
		    out_trans <= 0;
                    if (success) begin
		       if (read) begin
			  state <= ReadIn;
			  in_trans <= 1;
		       end
		       else begin
			  state <= WriteOut;
			  out_trans <= 1;
			  data_to_device <= data_from_OS;
		       end
                    end
                    else if (failure) begin
                        state <= Hold;
                        read_write_FSM_done <= 1;
                        isValueReadCorrect <= 0;
                    end
                end
                ReadIn: begin
                    in_trans <= 0;
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
                    out_trans <= 0;
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
