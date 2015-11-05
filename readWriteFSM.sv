typedef struct packed {
  logic [3:0] pid;
  logic [3:0] endp;
  logic [6:0] addr;
  logic [63:0] data;
} pkt_t;

module readWriteFSM
  (input bit         clk, rst_b, read, write, failure, success,
   input bit [15:0]  FSMmempage,
   input bit [63:0]  data_from_OS, data_from_device,
   output bit [63:0] data_to_device, data_to_OS,
   output bit        in_trans, out_trans, isValueReadCorrect, read_write_FSM_done);
   
    enum logic [1:0] {Hold = 2'b00, Out = 2'b01, ReadIn = 2'b10, WriteOut = 2'b11} state;
   
   
    always_ff @(posedge clk, negedge rst_b) begin
        if (~rst_b) begin
            state = Hold;
            isValueReadCorrect = 0;
            read_write_FSM_done = 0;
	    data_to_device = 0;
            in_trans = 0;
            out_trans = 0;
	    data_to_OS = 0;
        end
        else begin
            case(state)
                Hold: begin
                    isValueReadCorrect <= 0;
                    read_write_FSM_done <= 0;
                    in_trans <= 0;
                    out_trans <= 0;
                    if (read || write) begin
<<<<<<< HEAD
                       state <= Out;
		       out_trans <= 1;
                       data_to_device[15:0] <= FSMmempage;
                    end
                end
                Out: begin // out transaction occurs for both read and write
		    out_trans <= 0;
=======
                        state <= Out;
                        out_trans <= 1;
                        data_to_device <= {48'd0, FSMmempage};
                    end
                end
                Out: begin // out trans then in
                out_trans <= 0;
>>>>>>> dcc8045820088459b56c6beb596f9c04ef22c38e
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
<<<<<<< HEAD
=======
                    data_to_OS <= data_from_device;
>>>>>>> dcc8045820088459b56c6beb596f9c04ef22c38e
                    if (success) begin
                        state <= Hold;
                        read_write_FSM_done <= 1;
                        isValueReadCorrect <= 1;
		        data_to_OS <= data_from_device;
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
