module protocolFSM
  (input bit clk, rst_b, in_trans, out_trans, data_ready, crc_correct,
   input bit [3:0] pid_in,
   input bit [63:0] data_from_device, data_from_host,
   output bit failure, success,
   output bit [3:0] pid, endp,
   output bit [4:0] crc_type,
   output bit [6:0] addr,
   output bit [63:0] data_to_host, data_to_device);

   logic [7:0] 	    clk_count;
   logic [3:0] 	    timeout_count, corrupted_count;   
   
   enum logic [1:0] {Hold= 2'd0, InTrans= 2'd1, OutTransData= 2'd2, OutTrans= 2'd3} state;
   
    always_ff @(posedge clk, negedge rst_b) begin
        if (~rst_b) begin
            state <= Hold;
            timeout_count <= 0;
            clk_count <= 0;
            failure <= 0;
            success <= 0;
            corrupted_count <= 0;
        end
        else begin
            case(state)
                Hold: begin
                    clk_count <= 0;
                    failure <= 0;
                    success <= 0;
                    timeout_count <= 0;
                    corrupted_count <= 0;
                    if (in_trans) begin
                        pid <= 4'b1001;
                        addr <= 7'd5;
                        endp <= 4'd4;
                        crc_type <= 5'd5;
                        state <= InTrans;
                    end
                    else if (out_trans) begin
                        pid <= 4'b0001;
                        addr <= 7'd5;
                        endp <= 4'd4;
                        crc_type <= 5'd5; 
                        state <= OutTransData;
                    end
                end
                InTrans: begin
                    if (corrupted_count == 4'd8) begin // corrupted max reached
                        state <= Hold;
                        failure <= 1;
                    end
                    else if (timeout_count == 4'd8) begin // timeout max reached
                        state <= Hold;
                        failure <= 1;
                    end
                    else if (data_ready && ~crc_correct) begin // corrupted data sent back
                        corrupted_count <= corrupted_count + 1;
                        clk_count <= 0;
                        pid <= 4'b1010; //send nak
                    end
                    else if (data_ready && crc_correct) begin //correct data sent back
                        state <= Hold;
                        success <= 1;
                        pid <= 4'b0010; //send ack
                        data_to_host <= data_from_device;
                    end 
                    else if (clk_count == 8'd255) begin //timeout reached
                        timeout_count <= timeout_count + 1'd1;
                        clk_count <= 0;
                    end
                    else begin
                        clk_count <= clk_count + 1'd1; //keep track of clk cycles
                    end
                end // case: InTrans
                OutTransData: begin // send data
                    pid <= 4'b0011;
                    data_to_device <= data_from_host;
                    crc_type = 5'd16;
                    state <= OutTrans;
                end
                OutTrans: begin
                    if (corrupted_count == 4'd8) begin // corrupted max reached
                        state <= Hold;
                        failure <= 1;
                    end
                    else if (timeout_count == 4'd8) begin // timeout max reached
                        state <= Hold;
                        failure <= 1;
                    end
                    else if (pid_in == 4'b0010) begin
                        state <= OutTransData; // received nak so resend data
                        corrupted_count <= corrupted_count + 1'd1;
                    end
                    else if (pid_in == 4'b0010) begin
                        state <= Hold;
                        success <= 1;
                    end
                    else if (clk_count == 8'd255) begin //timeout reached
                        timeout_count <= timeout_count + 1'd1;
                        clk_count <= 0;
                    end
                    else begin
                        clk_count <= clk_count + 1'd1; //keep track of clk cycles
                    end
                end
           endcase
        end
    end

endmodule: protocolFSM