typedef struct packed {
  logic [3:0] pid;
  logic [3:0] endp;
  logic [6:0] addr;
  logic [63:0] data;
} pkt_t;

module protocolFSM
  (input bit         clk, rst_b, in_trans, out_trans, pkt_sent, pkt_received, crc_correct,
   input pkt_t       pkt_in,
   input bit [63:0]  data_from_host,
<<<<<<< HEAD
   output bit 	     failure, success, kill,
   output 	     pkt_t pkt_out,
=======
   output bit 	     failure, success, kill, encode, decode,
   output pkt_t      pkt_out,
   output bit [4:0]  crc_type,
>>>>>>> dcc8045820088459b56c6beb596f9c04ef22c38e
   output bit [63:0] data_to_host);

   logic [7:0] 	    clk_count;
   logic [3:0] 	    timeout_count, corrupted_count;   
   
   enum logic [2:0] {Hold= 3'd0, InTransWait= 3'd1, InTrans = 3'd2, OutTransWait = 3'd3, OutTransDataWait= 3'd4, OutTrans= 3'd5} state;
   
    always_ff @(posedge clk, negedge rst_b) begin
        if (~rst_b) begin
            state <= Hold;
            encode <= 0;
            decode <= 0;
            kill <= 0;
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
                        pkt_out.pid <= 4'b1001;
                        pkt_out.addr <= 7'd5;
                        pkt_out.endp <= 4'd4;
<<<<<<< HEAD
		        encode <= 1;
		        kill <= 1;
=======
                        crc_type <= 5'd5;
                        encode <= 1;
                        kill <= 1;
>>>>>>> dcc8045820088459b56c6beb596f9c04ef22c38e
                        state <= InTransWait;
                    end
                    else if (out_trans) begin
                        pkt_out.pid <= 4'b0001;
                        pkt_out.addr <= 7'd5;
                        pkt_out.endp <= 4'd4;
<<<<<<< HEAD
		        encode <= 1;
		        kill <= 1;
=======
                        crc_type <= 5'd5;
                        encode <= 1;
                        kill <= 1;
>>>>>>> dcc8045820088459b56c6beb596f9c04ef22c38e
                        state <= OutTransWait;
                    end
                end // case: Hold
                InTransWait: begin // sent a packet and waiting for the signal that it was sent
                    clk_count <= 0;
                    encode <= 0;
                    kill <= 0;
                    if (pkt_sent) begin //datastream_out confirmed pckt sent
                        state <= InTrans;
                        decode <= 1;
                    end
                end
                InTrans: begin
                    decode <= 0;
                    if (corrupted_count == 4'd8) begin // corrupted max reached
                        state <= Hold;
                        failure <= 1;
                    end
                    else if (timeout_count == 4'd8) begin // timeout max reached
                        state <= Hold;
                        failure <= 1;
                    end
                    else if (pkt_received && ~crc_correct) begin // corrupted data sent to pFSM
                        corrupted_count <= corrupted_count + 1;
                        pkt_out.pid <= 4'b1010; //send nak
                        encode <= 1;
                        kill <= 1;
                        state <= InTransWait;
                    end
                    else if (pkt_received && crc_correct) begin //correct data sent to pFSM
                        state <= Hold;
                        success <= 1;
                        pkt_out.pid <= 4'b0010; //send ack
                        encode <= 1;
                        kill <= 1;
                        data_to_host <= pkt_in.data;
                    end 
                    else if (clk_count == 8'd255) begin //timeout reached
                        timeout_count <= timeout_count + 1'd1;
                        pkt_out.pid <= 4'b1010; //send nak
                        encode <= 1;
                        kill <= 1;
                        state <= InTransWait;
                    end
                    else begin
                        clk_count <= clk_count + 1'd1; //keep track of clk cycles
                    end
                end // case: InTrans
<<<<<<< HEAD
	        OutTransWait: begin //wait for datastream to confirm out pkt sent
		   encode <= 0;
		   kill <= 0;
		   if (pkt_sent) begin
		      state <= OutTransDataWait;
		      pkt_out.pid <= 4'b0011; //send data
                      pkt_out.data <= data_from_host;
		      encode <= 1;
		      kill <= 1;
		   end
		end
=======
                OutTransWait: begin //wait for datastream to confirm out pkt sent
                    encode <= 0;
                    kill <= 0;
                    if (pkt_sent) begin
                        state <= OutTransDataWait;
                        pkt_out.pid <= 4'b0011; //send data
                        pkt_out.data <= data_from_host;
                        crc_type = 5'd16;
                        encode <= 1;
                        kill <= 1;
                    end
                end
>>>>>>> dcc8045820088459b56c6beb596f9c04ef22c38e
                OutTransDataWait: begin //wait for datastream to confirm data pkt sent
                    clk_count <= 0;
                    encode <= 0;
                    kill <= 0;
                    if (pkt_sent) begin
                        state <= OutTrans;
                        decode <= 1;
                    end
                end
                OutTrans: begin
                    decode <= 0;
                    if (corrupted_count == 4'd8) begin // corrupted max reached
                        state <= Hold;
                        failure <= 1;
                    end
                    else if (timeout_count == 4'd8) begin // timeout max reached
                        state <= Hold;
                        failure <= 1;
                    end
                    else if (pkt_received && pkt_in.pid == 4'b1010) begin
                         // received nak so resend data
<<<<<<< HEAD
		       state <= OutTransDataWait;
		       pkt_out.pid <= 4'b0011;
                       pkt_out.data <= data_from_host;
		       encode <= 1;
		       kill <= 1;
                       corrupted_count <= corrupted_count + 1'd1;
=======
                        state <= OutTransDataWait;
                        pkt_out.pid <= 4'b0011;
                        pkt_out.data <= data_from_host;
                        crc_type = 5'd16;
                        encode <= 1;
                        kill <= 1;
                        corrupted_count <= corrupted_count + 1'd1;
>>>>>>> dcc8045820088459b56c6beb596f9c04ef22c38e
                    end
                    else if (pkt_received && pkt_in.pid == 4'b0010) begin //received ack
                        state <= Hold;
                        success <= 1;
                    end
                    else if (clk_count == 8'd255) begin //timeout reached so resend data
                        timeout_count <= timeout_count + 1'd1;
                        state <= OutTransDataWait;
                        pkt_out.pid <= 4'b0011;
                        pkt_out.data <= data_from_host;
<<<<<<< HEAD
		        encode <= 1;
		        kill <= 1;
=======
                        crc_type = 5'd16;
                        encode <= 1;
                        kill <= 1;
>>>>>>> dcc8045820088459b56c6beb596f9c04ef22c38e
                    end
                    else begin
                        clk_count <= clk_count + 1'd1; //keep track of clk cycles
                    end
                end
           endcase
        end
    end

endmodule: protocolFSM
