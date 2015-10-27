module CRC(in, clk, rst_L, stream_begin, stream_done, halt_stream, out, CRC_done);
    
    input logic in, clk, rst_L, stream_begin, stream_done, halt_stream;
    output logic out, CRC_done;
   
    logic [2:0] count;
    logic [4:0] in_flop;
    
    enum logic [1:0] {STANDBY = 2'b00, CRC = 2'b01, SEND = 2'b10} state;
    
    assign out = (state == SEND) ? in_flop[0] : in;
   
    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) begin
            in_flop = 5'b11111;
            count = 0;
            CRC_done = 0;
            state = STANDBY; 	
        end
        else if (~halt_stream) begin
            case(state) 
                STANDBY: begin
                    CRC_done <= 0;
                    if(stream_begin) begin
                        state <= CRC;
                        in_flop[0] <= in_flop[4] ^ in;
                        in_flop[1] <= in_flop[0];
                        in_flop[2] <= in_flop[1] ^ in ^ in;
                        in_flop[3] <= in_flop[2];
                        in_flop[4] <= in_flop[3];
                    end   
                end
                CRC: begin
                    in_flop[0] <= in_flop[4] ^ in;
                    in_flop[1] <= in_flop[0];
                    in_flop[2] <= in_flop[1] ^ in_flop[4] ^ in;
                    in_flop[3] <= in_flop[2];
                    in_flop[4] <= in_flop[3];
                    if(stream_done) begin
                        state <= SEND;
                    end
                end
                SEND: begin
                    in_flop <= in_flop >> 1;
                    count <= count + 1'b1;
                    if(count == 4) begin
                        state <= STANDBY;
                        CRC_done <= 1;
                    end
                end
            endcase
        end
    end
    
endmodule: CRC
