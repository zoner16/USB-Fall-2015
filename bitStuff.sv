module bitStuff(clk, rst_L, in, stream_begin, CRC_done, out, stuff_done, halt_stream);
    
    input logic clk, rst_L, in, stream_begin, CRC_done;
    output logic out, stuff_done, halt_stream;

    enum logic [1:0] {STANDBY = 2'b00, SEND = 2'b01, STUFF = 2'b10} state;

    logic [2:0] count;
    
    assign out = (state == STUFF) ? 1'b0 : in;

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            count = 0;
            state = STANDBY;
            stuff_done = 0;
            halt_stream = 0;
        end

        else begin
            case(state) 
                STANDBY: begin
                    stuff_done <= 0;
                    if(stream_begin) begin
                        state <= SEND;
                    end
                end
                SEND: begin
                    if(count == 5 && in == 1) begin
                        state <= STUFF;
                        halt_stream <= 1;
                        count <= 0;                        
                    end
                    else if(CRC_done) begin
                        state <= STANDBY;
                        stuff_done <= 1;
                        count <= 0;
                    end
                    else if(~in) begin
                        count <= 0;
                    end
                    else begin
                        count <= count + 1'b1;
                    end
                end
                STUFF: begin
                    halt_stream <= 0;
                    if(CRC_done) begin
                        state <= STANDBY;
                        stuff_done <= 1;
                    end
                    else begin
                        state <= SEND;
                    end
                end
            endcase
        end
    end
    
    

endmodule
