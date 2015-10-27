module bitStream (clk, rst_L, encode, pid, addr, endp, halt_stream, out, NRZI_begin, stream_begin, stream_done);
   
    input logic clk, rst_L, encode, halt_stream;
    input logic [3:0] pid, endp;
    input logic [6:0] addr;
    output logic out, NRZI_begin, stream_begin, stream_done;

    logic [6:0] count, size;
    logic [3:0] npid;
    logic [7:0] sync;
    logic [26:0] packet;
    

    assign npid = ~pid;
    assign sync = 8'b10000000;
    assign size = 7'd27;

    enum logic {STANDBY = 1'b0, SEND = 1'b1} state;
   
    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) begin
            state = STANDBY;
            count = 0;
            NRZI_begin = 0;
            stream_begin = 0;
            stream_done = 0;
        end
        else if (~halt_stream) begin
            case(state) 
                STANDBY: begin
                    stream_done <= 0;
                    if (encode) begin
                        NRZI_begin <= 1;
                        state <= SEND;
                        packet <= {endp, addr, pid, npid, sync};
                    end
                end
                SEND: begin
                    out <= packet[count]; 
                    packet <= packet >> 1;
                    count <= count + 1'b1;
                    if(count == size - 1'b1) begin
                        state <= STANDBY;
                        stream_done <= 1;
                    end
                    else if (count <= 7'd7) begin
                        stream_begin <= 1;
                    end
                    else begin
                        stream_begin <= 0;
                    end
                end                
            endcase
        end
    end
    
   
endmodule: bitStream
