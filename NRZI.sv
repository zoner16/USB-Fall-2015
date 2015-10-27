module NRZI(clk, rst_L, in, out, NRZI_begin, NRZI_done, stuff_done);
    input logic clk, rst_L, in, NRZI_begin, stuff_done;
    output logic NRZI_done;
    
    enum logic [1:0] {STANDBY = 2'b00, SEND = 2'b01, EOP = 2'b10} state;
    output enum logic [1:0] {J = 2'b10, K = 2'b01, SE0 = 2'b00} out;
    
    logic [1:0] EOP_count;
   
    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            state = STANDBY;
            NRZI_done = 0;
            EOP_count = 0;
            out = J;
        end
        
        else begin
            case(state)
                STANDBY: begin
                    if(NRZI_begin) begin
                        state <= SEND;
                        out <= (in) ? J : K;
                    end
                    else begin
                        out <= J;
                    end
                end
                SEND: begin
                    if(stuff_done) begin
                        state <= EOP;
                    end
                    else begin
                        if(out == J) begin
                            out <= (in) ? J : K;
                        end
                        else begin
                            out <= (in) ? K : J;
                        end
                    end
                end
                EOP: begin
                    if(EOP_count == 2) begin
                        state <= STANDBY;
                        out <= J;
                        EOP_count <= 1'b0;
                        NRZI_done <= 1;
                    end
                    else begin
                        EOP_count <= EOP_count + 1'b1;
                        out <= SE0;
                    end
                end
            endcase
        end
    end
endmodule

