`define OUT   4'b0001
`define IN    4'b1001
`define DATA0 4'b0011
`define ACK   4'b0010
`define NAK   4'b1010

`define J   2'b10 
`define K   2'b01 
`define SE0 2'b00 

`define CRC5  16'h001F
`define CRC16 16'hFFFF
`define NONE  16'h0000

`define CRC5_residue  16'h000C
`define CRC16_residue 16'h800D

`default_nettype none

module dataStream_out(clk, rst_L, encode, pkt_in, done, NRZI_active, port);

    input logic clk, rst_L, encode;
    input pkt_t pkt_in; 
    output logic done, NRZI_active;
    output logic [1:0] port;
    
    logic data_begin, stream_done, halt_stream, stream_out, CRC_out, CRC_done, stuff_out, stuff_done, NRZI_done, jump_EOP;
    logic [3:0] endp, pid;
    logic [6:0] addr; 
    logic [15:0] CRC_type;
    logic [63:0] data;
    
    
    assign pid = pkt_in.pid;
    assign endp = pkt_in.endp;
    assign addr = pkt_in.addr;
    assign data = pkt_in.data;
    assign done = NRZI_done;    

    bitStream stream(.clk(clk), 
                     .rst_L(rst_L), 
                     .encode(encode), 
                     .pid(pid), 
                     .addr(addr), 
                     .endp(endp), 
                     .data(data),
                     .halt_stream(halt_stream), 
                     .out(stream_out), 
                     .data_begin(data_begin), 
                     .stream_done(stream_done),
                     .jump_EOP(jump_EOP),
                     .CRC_type(CRC_type));
    
    CRC crc(.clk(clk), 
            .rst_L(rst_L), 
            .in(stream_out), 
            .data_begin(data_begin), 
            .stream_done(stream_done), 
            .CRC_type(CRC_type),
            .halt_stream(halt_stream), 
            .out(CRC_out), 
            .CRC_done(CRC_done));
    
    bitStuff bitstuff(.clk(clk), 
                      .rst_L(rst_L), 
                      .in(CRC_out), 
                      .data_begin(data_begin), 
                      .CRC_done(CRC_done), 
                      .out(stuff_out), 
                      .stuff_done(stuff_done), 
                      .halt_stream(halt_stream));
    
    NRZI nrzi(.clk(clk), 
              .rst_L(rst_L), 
              .in(stuff_out), 
              .out(port), 
              .encode(encode), 
              .NRZI_active(NRZI_active),
              .NRZI_done(NRZI_done), 
              .stuff_done(stuff_done),
              .jump_EOP(jump_EOP));

endmodule: dataStream_out

module bitStream (clk, rst_L, encode, pid, addr, endp, data, halt_stream, out, data_begin, stream_done, jump_EOP, CRC_type);
   
    input logic clk, rst_L, encode, halt_stream;
    input logic [3:0] endp, pid;
    input logic [6:0] addr;
    input logic [63:0] data;
    output logic out, data_begin, stream_done, jump_EOP;
    output logic [15:0] CRC_type;


    logic [7:0] count, size;
    logic [3:0] npid;
    logic [7:0] sync;
    logic [79:0] packet;
    

    assign npid = ~pid;
    assign sync = 8'b10000000;
    assign out = packet[count];

    enum logic {STANDBY = 1'b0, SEND = 1'b1} state;
   
    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) begin
            state = STANDBY;
            count = 0;
            data_begin = 0;
            stream_done = 0;
            jump_EOP = 0;
        end
        else if (~halt_stream) begin
            case(state) 
                STANDBY: begin
                    jump_EOP <= 0;
                    stream_done <= 0;
                    if (encode) begin
                        state <= SEND;
                    end
                end
                SEND: begin
                    count <= count + 1'b1;
                    if(count == size - 2'd2) begin
                        state <= STANDBY;
                        stream_done <= 1;
                        count <= 0;
                        if(CRC_type == `NONE) begin
                            jump_EOP <= 1;
                        end
                    end
                    else if (count == 8'd15 && CRC_type != `NONE) begin
                        data_begin <= 1;
                    end
                    else begin
                        data_begin <= 0;
                    end
                end                
            endcase
        end
    end
    
    always_comb begin
        case(pid) 
            `OUT: begin
                size = 7'd27;
                CRC_type = `CRC5;
                packet = {endp, addr, npid, pid, sync};
            end
            `IN: begin
                size = 7'd27;
                CRC_type = `CRC5;
                packet = {endp, addr, npid, pid, sync};
            end
            `DATA0: begin
                size = 7'd80;
                CRC_type = `CRC16;
                packet = {data, npid, pid, sync};
            end
            `ACK: begin
                size = 7'd16;
                CRC_type = `NONE;
                packet = {npid, pid, sync};
            end
            `NAK: begin
                size = 7'd16;
                CRC_type = `NONE;
                packet = {npid, pid, sync};
            end
            default: begin
                size = 7'd0;
                CRC_type = `NONE;
                packet = 0;
            end
        endcase
    end
    
endmodule: bitStream

module CRC(clk, rst_L, in, data_begin, stream_done, CRC_type, halt_stream, out, CRC_done);
    
    input logic in, clk, rst_L, data_begin, stream_done, halt_stream;
    input logic [15:0] CRC_type;
    output logic out, CRC_done;
   
    logic [4:0] count, size;
    logic [15:0] in_flop;
    
    enum logic [1:0] {STANDBY = 2'b00, CRC = 2'b01, SEND = 2'b10} state;
    
    assign out = (state == SEND) ? ~in_flop[count] : in;
   
    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) begin
            in_flop = `CRC16;
            count = 0;
            CRC_done = 0;
            state = STANDBY; 	
        end
        else if (~halt_stream) begin
            case(state) 
                STANDBY: begin
                    CRC_done <= 0;
                    if(data_begin) begin
                        state <= CRC;
                        count <= size - 1'b1;
                        if(CRC_type == `CRC5) begin
                            in_flop[0] <= in_flop[4] ^ in;
                            in_flop[1] <= in_flop[0];
                            in_flop[2] <= in_flop[1] ^ (in_flop[4] ^ in);
                            in_flop[3] <= in_flop[2];
                            in_flop[4] <= in_flop[3];
                        end
                        else if(CRC_type == `CRC16) begin
                            in_flop[0] <= in_flop[15] ^ in;
                            in_flop[1] <= in_flop[0];
                            in_flop[2] <= in_flop[1] ^ (in_flop[15] ^ in);
                            in_flop[3] <= in_flop[2];
                            in_flop[4] <= in_flop[3];
                            in_flop[5] <= in_flop[4];
                            in_flop[6] <= in_flop[5];
                            in_flop[7] <= in_flop[6];
                            in_flop[8] <= in_flop[7];
                            in_flop[9] <= in_flop[8];
                            in_flop[10] <= in_flop[9];
                            in_flop[11] <= in_flop[10];
                            in_flop[12] <= in_flop[11];
                            in_flop[13] <= in_flop[12];
                            in_flop[14] <= in_flop[13];
                            in_flop[15] <= in_flop[14] ^ (in_flop[15] ^ in);
                        end
                    end   
                end
                CRC: begin
                    if(CRC_type == `CRC5) begin
                        in_flop[0] <= in_flop[4] ^ in;
                        in_flop[1] <= in_flop[0];
                        in_flop[2] <= in_flop[1] ^ (in_flop[4] ^ in);
                        in_flop[3] <= in_flop[2];
                        in_flop[4] <= in_flop[3];
                    end
                    else if(CRC_type == `CRC16) begin
                        in_flop[0] <= in_flop[15] ^ in;
                        in_flop[1] <= in_flop[0];
                        in_flop[2] <= in_flop[1] ^ (in_flop[15] ^ in);
                        in_flop[3] <= in_flop[2];
                        in_flop[4] <= in_flop[3];
                        in_flop[5] <= in_flop[4];
                        in_flop[6] <= in_flop[5];
                        in_flop[7] <= in_flop[6];
                        in_flop[8] <= in_flop[7];
                        in_flop[9] <= in_flop[8];
                        in_flop[10] <= in_flop[9];
                        in_flop[11] <= in_flop[10];
                        in_flop[12] <= in_flop[11];
                        in_flop[13] <= in_flop[12];
                        in_flop[14] <= in_flop[13];
                        in_flop[15] <= in_flop[14] ^ (in_flop[15] ^ in);
                    end
                    if(stream_done && CRC_type != `NONE) begin
                        state <= SEND;
                    end
                    else if(stream_done && CRC_type == `NONE) begin
                        state <= STANDBY;
                        CRC_done <= 1;
                    end
                end
                SEND: begin
                    count <= count - 1'b1;
                    if(count <= 0) begin
                        state <= STANDBY;
                        CRC_done <= 1;
                    end
                end
            endcase
        end
    end
    
    always_comb begin
        case(CRC_type)
            `CRC5: begin
                size = 5;
            end
            `CRC16: begin
                size = 16;
            end
            `NONE: begin
                size = 0;
            end
            default: begin
                size = 0;
            end
        endcase
    end
    
endmodule: CRC

module bitStuff(clk, rst_L, in, data_begin, CRC_done, out, stuff_done, halt_stream);
    
    input logic clk, rst_L, in, data_begin, CRC_done;
    output logic out, stuff_done, halt_stream;

    enum logic [1:0] {STANDBY = 2'b00, SEND = 2'b01, STUFF = 2'b10} state;

    logic delay_flag;
    logic [2:0] count;
    
    assign out = (state == STUFF) ? 1'b0 : in;

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            count = 0;
            state = STANDBY;
            halt_stream = 0;
            delay_flag = 0;
        end

        else begin
            case(state) 
                STANDBY: begin
                    delay_flag <= 0;
                    if(data_begin) begin
                        state <= SEND;
                    end
                end
                SEND: begin
                    if(count == 5 && in == 1) begin
                        state <= STUFF;
                        halt_stream <= 1;
                        count <= 0;
                        if(CRC_done)begin
                            delay_flag <= 1;
                        end                        
                    end
                    else if(CRC_done) begin
                        state <= STANDBY;
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
                    delay_flag <= 0;
                    halt_stream <= 0;
                    if(CRC_done) begin
                        state <= STANDBY;
                    end
                    else begin
                        state <= SEND;
                    end
                end
            endcase
        end
    end

    always_comb begin
        if(CRC_done && !(count == 5 && in)) begin
            stuff_done = 1;
        end
        else if(delay_flag) begin
            stuff_done = 1;
        end
        else begin
            stuff_done = 0;
        end
    end

    
endmodule: bitStuff

module NRZI(clk, rst_L, in, out, encode, NRZI_active, NRZI_done, stuff_done, jump_EOP);
    input logic clk, rst_L, in, encode, stuff_done, jump_EOP;
    output logic NRZI_active, NRZI_done;
    
    enum logic [1:0] {STANDBY = 2'b00, SEND = 2'b01, EOP = 2'b10} state;
    output logic [1:0] out;
    
    logic [1:0] EOP_count, prev;
   
    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin
            state = STANDBY;
            NRZI_done = 0;
            NRZI_active = 0;
            EOP_count = 0;
            prev = `J;
        end
        
        else begin
            case(state)
                STANDBY: begin
                    NRZI_done <= 0;
                    if(encode) begin
                        state <= SEND;
                        NRZI_active <= 1;
                    end
                end
                SEND: begin
                    prev <= out;
                    if(stuff_done || jump_EOP) begin
                        state <= EOP;
                        EOP_count <= 1;
                    end
                end
                EOP: begin
                    if(EOP_count == 2) begin
                        state <= STANDBY;
                        EOP_count <= 1'b0;
                        NRZI_done <= 1;
                        NRZI_active <= 0;
                    end
                    else begin
                        EOP_count <= EOP_count + 1'b1;
                    end
                end
            endcase
        end
    end
    
    always_comb begin
        if(NRZI_active) begin
            if(stuff_done || jump_EOP) begin
                out = `SE0;
            end
            else if(state == EOP && EOP_count == 1) begin
                out = `SE0;
            end
            else if(state == EOP && EOP_count == 2) begin
                out = `J;
            end
            else begin
                out = (in) ? prev : ~prev;
            end
        end
        else begin
            out = 2'bzz;
        end
    end
    
endmodule: NRZI