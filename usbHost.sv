// Write your usb host here.  Do not modify the port list.

module usbHost
    (input logic clk, rst_L,
     usbWires wires);

    /* Tasks needed to be finished to run testbenches */
    
    logic encode, stream_begin, stream_done, halt_stream, stream_out, CRC_out, CRC_done, stuff_out, stuff_done, NRZI_done, NRZI_begin;
    logic [1:0] port;
    logic [3:0] pid, endp;
    logic [6:0] addr; 

    task prelabRequest();
    // sends an OUT packet with ADDR=5 and ENDP=4
    // packet should have SYNC and EOP too
        pid     <= 4'b0001;
        addr    <= 7'b0000101;
        endp    <= 4'b0100;
        encode <= 1;
        
        @(posedge clk);
        encode <= 0;
        
        wait(NRZI_done);

    endtask: prelabRequest

    task readData
    // host sends memPage to thumb drive and then gets data back from it
    // then returns data and status to the caller
    (input  bit [15:0] mempage, // Page to write
     output bit [63:0] data, // array of bytes to write
     output bit        success);

    endtask: readData

    task writeData
    // Host sends memPage to thumb drive and then sends data
    // then returns status to the caller
    (input  bit [15:0] mempage, // Page to write
     input  bit [63:0] data, // array of bytes to write
     output bit        success);

    endtask: writeData

    // usbHost starts here!!
    
    
    bitStream stream(.clk(clk), 
                     .rst_L(rst_L), 
                     .encode(encode), 
                     .pid(pid), 
                     .addr(addr), 
                     .endp(endp), 
                     .halt_stream(halt_stream), 
                     .out(stream_out), 
                     .NRZI_begin(NRZI_begin), 
                     .stream_begin(stream_begin), 
                     .stream_done(stream_done));
    
    CRC crc(.clk(clk), 
            .rst_L(rst_L), 
            .in(stream_out), 
            .stream_begin(stream_begin), 
            .stream_done(stream_done), 
            .halt_stream(halt_stream), 
            .out(CRC_out), 
            .CRC_done(CRC_done));
    
    bitStuff bitstuff(.clk(clk), 
                      .rst_L(rst_L), 
                      .in(CRC_out), 
                      .stream_begin(stream_begin), 
                      .CRC_done(CRC_done), 
                      .out(stuff_out), 
                      .stuff_done(stuff_done), 
                      .halt_stream(halt_stream));
    
    NRZI nrzi(.clk(clk), 
              .rst_L(rst_L), 
              .in(stuff_out), 
              .out(port), 
              .NRZI_begin(NRZI_begin), 
              .NRZI_done(NRZI_done), 
              .stuff_done(stuff_done));
    
    assign wires.DP = port[1];
    assign wires.DM = port[0];

endmodule: usbHost

module bitStream (clk, rst_L, encode, pid, addr, endp, halt_stream, out, NRZI_begin, stream_begin, stream_done);
   
    input logic clk, rst_L, encode, halt_stream;
    input logic [3:0] pid, endp;
    input logic [6:0] addr;
    output logic out, NRZI_begin, stream_begin, stream_done;

    logic [7:0] count, size;
    logic [3:0] npid;
    logic [7:0] sync;
    logic [26:0] packet;
    

    assign npid = ~pid;
    assign sync = 8'b10000000;
    assign size = 7'd27;
    assign out = packet[0];

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
                        packet <= {endp, addr, npid, pid, sync};
                        count <= 1'b1;
                    end
                end
                SEND: begin
                    NRZI_begin <= 0;
                    packet <= packet >> 1;
                    count <= count + 1'b1;
                    if(count == size - 1'b1) begin
                        state <= STANDBY;
                        stream_done <= 1;
                        count <= 0;
                    end
                    else if (count == 8'd15) begin
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

module CRC(in, clk, rst_L, stream_begin, stream_done, halt_stream, out, CRC_done);
    
    input logic in, clk, rst_L, stream_begin, stream_done, halt_stream;
    output logic out, CRC_done;
   
    logic [2:0] count;
    logic [4:0] in_flop;
    
    enum logic [1:0] {STANDBY = 2'b00, CRC = 2'b01, SEND = 2'b10} state;
    
    assign out = (state == SEND) ? ~in_flop[4] : in;
   
    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) begin
            in_flop = 5'h1F;
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
                        in_flop[2] <= in_flop[1] ^ (in_flop[4] ^ in);
                        in_flop[3] <= in_flop[2];
                        in_flop[4] <= in_flop[3];
                    end   
                end
                CRC: begin
                    in_flop[0] <= in_flop[4] ^ in;
                    in_flop[1] <= in_flop[0];
                    in_flop[2] <= in_flop[1] ^ (in_flop[4] ^ in);
                    in_flop[3] <= in_flop[2];
                    in_flop[4] <= in_flop[3];
                    if(stream_done) begin
                        state <= SEND;
                    end
                end
                SEND: begin
                    $display ("Remainder: %b", in_flop);
                    in_flop <= in_flop << 1;
                    count <= count + 1'b1;
                    if(count == 3) begin
                        state <= STANDBY;
                        CRC_done <= 1;
                    end
                end
            endcase
        end
    end
    
endmodule: CRC

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
                        out <= SE0;
                        EOP_count <= 1;
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






