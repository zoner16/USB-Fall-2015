/*Global Definitions*/

/*Packet PIDs*/
`define OUT   4'b0001
`define IN    4'b1001
`define DATA0 4'b0011
`define ACK   4'b0010
`define NAK   4'b1010

/*Port States*/
`define J   2'b10 
`define K   2'b01 
`define SE0 2'b00 

/*CRC Definitions*/
`define CRC5  16'h001F
`define CRC16 16'hFFFF
`define NONE  16'h0000

/*Residue Definitions*/
`define CRC5_residue  16'h000C
`define CRC16_residue 16'h800D

//nettype for debugging DO NOT INCLUDE IN SIMULATION
`default_nettype none

/******************************************************************************
// dataStream_in
//*****************************************************************************
// clk              (input) - The clock
// rst_L            (input) - Reset (asserted low)
// kill             (input) - Kill switch for timeouts
// decode           (input) - Start command to decode NRZI signals
// port             (input) - Input from USB ports
// pkt_out          (output)- Data packet out
// done             (output)- Done indicating that packet is ready
// NRZI_active      (output)- Activity flag controlling port lines
// error            (output)- Error flag for CRC or EOP errors
*/
module dataStream_in (clk, rst_L, kill, decode, port, pkt_out, done, NRZI_active, error);
    
    input logic clk, rst_L, kill, decode;
    input logic [1:0] port;
    output pkt_t pkt_out;
    output logic done, NRZI_active, error;
    
    logic NRZI_out, EOP_error, data_begin, data_done, unstuff_out, halt_stream, CRC_out, stream_done;
    logic [15:0] CRC_type;
    
    assign done = stream_done;
    
    NRZI_in nrzi(.clk(clk), 
                 .rst_L(rst_L), 
                 .kill(kill),
                 .decode(decode), 
                 .in(port), 
                 .out(NRZI_out), 
                 .EOP_error(EOP_error),
                 .NRZI_active(NRZI_active));
                 
    bitUnstuff bitunstuff(.clk(clk), 
                          .rst_L(rst_L), 
                          .kill(kill),
                          .in(NRZI_out), 
                          .data_begin(data_begin), 
                          .data_done(data_done), 
                          .out(unstuff_out), 
                          .halt_stream(halt_stream));
               
    CRC_in crc(.clk(clk), 
               .rst_L(rst_L), 
               .kill(kill),
               .in(unstuff_out), 
               .data_begin(data_begin), 
               .halt_stream(halt_stream), 
               .data_done(data_done), 
               .CRC_type(CRC_type), 
               .out(CRC_out));
               
    dataPack datapack(.clk(clk), 
                      .rst_L(rst_L), 
                      .kill(kill),
                      .decode(decode), 
                      .in(CRC_out), 
                      .halt_stream(halt_stream), 
                      .EOP_error(EOP_error), 
                      .error(error), 
                      .data_begin(data_begin), 
                      .data_done(data_done), 
                      .stream_done(stream_done), 
                      .CRC_type(CRC_type), 
                      .pkt_out(pkt_out));

endmodule: dataStream_in

/******************************************************************************
// NRZI_in
//*****************************************************************************
// clk              (input) - The clock
// rst_L            (input) - Reset (asserted low)
// kill             (input) - Kill switch for timeouts
// decode           (input) - Start command to decode NRZI signals
// in               (input) - Input from USB ports
// out              (output)- Datastream out
// EOP_error        (output)- Error in recieving EOP
// NRZI_active      (output)- Activity flag controlling port lines
*/
module NRZI_in (clk, rst_L, kill, decode, in, out, EOP_error, NRZI_active);
    input logic clk, rst_L, kill, decode;
    input logic [1:0] in;
    output logic EOP_error, out, NRZI_active;
    
    logic [1:0] EOP_count, prev;
    
    enum logic [1:0] {STANDBY = 2'b00, SEND = 2'b01, EOP = 2'b10} state;
    
    always_ff @(posedge clk, negedge rst_L, negedge kill) begin
        if(~rst_L | ~kill) begin
            prev = `J;
            EOP_count = 0;
            EOP_error = 0;
            NRZI_active = 0;
            state = STANDBY;
        end
        else begin
            case(state)
                STANDBY: begin
                    EOP_error <= 0;
                    if(decode) begin
                        state <= SEND;
                        NRZI_active <= 1;
                    end
                end
                SEND: begin
                    prev <= in;
                    if(in == `K) begin
                        out <= (prev == `J) ? 1'b0 : 1'b1;
                    end
                    else if(in == `SE0) begin
                        state <= EOP;
                        EOP_count <= 2'd1;
                    end
                end
                EOP: begin
                    if(EOP_count == 1 && in == `SE0) begin
                        EOP_count <= 2'd2;
                    end
                    else if(EOP_count == 2 && in == `J) begin
                        EOP_count <= 2'd0;
                        state <= STANDBY;
                        NRZI_active <= 0;
                    end
                    else begin
                        EOP_error <= 1;
                        EOP_count <= 2'd0;
                        state <= STANDBY;
                        NRZI_active <= 0;
                    end
                end
            endcase
        end
        
    end
    
endmodule: NRZI_in

/******************************************************************************
// bitUnstuff
//*****************************************************************************
// clk              (input) - The clock
// rst_L            (input) - Reset (asserted low)
// kill             (input) - Kill switch for timeouts
// in               (input) - Input from datastream
// data_begin       (input) - Signal from the packer that the stuffed data is inbound
// data_done        (input) - Signal from the packer that the stuffed data is done
// out              (output)- Datastream out
// halt_stream      (output)- Signal to downstream modules to stop for unstuff
*/
module bitUnstuff (clk, rst_L, kill, in, data_begin, data_done, out, halt_stream);
    input logic clk, rst_L, kill, in, data_begin, data_done;
    output logic out, halt_stream;
    
    logic [2:0] count;
    enum logic [1:0] {STANDBY = 2'b00, SEND = 2'b01, UNSTUFF = 2'b10} state;
    
    assign out = in;
    
    always_ff @(posedge clk, negedge rst_L, negedge kill) begin
        if(~rst_L | ~kill) begin
            halt_stream = 0;
            count = 0;
            state = STANDBY;
        end
        else begin
            case(state)
                STANDBY: begin
                    if(data_begin) begin
                        state <= SEND;
                    end
                end
                SEND: begin
                    if(in) begin
                        count <= count + 3'd1;
                    end
                    else begin
                        count <= 0;
                    end
                    
                    if(data_done) begin
                        state <= STANDBY;
                    end
                    else if (count == 3'd5 && in == 1) begin
                        state <= UNSTUFF;
                        count <= 0;
                        halt_stream <= 1;
                    end                
                end
                UNSTUFF: begin
                    halt_stream <= 0;
                    if(data_done) begin
                        state <= STANDBY;
                    end
                    else begin
                        state <= SEND;
                    end
                end
            endcase
        end
    end
    
endmodule: bitUnstuff

/******************************************************************************
// CRC_in
//*****************************************************************************
// clk              (input) - The clock
// rst_L            (input) - Reset (asserted low)
// kill             (input) - Kill switch for timeouts
// in               (input) - Input from datastream
// data_begin       (input) - Signal from the packer that the CRC'd data is inbound
// halt_stream      (input) - Signal from unstuffer to stop for unstuff
// data_done        (input) - Signal from the packer that the stuffed data is done
// CRC_type         (input) - CRC type determined by packer upon recieving PID
// out              (output)- Datastream out
*/
module CRC_in (clk, rst_L, kill, in, data_begin, halt_stream, data_done, CRC_type, out);
    input logic clk, rst_L, kill, in, data_begin, halt_stream, data_done; 
    input logic [15:0] CRC_type;
    output logic out;

    logic [15:0] in_flop;
    
    enum logic [1:0] {STANDBY = 2'b00, CRC = 2'b01} state;
    
    assign out = in;
    
    always_ff @(posedge clk, negedge rst_L, negedge kill) begin
        if(~rst_L | ~kill) begin
            state = STANDBY;
        end
        else if (~halt_stream) begin
            case(state)
                STANDBY: begin
                    if(data_begin) begin
                        state <= CRC;
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
                    
                    if(data_done) begin
                        state <= STANDBY;
                    end
                end
            endcase
        end
    end
    
endmodule: CRC_in

/******************************************************************************
// dataPack
//*****************************************************************************
// clk              (input) - The clock
// rst_L            (input) - Reset (asserted low)
// kill             (input) - Kill switch for timeouts
// decode           (input) - Start command to decode NRZI signals
// in               (input) - Input from datastream
// halt_stream      (input) - Signal from unstuffer to stop for unstuff
// error            (output)- Error flag for CRC or EOP errors
// data_begin       (output)- Signal to upstream modules that data is beginning
// data_done        (output)- Signal to upstream modules that data is done
// stream_done      (output)- Done indicating that packet is ready
// CRC_type         (output)- CRC type determined by packer upon recieving PID
// pkt_out          (output)- Data packet out
*/
module dataPack (clk, rst_L, kill, decode, in, halt_stream, EOP_error, error, data_begin, data_done, stream_done, CRC_type, pkt_out);
    input logic clk, rst_L, kill, decode, in, halt_stream, EOP_error;
    output logic error, data_begin, data_done, stream_done;
    output logic [15:0] CRC_type;
    output pkt_t pkt_out;

    logic PID_error;
    logic [2:0] PID_count;
    logic [4:0] CRC_count, CRC_size;
    logic [7:0] pid, count, size; 
    logic [9:0] watch;
    logic [15:0] residue;
    
    enum logic [2:0] {STANDBY = 3'b000, SYNC = 3'b001, PID = 3'b010, PACK = 3'b11, CRC = 3'b100, SEND = 3'b101} state;
    
    always_ff @(posedge clk, negedge rst_L, negedge kill) begin
        if(~rst_L | ~kill) begin
            watch = 0;
            count = 0;
            pid = 0;
            residue = 0;
            PID_count = 0;
            CRC_count = 0;
            error = 0;
            data_begin = 0;
            stream_done = 0;
            pkt_out = 0;
            state = STANDBY;
        end
        else if (~halt_stream) begin
            if(EOP_error) begin
                error <= 1;
            end
            case(state)
                STANDBY: begin
                    error <= 0;
                    stream_done <= 0;
                    watch <= 0;
                    pid <= 0;
                    residue <= 0;
                    if(decode) begin
                        state <= SYNC;
                    end
                end
                SYNC: begin
                    if(watch >= 10'd7 && in)begin
                        state <= PID;
                    end
                    else if(~in) begin
                        watch <= watch + 10'd1;
                    end
                    else begin
                        watch <= 0;
                    end
                end
                PID: begin
                    pid[PID_count] <= in;
                    PID_count <= PID_count + 3'd1;
                    if(PID_count == 3'd7 && CRC_type == `NONE) begin
                        state <= SEND;
                        PID_count <= 0;
                    end
                    else if(PID_count == 3'd7 && CRC_type != `NONE) begin
                        state <= PACK;
                        PID_count <= 0;
                        data_begin <= 1;
                    end
                end
                PACK: begin
                    data_begin <= 0;
                    pkt_out[count] <= in;
                    count <= count + 7'd1;
                    if(count == size - 1'd1) begin
                        state <= CRC;
                        count <= 0;
                    end
                end
                CRC: begin
                    residue[CRC_count] <= in;
                    if(CRC_count == CRC_size - 1) begin
                        state <= SEND;
                        CRC_count <= 0;
                        data_done <= 1;
                    end
                end
                SEND: begin
                    if(PID_error) begin
                        error <= 1;
                    end
                    else if(CRC_type == `CRC5 && residue != `CRC5_residue) begin
                        error <= 1;
                    end
                    else if(CRC_type == `CRC16 && residue != `CRC16_residue) begin
                        error <= 1;
                    end
                    stream_done <= 1;
                    state <= STANDBY;
                end
            endcase
        end
    end
    
    always_comb begin
        case(pid[3:0])
            `OUT: begin
                CRC_type = `CRC5;
                size = 8'd11;
                CRC_size = 5'd5;
                PID_error = 0;
            end
            `IN: begin
                CRC_type = `CRC5;
                size = 8'd11;
                CRC_size = 5'd5;
                PID_error = 0;
            end
            `DATA0: begin
                CRC_type = `CRC16;
                size = 8'd64;
                CRC_size = 5'd16;
                PID_error = 0;
            end
            `ACK: begin
                CRC_type = `NONE;
                size = 8'd0;
                CRC_size = 5'd0;
                PID_error = 0;
            end
            `NAK: begin
                CRC_type = `NONE;
                size = 8'd0;
                CRC_size = 5'd0;
                PID_error = 0;
            end  
            default: begin
                CRC_type = `NONE;
                size = 8'd0;
                CRC_size = 5'd0;
                PID_error = 1;
            end
        endcase
    end
    
endmodule: dataPack
