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

/*SYNC*/
`define SYNC 8'b10000000

/*CRC Definitions*/
`define CRC5  16'h001F
`define CRC16 16'hFFFF
`define NONE  16'h0000

/*Residue Definitions*/
`define CRC5_residue  16'h000C
`define CRC16_residue 16'h800D

//nettype for debugging DO NOT INCLUDE IN SIMULATION
//`default_nettype none


typedef struct packed {
  logic [3:0] pid;
  logic [3:0] endp;
  logic [6:0] addr;
  logic [63:0] data;
} pkt_t;

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
    
    assign done = stream_done; //link done signal
    
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
    
    enum logic [1:0] {STANDBY = 2'b00, SEND = 2'b01, EOP = 2'b10} state; //state enumeration
    
    always_ff @(posedge clk, negedge rst_L, posedge kill) begin
        if(~rst_L | kill) begin //reset/kill state
            prev = `J;
            EOP_count = 0;
            EOP_error = 0;
            NRZI_active = 0;
            state = STANDBY;
        end
        
        else begin
            case(state)
                STANDBY: begin //standing by for decoding signal
                    EOP_error <= 0;
                    if(decode) begin
                        state <= SEND;
                        NRZI_active <= 1; //NRZI activated
                    end
                end
                SEND: begin //sending 
                    prev <= in; //reset previous state
                    if(in == `K) begin
                        out <= (prev == `J) ? 1'b0 : 1'b1; //output logic for K
                    end
                    else if(in == `SE0) begin //spot EOP
                        state <= EOP;
                        EOP_count <= 2'd1;
                    end
                    else begin
                        out <= (prev == `J) ? 1'b1 : 1'b0; //output logic for J
                    end
                end
                EOP: begin //EOP confirmation
                    if(EOP_count == 1 && in == `SE0) begin
                        EOP_count <= 2'd2;
                    end
                    else if(EOP_count == 2 && in == `J) begin
                        EOP_count <= 2'd0;
                        state <= STANDBY;
                        NRZI_active <= 0;
                    end
                    else begin
                        EOP_error <= 1; //flag incorrect EOP
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
    enum logic [1:0] {STANDBY = 2'b00, SEND = 2'b01, UNSTUFF = 2'b10} state; //state enumeration
    
    assign out = in; //direct in to out
    
    always_ff @(posedge clk, negedge rst_L, posedge kill) begin
        if(~rst_L | kill) begin //reset/kill state
            halt_stream = 0;
            count = 0;
            state = STANDBY;
        end
        
        else begin
            case(state)
                STANDBY: begin //waiting for data to start
                    if(data_begin) begin
                        state <= SEND;
                    end
                end
                SEND: begin //waiting for stuffed bit
                    if(data_done) begin //return to standby at the end of the stream
                        state <= STANDBY;
                    end
                    else if (count == 3'd5 && in == 1) begin //unstuff once six ones are seen
                        state <= UNSTUFF;
                        halt_stream <= 1; //halt stream operations
                        count <= 0;
                    end   
                    else if(in) begin
                        count <= count + 3'd1;//increment count
                    end
                    else begin
                        count <= 0;//reset count on 1
                    end                    
                end
                UNSTUFF: begin //unstuff state 
                    halt_stream <= 0; //reset the halted stream
                    if(data_done) begin //go to standby on data_done
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
    
    enum logic [1:0] {STANDBY = 2'b00, CRC = 2'b01} state; //state enumeration
    
    assign out = in; //direct in to out
    
    always_ff @(posedge clk, negedge rst_L, posedge kill) begin
        if(~rst_L | kill) begin //reset/kill state
            state = STANDBY;
            in_flop = 16'hFFFF;
        end
        
        else if (~halt_stream) begin //only calculate if stream not halted
            case(state)
                STANDBY: begin //waiting for data to begin
                    if(data_begin) begin
                        state <= CRC;
                    end
                end
                CRC: begin
                    if(CRC_type == `CRC5) begin //CRC5 calculation
                        in_flop[0] <= in_flop[4] ^ in;
                        in_flop[1] <= in_flop[0];
                        in_flop[2] <= in_flop[1] ^ (in_flop[4] ^ in);
                        in_flop[3] <= in_flop[2];
                        in_flop[4] <= in_flop[3];
                    end
                    else if(CRC_type == `CRC16) begin //CRC16 calculation
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
                        state <= STANDBY; //return to standby at the end of the data
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
    
    enum logic [2:0] {STANDBY = 3'b000, SYNC = 3'b001, PID = 3'b010, PACK = 3'b11, CRC = 3'b100, SEND = 3'b101} state; //state enumeration
    
    always_ff @(posedge clk, negedge rst_L, posedge kill) begin
        if(~rst_L | kill) begin //reset/kill state
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
        
        else if (~halt_stream) begin //only process stream if not halted
            if(EOP_error) begin //set error flag if EOP error reported from NRZI
                error <= 1;
            end
            case(state)
                STANDBY: begin //waiting for decode signal
                    pkt_out <= 0;
                    error <= 0;
                    stream_done <= 0;
                    watch <= 0;
                    pid <= 0;
                    residue <= 0;
                    if(decode) begin //begin decoding
                        state <= SYNC;
                    end
                end
                SYNC: begin //watch for SYNC
                    if(watch >= 10'd7 && in)begin //watch for seven 0s and a 1
                        state <= PID; //head to PID state
                        watch <= 0;
                    end
                    else if(~in) begin //increment watch on 0
                        watch <= watch + 10'd1;
                    end
                    else begin //reset watch on 1
                        watch <= 0;
                    end
                end
                PID: begin //Pack PID
                    pid[PID_count] <= in; //fill PID with in 
                    PID_count <= PID_count + 3'd1; //increment PID counter
                    if(PID_count == 3'd7 && CRC_type == `NONE) begin //if no CRC (ACK/NAK)
                        state <= SEND; //skip to SEND stage, no data
                        pkt_out.pid <= pid[3:0]; //put PID into packet
                        PID_count <= 0; //reset count
                    end
                    else if(PID_count == 3'd7 && CRC_type != `NONE) begin
                        state <= PACK; //head to PACK state
                        pkt_out.pid <= pid[3:0]; //put PID into packet
                        data_begin <= 0; //reset data_begin signal
                        PID_count <= 0; //reset count
                    end
                    else if(PID_count == 3'd6 && CRC_type != `NONE) begin
                        data_begin <= 1; //indicate that data is beginning
                    end
                end
                PACK: begin //PACK addr/endp/data
                    data_begin <= 0; //reset data flag
                    if(CRC_type == `CRC5) begin //if CRC5
                        if(count <= 7'd7) begin //addr stage
                            pkt_out.addr[count] <= in;
                        end
                        else begin //endp stage
                            pkt_out.endp[count-7'd8] <= in;
                        end
                    end
                    else if(CRC_type == `CRC16) begin //if CRC16
                        pkt_out.data[count] <= in; //add in to data
                    end
                    count <= count + 7'd1; //increment count
                    if(count == size - 1'd1) begin
                        state <= CRC; //head to CRC stage
                        count <= 0; //reset count
                    end
                end
                CRC: begin //record CRC residue
                    residue[CRC_count] <= in; //put in in residue
                    CRC_count <= CRC_count + 1'b1; //increment CRC_count
                    if(CRC_count == CRC_size - 1) begin
                        state <= SEND; //head to SEND
                        CRC_count <= 0; //reset CRC_count
                        data_done <= 1; //indicate to upstream that data is done 
                    end
                end
                SEND: begin //send packet and error signals
                    if(PID_error) begin //throw error if unrecognized PID
                        error <= 1;
                    end
                    else if(pid[3:0] != ~pid[7:4])begin //throw error if PID and NPID aren't complements
                        error <= 1;
                    end
                    else if(CRC_type == `CRC5 && residue != `CRC5_residue) begin //throw error if CRC5 residue doesn't match
                        error <= 1;
                    end
                    else if(CRC_type == `CRC16 && residue != `CRC16_residue) begin //throw error if CRC16 residue doesn't match
                        error <= 1;
                    end
                    stream_done <= 1; //put out done signal
                    state <= STANDBY; //return to standby
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
