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

//`default_nettype none

typedef struct packed {
  logic [3:0] pid;
  logic [3:0] endp;
  logic [6:0] addr;
  logic [63:0] data;
} pkt_t;

/******************************************************************************
// dataStream_out
//*****************************************************************************
// clk              (input) - The clock
// rst_L            (input) - Reset (asserted low)
// encode           (input) - Start command to encode NRZI signals
// pkt_in           (input) - Data packet in
// done             (output)- Done indicating that packet has been sent
// NRZI_active      (output)- Activity flag controlling port lines
// port             (output)- Input from USB ports
*/
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
    
    
    assign pid = pkt_in.pid; //link packet pieces
    assign endp = pkt_in.endp; 
    assign addr = pkt_in.addr;
    assign data = pkt_in.data;
    assign done = NRZI_done;   //link done signal 

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

/******************************************************************************
// bitStream
//*****************************************************************************
// clk              (input) - The clock
// rst_L            (input) - Reset (asserted low)
// encode           (input) - Start command to encode NRZI signals
// pid              (input) - PID fromt the packet
// addr             (input) - Address from the packet
// endp             (input) - Endpoint from the packet
// data             (input) - Data from the packet
// halt_stream      (input) - Signal from stuffer to pause for stuff
// out              (output)- Datastream out
// data_begin       (output)- Signal to downstream modules that data is beginning
// stream_done      (output)- Done indicating that packet has been sent
// jump_EOP         (output)- Signal to NRZI to jump directly to EOP without waiting for stuffer
// CRC_type         (output)- Signal telling CRC what type to use based on PID
*/
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
    

    assign npid = ~pid; //set NPID to complement of PID
    assign out = packet[count]; //assign out to index count of packet

    enum logic {STANDBY = 1'b0, SEND = 1'b1} state;
   
    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) begin //reset case
            state = STANDBY;
            count = 0;
            data_begin = 0;
            stream_done = 0;
            jump_EOP = 0;
        end
        
        else if (~halt_stream) begin //don't increment on halt_stream
            case(state) 
                STANDBY: begin //waiting for encode
                    jump_EOP <= 0; //reset jump_EOP signal
                    stream_done <= 0; //reset done signal
                    if (encode) begin
                        state <= SEND;
                    end
                end
                SEND: begin //sending packet
                    count <= count + 1'b1; //increment count
                    if(count == size - 2'd2) begin //reached end of packet
                        state <= STANDBY; 
                        stream_done <= 1; //set done flag
                        count <= 0; //reset count
                        if(CRC_type == `NONE) begin //tell NRZI to proceed to EOP without flag from stuffer if no CRC
                            jump_EOP <= 1; 
                        end
                    end
                    else if (count == 8'd14 && CRC_type != `NONE) begin //set data flag for CRC5 and CRC16 packets
                        data_begin <= 1;
                    end
                    else begin
                        data_begin <= 0; //reset data flag
                    end
                end                
            endcase
        end
    end
    
    always_comb begin
        case(pid) //calculate packet parameters based on PID
            `OUT: begin
                size = 7'd27;
                CRC_type = `CRC5;
                packet = {endp, addr, npid, pid, `SYNC};
            end
            `IN: begin
                size = 7'd27;
                CRC_type = `CRC5;
                packet = {endp, addr, npid, pid, `SYNC};
            end
            `DATA0: begin
                size = 7'd80;
                CRC_type = `CRC16;
                packet = {data, npid, pid, `SYNC};
            end
            `ACK: begin
                size = 7'd16;
                CRC_type = `NONE;
                packet = {npid, pid, `SYNC};
            end
            `NAK: begin
                size = 7'd16;
                CRC_type = `NONE;
                packet = {npid, pid, `SYNC};
            end
            default: begin //default if unrecognised
                size = 7'd0;
                CRC_type = `NONE;
                packet = 0;
            end
        endcase
    end
    
endmodule: bitStream


/******************************************************************************
// CRC
//*****************************************************************************
// clk              (input) - The clock
// rst_L            (input) - Reset (asserted low)
// in               (input) - Start command to encode NRZI signals
// data_begin       (input) - Signal from bitstream that data is beginning
// stream_done      (input) - Done indicating that packet has been sent
// CRC_type         (input) - Signal telling CRC what type to use based on PID
// halt_stream      (input) - Signal from stuffer to pause for stuff
// out              (output)- Datastream out
// CRC_done         (output)- Signal to stuffer that CRC has finished
*/
module CRC(clk, rst_L, in, data_begin, stream_done, CRC_type, halt_stream, out, CRC_done);
    
    input logic in, clk, rst_L, data_begin, stream_done, halt_stream;
    input logic [15:0] CRC_type;
    output logic out, CRC_done;
   
    logic [4:0] count, size;
    logic [15:0] in_flop;
    
    enum logic [1:0] {STANDBY = 2'b00, CRC = 2'b01, SEND = 2'b10} state; //state enumeration
    
    assign out = (state == SEND) ? ~in_flop[count] : in; //output either direct in or complemented remainder depending on state
   
    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) begin //reset state
            in_flop = `CRC16;
            count = 0;
            CRC_done = 0;
            state = STANDBY; 	
        end
        
        else if (~halt_stream) begin //do not calculate on halted stream
            case(state) 
                STANDBY: begin //waiting for data to begin
                    CRC_done <= 0; //reset done flag
                    if(data_begin) begin
                        state <= CRC;
                        count <= size - 1'b1; //set count based on PID                    
                    end   
                end
                CRC: begin //calculating CRC
                    if(CRC_type == `CRC5) begin //CRC5 calculation
                        in_flop[0] <= in_flop[4] ^ in;
                        in_flop[1] <= in_flop[0];
                        in_flop[2] <= in_flop[1] ^ (in_flop[4] ^ in);
                        in_flop[3] <= in_flop[2];
                        in_flop[4] <= in_flop[3];
                    end
                    else if(CRC_type == `CRC16) begin //CRC 16 calculation
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
                    if(stream_done && CRC_type != `NONE) begin //proceed to SEND if CRC enabled
                        state <= SEND;
                    end
                    else if(stream_done && CRC_type == `NONE) begin //return to standby otherwise
                        state <= STANDBY;
                        CRC_done <= 1; //send done flag
                    end
                end
                SEND: begin //sending remainder
                    count <= count - 1'b1; //decrement CRC count
                    if(count <= 0) begin //reset once at the last bit
                        state <= STANDBY;
                        CRC_done <= 1;
                    end
                end
            endcase
        end
    end
    
    always_comb begin
        case(CRC_type) //calculate size based on type
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

/******************************************************************************
// bitStuff
//*****************************************************************************
// clk              (input) - The clock
// rst_L            (input) - Reset (asserted low)
// in               (input) - Start command to encode NRZI signals
// data_begin       (input) - Signal from bitstream that data is beginning
// CRC_done         (input) - Done indicating that CRC has finished sending
// out              (output)- Datastream out
// stuff_done       (output)- Signal to NRZI to tell it to send EOP
// halt_stream      (output)- Signal from stuffer to pause for stuff
*/
module bitStuff(clk, rst_L, in, data_begin, CRC_done, out, stuff_done, halt_stream);
    
    input logic clk, rst_L, in, data_begin, CRC_done;
    output logic out, stuff_done, halt_stream;

    enum logic [1:0] {STANDBY = 2'b00, SEND = 2'b01, STUFF = 2'b10} state;

    logic delay_flag;
    logic [2:0] count;
    
    assign out = (state == STUFF) ? 1'b0 : in; //output 1 if in STUFF state, in otherwise

    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin //reset state
            count = 0;
            state = STANDBY;
            halt_stream = 0;
            delay_flag = 0;
        end

        else begin
            case(state) 
                STANDBY: begin //waiting for data
                    delay_flag <= 0; //reset delayed done flag
                    if(data_begin) begin
                        state <= SEND;
                    end
                end
                SEND: begin
                    if(count == 5 && in == 1) begin //head to stuff if six 1s
                        state <= STUFF; 
                        halt_stream <= 1; //halt stream to stuff
                        count <= 0; //reset count
                        if(CRC_done)begin
                            delay_flag <= 1; //set delay flag if CRC is also done
                        end                        
                    end
                    else if(CRC_done) begin //return to STANDBY if CRC is done
                        state <= STANDBY;
                        count <= 0;
                    end
                    else if(~in) begin //reset count on 0
                        count <= 0;
                    end
                    else begin
                        count <= count + 1'b1; //increment count otherwise
                    end
                end
                STUFF: begin //stream is being STUFFED
                    delay_flag <= 0; //reset delay flag
                    halt_stream <= 0; //reset halt signal
                    if(CRC_done | delay_flag) begin
                        state <= STANDBY; //head back to STANDBY if CRC is done
                    end
                    else begin
                        state <= SEND; //head back to SEND otherwise
                    end
                end
            endcase
        end
    end

    always_comb begin
        if(CRC_done && !(count == 5 && in)) begin
            stuff_done = 1; //indicate stuff is done if CRC is done and no stuffing
        end
        else if(delay_flag) begin
            stuff_done = 1; //indicate stuff is done on delayed flag
        end
        else begin
            stuff_done = 0; //default not done
        end
    end

endmodule: bitStuff

/******************************************************************************
// NRZI
//*****************************************************************************
// clk              (input) - The clock
// rst_L            (input) - Reset (asserted low)
// in               (input) - Start command to encode NRZI signals
// out              (output)- Output connected directly to the ports
// encode           (input) - Start command to encode NRZI signals
// NRZI_active      (input) - Activity flag controlling port lines
// NRZI_done        (output)- Flag indicating that NRZI has sent EOP
// stuff_done       (output)- Signal from stuffer to tell it to send EOP
// jump_EOP         (output)- Signal from bitStream to jump directly to EOP without waiting for stuffer
*/
module NRZI(clk, rst_L, in, out, encode, NRZI_active, NRZI_done, stuff_done, jump_EOP);
    input logic clk, rst_L, in, encode, stuff_done, jump_EOP;
    output logic NRZI_active, NRZI_done;
    output logic [1:0] out;
    
    enum logic [1:0] {STANDBY = 2'b00, SEND = 2'b01, EOP = 2'b10} state; //enumerate states
    
    logic [1:0] EOP_count, prev;
   
    always_ff @(posedge clk, negedge rst_L) begin
        if(~rst_L) begin //reset state
            state = STANDBY;
            NRZI_done = 0;
            NRZI_active = 0;
            EOP_count = 0;
            prev = `J;
        end
        
        else begin
            case(state)
                STANDBY: begin //waiting for encode signal
                    NRZI_done <= 0; //reset done flag
                    if(encode) begin
                        state <= SEND;
                        NRZI_active <= 1; //set active signal for port control
                    end
                end
                SEND: begin
                    prev <= out; //set previous to current
                    if(stuff_done || jump_EOP) begin //head to EOP if stuffer is done or get the jump signal from the stream
                        state <= EOP; 
                        EOP_count <= 1; //count goes to 1
                    end
                end
                EOP: begin
                    if(EOP_count == 2) begin //head to STANDBY once EOP is sent
                        state <= STANDBY;
                        EOP_count <= 1'b0; //reset EOP count
                        NRZI_done <= 1; //set the done flag
                        NRZI_active <= 0; //turn off the active active signal
                    end
                    else begin
                        EOP_count <= EOP_count + 1'b1; //increment EOP count
                    end
                end
            endcase
        end
    end
    
    always_comb begin
        if(NRZI_active) begin //output logic 
            if(stuff_done || jump_EOP) begin 
                out = `SE0; //put first SE0 on the line when packet is done
            end
            else if(state == EOP && EOP_count == 1) begin //put second SE0 on the line after that
                out = `SE0;
            end
            else if(state == EOP && EOP_count == 2) begin //put J on the line after both SE0
                out = `J;
            end
            else begin 
                out = (in) ? prev : ~prev; //regular output is according to project specs
            end
        end
        else begin
            out = 2'bzz; //drive tristate with z when inactive
        end
    end
    
endmodule: NRZI
