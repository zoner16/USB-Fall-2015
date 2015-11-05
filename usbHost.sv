//`default_nettype none

`define OUT   4'b0001
`define IN    4'b1001
`define DATA0 4'b0011
`define ACK   4'b0010
`define NAK   4'b1010

`define J   2'b10 
`define K   2'b01 
`define SE0 2'b00 

`define SYNC 8'b10000000

`define CRC5  16'h001F
`define CRC16 16'hFFFF
`define NONE  16'h0000

`define CRC5_residue  16'h000C
`define CRC16_residue 16'h800D

typedef struct packed {
  logic [3:0] pid;
  logic [3:0] endp;
  logic [6:0] addr;
  logic [63:0] data;
} pkt_t;

// Write your usb host here.  Do not modify the port list.

module usbHost
    (input logic clk, rst_L,
     usbWires wires);

    /* Tasks needed to be finished to run testbenches */
    
    logic encode, decode, kill, error, in_done, out_done, NRZI_in_active, NRZI_out_active, failure, protocol_success;
    logic isValueReadCorrect, read_done, write_done, in_trans, out_trans;
    logic [1:0] port_in, port_out;
    bit read, write;
    bit [15:0] FSMmempage;
    bit [63:0] data_from_OS, data_to_OS, data_from_device, data_to_device;
    pkt_t pkt_in, pkt_out;
    bit [3:0] endp;

    task prelabRequest();
    // sends an OUT packet with ADDR=5 and ENDP=4
    // packet should have SYNC and EOP too
    /*
        pkt_in.pid     <= 4'b0001;
        pkt_in.endp    <= 4'b0100;
        pkt_in.addr    <= 7'b0000101;
        pkt_in.data    <= 64'd0;
        encode  <= 1;
        
        @(posedge clk);
        encode  <= 0;
        
        wait(out_done);
    */
    endtask: prelabRequest

    task readData
    // host sends memPage to thumb drive and then gets data back from it
    // then returns data and status to the caller
    (input  bit [15:0] mempage, // Page to write
     output bit [63:0] data, // array of bytes to write
     output bit        success);
       success <= 0;
       FSMmempage <= mempage;
       read <= 1;
       wait (read_done);
       read <= 0;
       data <= data_to_OS;
       success <= isValueReadCorrect;
       @(posedge clk);
       
    endtask: readData

    task writeData
    // Host sends memPage to thumb drive and then sends data
    // then returns status to the caller
    (input  bit [15:0] mempage, // Page to write
     input  bit [63:0] data, // array of bytes to write
     output bit        success);
       
       success <= 0;
       FSMmempage <= mempage;
       write <= 1;
       data_from_OS <= data;
       wait (write_done);
       write <= 0;
       success <= isValueReadCorrect;
       @(posedge clk);
       
    endtask: writeData

    // usbHost starts here!!
    
    readWriteFSM readwrite(.clk(clk), 
                           .rst_b(rst_L), 
                           .read(read), 
                           .write(write), 
                           .failure(failure), 
                           .success(protocol_success), 
                           .FSMmempage(FSMmempage), 
                           .data_from_OS(data_from_OS), 
                           .data_from_device(data_from_device), 
                           .data_to_device(data_to_device), 
                           .data_to_OS(data_to_OS),
                           .endp(endp), 
                           .in_trans(in_trans), 
                           .out_trans(out_trans), 
                           .isValueReadCorrect(isValueReadCorrect), 
                           .read_done(read_done),
                           .write_done(write_done));
                           
    protocolFSM protocol(.clk(clk),
                         .endp(endp),
                         .rst_b(rst_L), 
                         .in_trans(in_trans), 
                         .out_trans(out_trans), 
                         .pkt_sent(out_done), 
                         .pkt_received(in_done), 
                         .error(error),
                         .pkt_in(pkt_out), 
                         .data_from_host(data_to_device), 
                         .failure(failure), 
                         .success(protocol_success), 
                         .kill(kill), 
                         .encode(encode), 
                         .decode(decode), 
                         .pkt_out(pkt_in), 
                         .data_to_host(data_from_device));
    
    dataStream_out stream_out(.clk(clk), 
                              .rst_L(rst_L), 
                              .encode(encode), 
                              .pkt_in(pkt_in), 
                              .done(out_done), 
                              .NRZI_active(NRZI_out_active),
                              .port(port_out));    
                   
    dataStream_in stream_in(.clk(clk), 
                            .rst_L(rst_L), 
                            .kill(kill), 
                            .decode(decode), 
                            .port(port_in), 
                            .pkt_out(pkt_out), 
                            .done(in_done), 
                            .NRZI_active(NRZI_in_active),
                            .error(error));
    

    //assign wires to port pins
    assign wires.DP = (NRZI_out_active) ? port_out[1] : 1'bz;
    assign wires.DM = (NRZI_out_active) ? port_out[0] : 1'bz;

    assign port_in[0] = (NRZI_in_active) ? wires.DM : 1'bz;
    assign port_in[1] = (NRZI_in_active) ? wires.DP : 1'bz;

endmodule: usbHost

module readWriteFSM
  (input bit         clk, rst_b, read, write, failure, success,
   input bit [15:0]  FSMmempage,
   input bit [63:0]  data_from_OS, data_from_device,
   output bit [63:0] data_to_device, data_to_OS,
   output bit [3:0] endp,
   output bit        in_trans, out_trans, isValueReadCorrect, read_done, write_done);
   
    enum logic [1:0] {Hold = 2'b00, Out = 2'b01, ReadIn = 2'b10, WriteOut = 2'b11} state;
   
   
    always_ff @(posedge clk, negedge rst_b) begin
        if (~rst_b) begin
            state = Hold;
            isValueReadCorrect = 0;
            read_done = 0;
            write_done = 0;
            data_to_device = 0;
            in_trans = 0;
            out_trans = 0;
            data_to_OS = 0;
        end
        else begin
            case(state)
                Hold: begin
                    isValueReadCorrect <= 0;
                    read_done <= 0;
                    write_done <= 0;
                    data_to_device <= 0;
                    data_to_OS <= 0;
                    in_trans <= 0;
                    out_trans <= 0;
                    endp <= 4'd0;
                    if (read || write) begin
                        state <= Out;
                        out_trans <= 1;
                        data_to_device[63:48] <= FSMmempage;
                        endp <= 4'd4;
                    end
                end
                Out: begin // out transaction occurs for both read and write
                    out_trans <= 0;
                    if (success) begin
                        if (read) begin
                            state <= ReadIn;
                            in_trans <= 1;
                            endp <= 4'd8;
                        end
                        else begin
                            state <= WriteOut;
                            out_trans <= 1;
                            endp <= 4'd8;
                            data_to_device <= data_from_OS;
                        end
                    end
                    else if (failure) begin
                        state <= Hold;
                        if(read) begin
                            read_done <= 1;
                        end
                        else if(write) begin
                            write_done <= 1;
                        end
                        isValueReadCorrect <= 0;
                    end
                end
                ReadIn: begin
                    in_trans <= 0;
                    if (success) begin
                        state <= Hold;
                        read_done <= 1;
                        isValueReadCorrect <= 1;
                        data_to_OS <= data_from_device;
                    end
                    else if (failure) begin
                        state <= Hold;
                        read_done <= 1;
                        isValueReadCorrect <= 0;
                    end
                end // case: ReadIn
                WriteOut: begin
                    out_trans <= 0;
                    if (success) begin
                        state <= Hold;
                        write_done <= 1;
                        isValueReadCorrect <= 1;
                    end
                    else if (failure) begin
                        state <= Hold;
                        write_done <= 1;
                        isValueReadCorrect <= 0;
                    end
                end
            endcase
        end
    end

endmodule: readWriteFSM

module protocolFSM
  (input bit         clk, rst_b, in_trans, out_trans, pkt_sent, pkt_received, error,
   input pkt_t       pkt_in,
   input bit [63:0]  data_from_host,
   input bit [3:0]   endp,
   output bit 	     failure, success, kill, encode, decode,
   output pkt_t      pkt_out,
   output bit [63:0] data_to_host);

   logic            wait_ack;
   logic [7:0] 	    clk_count;
   logic [3:0] 	    timeout_count, corrupted_count;   
   
   enum logic [2:0] {Hold= 3'd0, InTransWait= 3'd1, InTrans = 3'd2, OutTransWait = 3'd3, OutTransDataWait= 3'd4, OutTrans= 3'd5} state;
   
    always_ff @(posedge clk, negedge rst_b) begin
        if (~rst_b) begin
            state <= Hold;
            encode <= 0;
            kill <= 0;
            timeout_count <= 0;
            clk_count <= 0;
            failure <= 0;
            success <= 0;
            corrupted_count <= 0;
            wait_ack = 0;
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
                        pkt_out.endp <= endp;
                        encode <= 1;
                        kill <= 1;

                        state <= InTransWait;
                    end
                    else if (out_trans) begin
                        pkt_out.pid <= 4'b0001;
                        pkt_out.addr <= 7'd5;
                        pkt_out.endp <= endp;
                        encode <= 1;
                        kill <= 1;
                        state <= OutTransWait;
                    end
                end // case: Hold
                InTransWait: begin // sent a packet and waiting for the signal that it was sent
                    clk_count <= 0;
                    encode <= 0;
                    kill <= 0;
                    if (pkt_sent) begin //datastream_out confirmed pckt sent
                        state <= InTrans;
                    end
                end
                InTrans: begin
                    if ((corrupted_count + timeout_count) == 4'd8) begin // corrupted max reached
                        state <= Hold;
                        failure <= 1;
                    end
                    else if (pkt_received && error) begin // corrupted data sent to pFSM
                        corrupted_count <= corrupted_count + 1;
                        pkt_out.pid <= 4'b1010; //send nak
                        encode <= 1;
                        kill <= 1;
                        state <= InTransWait;
                    end
                    else if (pkt_received && ~error) begin //correct data sent to pFSM
                        wait_ack <= 1;
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
                    else if(wait_ack && pkt_sent) begin
                        state <= Hold;
                        success <= 1;
                        encode <= 0;
                        kill <= 0;
                        wait_ack <= 0;
                    end
                    else begin
                        encode <= 0;
                        kill <= 0;
                        clk_count <= clk_count + 1'd1; //keep track of clk cycles
                    end
                end // case: InTrans
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
                OutTransDataWait: begin //wait for datastream to confirm data pkt sent
                    clk_count <= 0;
                    encode <= 0;
                    kill <= 0;
                    if (pkt_sent) begin
                        state <= OutTrans;
                    end
                end
                OutTrans: begin
                    if ((timeout_count + corrupted_count) == 4'd8) begin // corrupted max reached
                        state <= Hold;
                        failure <= 1;
                    end
                    else if (pkt_received && pkt_in.pid == 4'b1010) begin
                        // received nak so resend data
                        state <= OutTransDataWait;
                        pkt_out.pid <= 4'b0011;
                        pkt_out.data <= data_from_host;
                        encode <= 1;
                        kill <= 1;
                        corrupted_count <= corrupted_count + 1'd1;

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
                        encode <= 1;
                        kill <= 1;
                    end
                    else begin
                        clk_count <= clk_count + 1'd1; //keep track of clk cycles
                    end
                end
           endcase
        end
    end

    always_comb begin
        if((state == InTransWait || state == OutTransDataWait) && pkt_sent) begin
            decode = 1;
        end
        else begin
            decode = 0;
        end
    end

endmodule: protocolFSM

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
                    count <= 0; //reset count
                    if (encode) begin
                        state <= SEND;
                    end
                end
                SEND: begin //sending packet
                    count <= count + 1'b1; //increment count
                    if(count == size - 2'd2) begin //reached end of packet
                        state <= STANDBY; 
                        stream_done <= 1; //set done flag
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
                    in_flop <= `CRC16;
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
                        in_flop <= `CRC16;
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
                    prev <= `J;
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
                        if(stuff_done) begin 
                            EOP_count <= 1; //count goes to 1
                        end
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
            if(stuff_done) begin 
                out = `SE0; //put first SE0 on the line when packet is done
            end
            else if(state == EOP && EOP_count != 2) begin //put second SE0 on the line after that
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
    
    logic NRZI_out, EOP_error, data_begin, data_done, unstuff_out, halt_stream, CRC_out, stream_done, NRZI_done, CRC_error;
    logic [15:0] CRC_type;
    
    assign done = NRZI_done; //link done signal
    
    NRZI_in nrzi(.clk(clk), 
                 .rst_L(rst_L), 
                 .kill(kill),
                 .decode(decode), 
                 .stream_done(stream_done),
                 .in(port), 
                 .out(NRZI_out), 
                 .EOP_error(EOP_error),
                 .NRZI_active(NRZI_active),
                 .NRZI_done(NRZI_done));
                 
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
               .out(CRC_out),
               .CRC_error(CRC_error));
               
    dataPack datapack(.clk(clk), 
                      .rst_L(rst_L), 
                      .kill(kill),
                      .decode(decode), 
                      .in(CRC_out), 
                      .halt_stream(halt_stream), 
                      .EOP_error(EOP_error),
                      .NRZI_done(NRZI_done),
                      .error(error), 
                      .data_begin(data_begin), 
                      .data_done(data_done), 
                      .stream_done(stream_done),
                      .CRC_error(CRC_error), 
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
module NRZI_in (clk, rst_L, kill, decode, stream_done, in, out, EOP_error, NRZI_active, NRZI_done);
    input logic clk, rst_L, kill, decode, stream_done;
    input logic [1:0] in;
    output logic EOP_error, out, NRZI_active, NRZI_done;
    
    logic [1:0] EOP_count, prev;
    
    enum logic [1:0] {STANDBY = 2'b00, SEND = 2'b01, EOP = 2'b10} state; //state enumeration
    
    always_ff @(posedge clk, negedge rst_L, posedge kill) begin
        if(~rst_L | kill) begin //reset/kill state
            prev = `J;
            EOP_count = 0;
            EOP_error = 0;
            NRZI_active = 0;
            NRZI_done = 0;
            state = STANDBY;
        end
        
        else begin
            case(state)
                STANDBY: begin //standing by for decoding signal
                    prev <= `J;
                    EOP_error <= 0;
                    NRZI_done <= 0;
                    if(decode) begin
                        state <= SEND;
                        NRZI_active <= 1; //NRZI activated
                    end
                end
                SEND: begin //sending 
                    prev <= in; //reset previous state
                    if(stream_done) begin //watch for EOP
                        state <= EOP;
                        EOP_count <= 2'd1;
                        if(in != `SE0) begin
                            EOP_error <= 1;
                        end
                    end
                end
                EOP: begin //EOP confirmation
                    if(EOP_count == 1 && in == `SE0) begin
                        EOP_count <= 2'd2;
                    end
                    else if(EOP_count == 2 && in == `J) begin
                        EOP_count <= 2'd0;
                        state <= STANDBY;
                        NRZI_done <= 1;
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
    

    always_comb begin
        if(state == SEND)begin
            if(in == `K) begin
                out = (prev == `J) ? 1'b0 : 1'b1;
            end
            else begin
                out = (prev == `J) ? 1'b1 : 1'b0;
            end
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
module CRC_in (clk, rst_L, kill, in, data_begin, halt_stream, data_done, CRC_type, out, CRC_error);
    input logic clk, rst_L, kill, in, data_begin, halt_stream, data_done; 
    input logic [15:0] CRC_type;
    output logic out, CRC_error;

    logic [15:0] in_flop;
    
    enum logic [1:0] {STANDBY = 2'b00, CRC = 2'b01} state; //state enumeration
    
    assign out = in; //direct in to out
    
    always_ff @(posedge clk, negedge rst_L, posedge kill) begin
        if(~rst_L | kill) begin //reset/kill state
            state = STANDBY;
            in_flop = 16'hFFFF;
            CRC_error = 0;
        end
        
        else if (~halt_stream) begin //only calculate if stream not halted
            case(state)
                STANDBY: begin //waiting for data to begin
                    CRC_error <= 0;
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
                        if(CRC_type == `CRC5 && in_flop[4:0] != `CRC5_residue) begin //throw error if CRC5 residue doesn't match
                            CRC_error <= 1;
                        end
                        else if(CRC_type == `CRC16 && in_flop != `CRC16_residue) begin //throw error if CRC16 residue doesn't match
                            CRC_error <= 1;
                        end
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
module dataPack (clk, rst_L, kill, decode, in, halt_stream, EOP_error, NRZI_done, error, data_begin, data_done, stream_done, CRC_error, CRC_type, pkt_out);
    input logic clk, rst_L, kill, decode, in, halt_stream, EOP_error, NRZI_done, CRC_error;
    output logic error, data_begin, data_done, stream_done;
    output logic [15:0] CRC_type;
    output pkt_t pkt_out;

    logic PID_error;
    logic [2:0] PID_count;
    logic [4:0] CRC_count, CRC_size;
    logic [7:0] pid, count, size; 
    logic [9:0] watch;
    
    enum logic [2:0] {STANDBY = 3'b000, SYNC = 3'b001, PID = 3'b010, PACK = 3'b11, CRC = 3'b100, SEND = 3'b101} state; //state enumeration
    
    always_ff @(posedge clk, negedge rst_L, posedge kill) begin
        if(~rst_L | kill) begin //reset/kill state
            watch = 0;
            count = 0;
            pid = 0;
            PID_count = 0;
            CRC_count = 0;
            error = 0;
            data_begin = 0;
            data_done = 0;
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
                        stream_done <= 1; //put out done signal
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
                CRC: begin
                    CRC_count <= CRC_count + 1'b1; //increment CRC_count
                    if(CRC_count == CRC_size - 1) begin
                        state <= SEND; //head to SEND
                        CRC_count <= 0; //reset CRC_count
                        data_done <= 1; //indicate to upstream that data is done 
                        stream_done <= 1; //put out done signal
                    end
                end
                SEND: begin //send packet and error signals
                    if(PID_error) begin //throw error if unrecognized PID
                        error <= 1;
                    end
                    else if(pid[3:0] != ~pid[7:4])begin //throw error if PID and NPID aren't complements
                        error <= 1;
                    end
                    else if(CRC_error) begin
                        error <= 1;
                    end
                    if(NRZI_done) begin
                        state <= STANDBY; //return to standby
                    end
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
