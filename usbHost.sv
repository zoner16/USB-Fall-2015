//`default_nettype none

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

typedef struct packed {
  logic [3:0] pid;
  logic [3:0] endp;
  logic [6:0] addr;
  logic [63:0] data;
} pkt_t;

interface usbWires;
	tri0 DP;
	tri0 DM;

endinterface
// Write your usb host here.  Do not modify the port list.

module usbHost
    (input logic clk, rst_L,
     usbWires wires);

    /* Tasks needed to be finished to run testbenches */
    
    logic encode, decode, kill, error, in_done, out_done, NRZI_in_active, NRZI_out_active;
    logic [1:0] port, port_in, port_out;
    pkt_t pkt_in, pkt_out;

    task prelabRequest();
    // sends an OUT packet with ADDR=5 and ENDP=4
    // packet should have SYNC and EOP too
        pkt_in.pid     <= 4'b0001;
        pkt_in.endp    <= 4'b0100;
        pkt_in.addr    <= 7'b0000101;
        pkt_in.data    <= 64'd0;
        encode  <= 1;
        
        @(posedge clk);
        encode  <= 0;
        
        wait(out_done);

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

endmodule: usbHost


