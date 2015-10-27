// Write your usb host here.  Do not modify the port list.
interface usbWires;
	tri0 DP;
	tri0 DM;

endinterface

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
