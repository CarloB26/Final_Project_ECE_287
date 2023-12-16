`timescale 1ps / 1ps

module tb();

  // Inputs
  reg clk;
  reg start;
  reg rst;
  reg [31:0] in;
  reg [31:0] kp;
  reg [31:0] ki; 
  reg [31:0] kd; 
  
  // Output
  wire [31:0] o;

  // Instantiate the module under test
  project uut (
    .clk(clk),
    .start(start),
    .rst(rst),
    .in(in),
	 .kp(kp),
	 .ki(ki),
	 .kd(kd),
    .o(o)
  );

  // Clock generation
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  // Initial stimulus
  initial begin
    // Initialize inputs
    start = 0;
    rst = 0;
	 kp = 3; //set kp value
	 ki = 2; //set ki value
	 kd = 1; //set kd value
	 in = 32'b00111111; //set inputvalue 

    // Apply reset
    #10 rst = 1;

    // Apply start signal
    #10 start = 1;

    #50000; //run until 50000ps
    $finish;
  end

  //monitor values for debugging
  initial
  begin
    $monitor("uut.i=%b uut.s=%b in=%b out=%b uut.error=%b uut.prev_error=%b uut.y=%b uut.y_1=%b uut.y_int=%b uut.y_int_1=%b uut.yd=%b uut.ydd=%b uut.yd_1=%b uut.ydd_1=%b uut.yddtemp=%b" , 
             uut.i, uut.s, in, uut.o, uut.error, uut.prev_error, uut.y, uut.y_1, uut.y_int, uut.y_int_1, uut.yd, uut.ydd, uut.yd_1, uut.ydd_1, uut.yddtemp);
  end

endmodule
