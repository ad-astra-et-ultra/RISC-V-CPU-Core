`include "riscV_core.v"

module top ();

reg clk;
reg reset;
wire zero;

initial begin
   reset = 0;
   
end

initial begin
   clk = 0;
   forever #10 clk = ~clk;
end
/*clock cl (clk);*/
cpu_top cpu(clk, reset, zero);

initial begin
$dumpfile("output_wave.vcd");
$dumpvars(0,top);
end

initial begin: TOP_STIMULUS
reset=0;
@(negedge clk);
reset=1;
end

initial
   #1400 $finish;
endmodule
