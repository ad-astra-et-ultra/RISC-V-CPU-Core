module top();
  reg clk;
  reg reset;
  initial begin
    reset<=1;
    clk<=0;
  end
  always #10 clk <= ~clk;

  cpu_top cpu_inst(.clk(clk), .reset(reset));
    initial begin
    #1 reset = ~reset;
    end

  initial
   #60000 $finish;
  
endmodule
