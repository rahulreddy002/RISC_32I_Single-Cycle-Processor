
module tb_risc ();
  
  reg clk,rst;
  
  risc_top uut (.clk(clk),.rst(rst));
  
  initial begin
    clk=1'b0;
  end
  
  
  always begin
    #5 clk=~clk;
  end
  
  initial begin
    rst=1'b1;
    #5;
    rst=1'b0;
     #200 $finish();
  end
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars ();
   
  end
  
endmodule
  
