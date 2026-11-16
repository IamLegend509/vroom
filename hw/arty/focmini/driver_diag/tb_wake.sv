`timescale 1ns/1ps
module tb_wake;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,arm=0,sweep=0,stop=0,ping=0,uart_error=0,feed=1;
 wire sl,re;wire [2:0] state;wire [3:0] result;wire [13:0] elapsed;wire rejected;
 wake_control #(.MS_CYCLES(100),.WAKE_CYCLES(1000),.KEEPALIVE_CYCLES(500)) dut(
 .clk(clk),.rst(rst),.arm(arm),.sweep(sweep),.stop(stop),.ping(ping),.uart_error(uart_error),
 .sleep_n(sl),.reset_n(re),.state(state),.result(result),.elapsed_ms(elapsed),.rejected(rejected));
 integer count=0,active_ticks=0;
 integer trace,last_state=-1,last_ms=-1;
 initial begin trace=$fopen("wake_trace.csv","w");$fdisplay(trace,"sim_ns,state,elapsed_ms,sleep_n,reset_n,result");end
 always @(posedge clk) if(!rst && state!=0) active_ticks=active_ticks+1;
 always @(negedge clk) begin
  count=count+1;ping=feed && count%100==0;
  if(state==0 && (sl || re)) $fatal(1,"idle pins high");
  if(re && !sl) $fatal(1,"reset released while asleep");
  if(state!=last_state || elapsed!=last_ms) begin
   $fdisplay(trace,"%0.0f,%0d,%0d,%0d,%0d,%0d",$realtime,state,elapsed,sl,re,result);
   last_state=state;last_ms=elapsed;
  end
 end
 task reset_all;
  begin
   @(negedge clk);rst=1;arm=0;sweep=0;stop=0;feed=1;uart_error=0;
   repeat(5) @(negedge clk);rst=0;repeat(2) @(negedge clk);
  end
 endtask
 task launch;
  begin @(negedge clk);arm=1;@(negedge clk);arm=0;end
 endtask
 time began;
 initial begin
  reset_all;sweep=1;@(negedge clk);sweep=0;if(state!=0) $fatal(1,"SWEEP activated diagnostic");
  launch;began=$time;#1;
  if(state!=1 || !sl || re) $fatal(1,"wake ordering");
  repeat(999) @(negedge clk);#1;if(re) $fatal(1,"reset released early");
  @(negedge clk);#1;if(state!=2 || !re) $fatal(1,"reset not released after delay");
  sweep=1;repeat(20) @(negedge clk);sweep=0;
  wait(state==0);#1;
  if(result!=1 || elapsed!=10000 || active_ticks!=1000000) $fatal(1,"duration mismatch dt=%0t",$time-began);
  reset_all;launch;feed=0;wait(state==0);if(result!=8) $fatal(1,"watchdog");
  reset_all;launch;stop=1;#1;if(sl || re) $fatal(1,"immediate STOP");@(negedge clk);stop=0;
  if(state!=0 || result!=2) $fatal(1,"STOP state");
  reset_all;launch;arm=1;#1;if(sl || re) $fatal(1,"repeated ARM must stop");@(negedge clk);arm=0;
  if(state!=0 || result!=2) $fatal(1,"ARM restarted duration");
  reset_all;launch;uart_error=1;#1;if(sl || re) $fatal(1,"UART kill");@(negedge clk);uart_error=0;
  if(state!=0 || result!=9) $fatal(1,"UART result");
  reset_all;launch;rst=1;#1;if(sl || re) $fatal(1,"reset gate");
  $display("PASS WAKE: startup order/delay, ten-second duration, SWEEP rejection, repeat ARM disarms, STOP/reset/UART/keepalive shutdown");$finish;
 end
 initial begin #12000000;$fatal(1,"wake timeout");end
endmodule
