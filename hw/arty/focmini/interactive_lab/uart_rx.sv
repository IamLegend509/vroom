// UART 8-N-1 receiver. First-stage CDC isolation, mid-bit sampling and stop check.
// A continuous BREAK produces one error, then waits for idle high.
module sweep_uart_rx #(parameter integer DIV=868)(
 input wire clk,rst,rx, output reg [7:0] data,
 output reg valid,framing_error
);
 initial data=0; initial valid=0; initial framing_error=0;
 (* ASYNC_REG="TRUE" *) reg [1:0] rx_sync=2'b11;
 always @(posedge clk) rx_sync<={rx_sync[0],rx};
 reg [2:0] state=0;
 reg [2:0] bit_index=0;
 reg [7:0] shift=0;
 integer timer=0;
 always @(posedge clk) begin
  valid<=0;framing_error<=0;
  if(rst) begin state<=0;timer<=0;data<=0;shift<=0;bit_index<=0;end
  else case(state)
   0: if(!rx_sync[1]) begin timer<=DIV/2-1;state<=1;end
   1: if(timer!=0) timer<=timer-1;
      else if(rx_sync[1]) state<=0;
      else begin timer<=DIV-1;bit_index<=0;state<=2;end
   2: if(timer!=0) timer<=timer-1;
      else begin
       shift[bit_index]<=rx_sync[1];timer<=DIV-1;
       if(bit_index==7) state<=3; else bit_index<=bit_index+1'b1;
      end
   3: if(timer!=0) timer<=timer-1;
      else if(rx_sync[1]) begin data<=shift;valid<=1;state<=0;end
      else begin framing_error<=1;state<=4;end
   4: if(rx_sync[1]) state<=0;
   default: state<=0;
  endcase
 end
endmodule
