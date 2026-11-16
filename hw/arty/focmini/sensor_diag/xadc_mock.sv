// Digital handshake model only; does NOT model analog settling or ADC accuracy.
module XADC #(parameter INIT_40=0,INIT_41=0,INIT_42=0)(
 input DCLK,RESET,input [6:0] DADDR,input DEN,DWE,input [15:0] DI,VAUXP,VAUXN,
 input VP,VN,CONVST,CONVSTCLK,output reg [15:0] DO=0,output reg DRDY=0,EOC=0,
 output reg [4:0] CHANNEL=5'h14,output [7:0] ALM,output OT,BUSY,EOS,JTAGBUSY,JTAGLOCKED,JTAGMODIFIED,
 output [4:0] MUXADDR);
 reg run_adc=1;
 reg [11:0] sample=12'h654;
 integer count=0;
 initial if(INIT_40!=16'h0114||INIT_41!=16'h3fff||INIT_42!=16'h6400) $fatal(1,"XADC config");
 always @(posedge DCLK) begin
  EOC<=0;DRDY<=0;
  if(RESET) count<=0;
  else begin
   if(count==31) begin count<=0;if(run_adc) EOC<=1;end else count<=count+1;
   if(DEN) begin
    if(DADDR!=7'h14 || DWE) $fatal(1,"incorrect DRP read");
    DO<={sample,4'b0};DRDY<=1;
   end
  end
 end
 assign ALM=0;assign {OT,BUSY,EOS,JTAGBUSY,JTAGLOCKED,JTAGMODIFIED}=0;assign MUXADDR=0;
endmodule
