// Exact uppercase line commands. Overflow/invalid/partial lines cannot expose a
// valid suffix. CRLF empty line ignored. '!' is an immediate stop, anywhere.
module position_commands #(parameter integer GAP_CYCLES=10000000)(
 input wire clk,rst,valid,framing_error,input wire [7:0] data,
 output reg arm=0,sweep=0,stop=0,ping=0,rejected=0,
 output reg [15:0] accepted=0
);
 reg [39:0] line=0;
 reg [2:0] length=0;
 reg discard=0;
 integer age=0;
 always @(posedge clk) begin
  arm<=0;sweep<=0;stop<=0;ping<=0;rejected<=0;
  if(rst) begin line<=0;length<=0;discard<=0;age<=0;accepted<=0;end
  else begin
   if(length!=0 && age<GAP_CYCLES) age<=age+1;
   if(length!=0 && age==GAP_CYCLES-1) begin
    line<=0;length<=0;discard<=1;rejected<=1;
   end
   if(framing_error) begin line<=0;length<=0;discard<=1;rejected<=1;end
   else if(valid) begin
    age<=0;
    if(data==8'h21) begin // !
     stop<=1;accepted<=accepted+1'b1;line<=0;length<=0;discard<=1;
    end else if(data==8'h0a || data==8'h0d) begin
     if(discard || (length!=0 && age>=GAP_CYCLES-1)) rejected<=1;
     else if(length==3 && line==40'h000041524d) begin arm<=1;accepted<=accepted+1'b1;end
     else if(length==4 && line==40'h0054455354) begin sweep<=1;accepted<=accepted+1'b1;end
     else if(length==4 && line==40'h0053544f50) begin stop<=1;accepted<=accepted+1'b1;end
     else if(length==4 && line==40'h0050494e47) begin ping<=1;accepted<=accepted+1'b1;end
     else if(length!=0) rejected<=1;
     line<=0;length<=0;discard<=0;
    end else if(!discard) begin
     // Include an expiring byte in rejection, never resurrect a timed-out line.
     if(data<8'h20 || data>8'h7e || length==5 || (length!=0 && age>=GAP_CYCLES-1)) begin
      discard<=1;line<=0;length<=0;
     end else begin line<={line[31:0],data};length<=length+1'b1;end
    end
   end
  end
 end
endmodule
