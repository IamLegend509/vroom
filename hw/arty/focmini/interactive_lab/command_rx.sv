// Interactive command parser:
//   ARM, TEST, PING, STOP, '!'
//   D<3-hex-nibbles> (e.g. D3A5 -> sets 12-bit target angle = 12'h3A5)
//   S<2-hex-nibbles> (e.g. S18  -> sets max reference slew step)
//   M0               (Classical fixed-1.15V linear phase integrator)
//   M1               (RL / Nonlinear IASA + Dynamic Voltage Modulation law)
module position_commands #(parameter integer GAP_CYCLES=10000000)(
 input wire clk,rst,valid,framing_error,input wire [7:0] data,
 output reg arm,sweep,stop,ping,rejected,
 output reg [15:0] accepted,
 output reg cmd_target_valid,
 output reg [11:0] cmd_target,
 output reg [7:0] cmd_slew,
 output reg rl_mode
);
 initial begin arm=0; sweep=0; stop=0; ping=0; rejected=0; accepted=0; cmd_target_valid=0; cmd_target=0; cmd_slew=8'd24; rl_mode=1'b1; end
 reg [39:0] line=0;
 reg [2:0] length=0;
 reg discard=0;
 integer age=0;

 function automatic [4:0] hex_nib(input [7:0] c);
  reg [7:0] d;
  begin
   if(c>=8'h30 && c<=8'h39) hex_nib = {1'b1, c[3:0]};
   else if(c>=8'h41 && c<=8'h46) begin d = c - 8'h41 + 8'd10; hex_nib = {1'b1, d[3:0]}; end
   else if(c>=8'h61 && c<=8'h66) begin d = c - 8'h61 + 8'd10; hex_nib = {1'b1, d[3:0]}; end
   else hex_nib = 5'd0;
  end
 endfunction

 wire [4:0] n2 = hex_nib(line[23:16]);
 wire [4:0] n1 = hex_nib(line[15:8]);
 wire [4:0] n0 = hex_nib(line[7:0]);

 always @(posedge clk) begin
  arm<=0;sweep<=0;stop<=0;ping<=0;rejected<=0;cmd_target_valid<=0;
  if(rst) begin line<=0;length<=0;discard<=0;age<=0;accepted<=0;cmd_target<=0;cmd_slew<=8'd24;rl_mode<=1'b1;end
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
     else if(length==4 && line[31:24]==8'h44 && n2[4] && n1[4] && n0[4]) begin
      cmd_target <= {n2[3:0], n1[3:0], n0[3:0]};
      cmd_target_valid <= 1'b1;
      ping <= 1'b1;
      accepted <= accepted + 1'b1;
     end else if(length==3 && line[23:16]==8'h53 && n1[4] && n0[4]) begin
      cmd_slew <= ({n1[3:0], n0[3:0]} == 8'd0) ? 8'd1 : {n1[3:0], n0[3:0]};
      ping <= 1'b1;
      accepted <= accepted + 1'b1;
     end else if(length==2 && line[15:8]==8'h4d && (line[7:0]==8'h30 || line[7:0]==8'h31)) begin
      rl_mode <= (line[7:0] == 8'h31);
      ping <= 1'b1;
      accepted <= accepted + 1'b1;
     end else if(length!=0) rejected<=1;
     line<=0;length<=0;discard<=0;
    end else if(!discard) begin
     if(data<8'h20 || data>8'h7e || length==5 || (length!=0 && age>=GAP_CYCLES-1)) begin
      discard<=1;line<=0;length<=0;
     end else begin line<={line[31:0],data};length<=length+1'b1;end
    end
   end
  end
 end
endmodule
