`timescale 1ns/1ps
module tb_i2c;
    reg clk=0; always #5 clk=~clk;
    reg rst=1,start=0;
    tri1 scl,sda;
    wire mlc,mld,busy,done,ok;
    reg slc=0,sld=0;
    assign scl=(mlc||slc)?1'b0:1'bz;
    assign sda=(mld||sld)?1'b0:1'bz;
    wire [3:0] error; wire [7:0] status; wire [11:0] angle; wire [1:0] bus_level;
    as5600_reader #(.CLK_HZ(4000000)) dut(.clk(clk),.rst(rst),.start(start),.scl_in(scl),.sda_in(sda),
        .scl_low(mlc),.sda_low(mld),.busy(busy),.done(done),.ok(ok),.error(error),
        .status(status),.angle(angle),.bus_level(bus_level));
    task launch;
        begin repeat(10) @(negedge clk); start=1; @(negedge clk); start=0; end
    endtask
    task get_byte(input [7:0] expected);
        reg [7:0] b; integer i;
        begin
            for(i=7;i>=0;i=i-1) begin @(posedge scl); #1; b[i]=sda; end
            if(b!==expected) $fatal(1,"write mismatch got %h expected %h",b,expected);
        end
    endtask
    task ack_byte(input bit yes, input integer stretch);
        begin
            @(negedge scl); sld=yes;
            if(stretch!=0) begin slc=1; repeat(stretch) @(negedge clk); slc=0; end
            @(posedge scl); @(negedge scl); sld=0;
        end
    endtask
    task put_byte(input [7:0] b,input bit expect_ack);
        integer i;
        begin
            for(i=7;i>=0;i=i-1) begin
                sld=~b[i]; @(posedge scl); @(negedge scl);
            end
            sld=0; @(posedge scl); #1;
            if(sda!==!expect_ack) $fatal(1,"master ACK/NACK incorrect");
            @(negedge scl);
        end
    endtask
    task slave(input integer nack_stage,input integer stretch);
        begin : transaction
            @(negedge sda); if(scl!==1) $fatal(1,"no START");
            get_byte(8'h6c); ack_byte(nack_stage!=0,stretch);
            if(nack_stage==0) disable transaction;
            get_byte(8'h0b); ack_byte(nack_stage!=1,0);
            if(nack_stage==1) disable transaction;
            @(negedge sda); if(scl!==1) $fatal(1,"no RESTART");
            get_byte(8'h6d); ack_byte(nack_stage!=2,0);
            if(nack_stage==2) disable transaction;
            put_byte(8'h20,1); put_byte(8'h0a,1); put_byte(8'hbc,0);
            @(posedge sda); if(scl!==1) $fatal(1,"no STOP");
        end
    endtask
    task check(input bit success,input [3:0] code);
        begin wait(done); #1;
            if(ok!==success || error!==code) $fatal(1,"result ok=%b err=%d expected %b %d",ok,error,success,code);
            if(success && (status!==8'h20 || angle!==12'habc)) $fatal(1,"data corrupt");
            if(mlc || mld) $fatal(1,"bus not released");
            @(negedge clk);
        end
    endtask
    integer n;
    initial begin
        repeat(10) @(negedge clk); rst=0;
        fork slave(-1,0); begin launch; check(1,0); end join
        fork slave(-1,100); begin launch; check(1,0); end join
        for(n=0;n<3;n=n+1) begin
            fork slave(n,0); begin launch; check(0,4'(n+2)); end join
        end
        sld=1; launch; check(0,1); sld=0;
        slc=1; launch; check(0,1); slc=0;
        fork
            begin @(negedge sda); @(negedge scl); slc=1; end
            begin launch; check(0,5); end
        join
        slc=0;
        fork slave(-1,0); begin launch; check(1,0); end join
        fork
            begin launch; repeat(40) @(negedge clk); rst=1; @(negedge clk);
                if(mlc||mld||busy) $fatal(1,"reset not released"); rst=0; end
        join
        $display("PASS I2C: data, ACK/NACK at 3 stages, stretch, stuck SDA/SCL, timeout, recovery, reset");
        $finish;
    end
    initial begin #2000000; $fatal(1,"test timeout"); end
endmodule
