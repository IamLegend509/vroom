#!/usr/bin/env python3
"""Host integration over a pseudo-terminal; no physical serial device is opened.
This is a protocol emulator, not an RTL/motor model. Real RTL tested separately.
"""
import json,os,pathlib,pty,select,threading,time
from bench import Link,session

class Board:
 def __init__(self,kind):
  self.master,self.slave=pty.openpty();self.port=os.ttyname(self.slave)
  self.kind=kind;self.commands=[];self.done=threading.Event();self.error=None
  self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
 def run(self):
  state=0;result=0;n=0;x=0;t=0;buf=b'';started=None;sent_run=False
  try:
   while not self.done.wait(.01):
    if select.select([self.master],[],[],0)[0]:
     buf+=os.read(self.master,4096)
     while b'\n' in buf:
      raw,_,buf=buf.partition(b'\n');s=raw.decode();self.commands.append(s)
      if s=='!':
       n+=1;x+=1
       if state!=6:state=0;result=2
      elif s=='ARM':n+=1;state=3;result=0
      elif s=='SWEEP':
       n+=1
       if state==3:state=5;started=time.monotonic();t=0
       else:x+=1
      elif s=='PING':n+=1
      else:raise AssertionError('Unexpected command '+repr(s))
    if state==5 and started:
     t=min(10000,int((time.monotonic()-started)*40000))
     if self.kind=='fault' and t>2000:state=6;result=4
     elif self.kind=='complete' and t==10000:state=0;result=1
     if self.kind=='dropout' and sent_run:continue
    packet=f'O1 a=4B6 v=1 c=0 i=1 e=0 s=37 r=254 b=3 f=1 m={state:X} q={result:X} n={n:04X} t={t:04X} p=00 g={int(state==5)} x={x:04X}\r\n'
    os.write(self.master,packet.encode());sent_run|=state==5
  except BaseException as e:self.error=e
 def close(self):
  self.done.set();self.worker.join(1);os.close(self.master);os.close(self.slave)
  if self.error:raise self.error

root=pathlib.Path('host_protocol_tests');root.mkdir()
results=[]
for kind,mode,expected in [('dry','check','disarmed_check_passed'),('complete','sweep','sweep_completed'),('fault','sweep','FPGA latched driver fault'),('dropout','sweep','No valid O1 telemetry')]:
 out=root/kind;out.mkdir();board=Board(kind);link=Link(board.port,out)
 try:
  try:answer=session(link,mode);assert answer['outcome']==expected;observed=answer['outcome']
  except (RuntimeError,TimeoutError) as exc:
   assert expected in str(exc),(kind,str(exc));observed=str(exc)
 finally:
  if link.identified:link.send('!')
  time.sleep(.03);link.close();board.close()
 if mode=='check':assert 'ARM' not in board.commands
 else:assert board.commands.count('ARM')==1 and board.commands.count('SWEEP')==1
 assert board.commands[-1]=='!'
 results.append({'case':kind,'observed':observed,'commands':board.commands})
(root/'results.json').write_text(json.dumps(results,indent=2))
print('PASS HOST: actual pseudo-terminal framing; disarmed check sends no ARM; one sweep only; completion, fault, telemetry dropout and final STOP')
