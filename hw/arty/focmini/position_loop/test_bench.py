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
      elif s=='TEST':
       n+=1
       if state==3:state=5;started=time.monotonic();t=0
       else:x+=1
      elif s=='PING':n+=1
      else:raise AssertionError('Unexpected command '+repr(s))
    if state==5 and started:
     t=min(51000,int((time.monotonic()-started)*40000))
     if self.kind=='fault' and t>2000:state=6;result=4
     elif self.kind in ('complete','bad_settling') and t==51000:state=0;result=1
     if self.kind=='dropout' and sent_run:continue
    packet=f'C3 a=4B6 v=1 c=0 i=1 e=0 s=37 r=254 b=3 f=1 m={state:X} q={result:X} n={n:04X} t={t:04X} p=00 g={int(state==5)} x={x:04X} d=254 z=000 o=0000 h=1 k={int(t>=2000)} y={15 if result==1 and self.kind!='bad_settling' else 0:X}\r\n'
    if self.kind=='wrong_protocol':packet=packet.replace('C3','O1',1)
    os.write(self.master,packet.encode());sent_run|=state==5
  except BaseException as e:self.error=e
 def close(self):
  self.done.set();self.worker.join(1);os.close(self.master);os.close(self.slave)
  if self.error:raise self.error

root=pathlib.Path('host_protocol_tests');root.mkdir()
results=[]
for kind,mode,expected in [('dry','check','disarmed_check_passed'),('complete','test','position_test_completed'),('bad_settling','test','Position test ended without a verified completed state'),('wrong_protocol','test','No valid C3 telemetry'),('fault','test','FPGA latched driver fault'),('dropout','test','No valid C3 telemetry')]:
 out=root/kind;out.mkdir();board=Board(kind);link=Link(board.port,out)
 try:
  try:answer=session(link,mode);assert answer['outcome']==expected;observed=answer['outcome']
  except (RuntimeError,TimeoutError) as exc:
   assert expected in str(exc),(kind,str(exc));observed=str(exc)
 finally:
  if link.identified:link.send('!')
  time.sleep(.03);link.close();board.close()
 if kind=='wrong_protocol':assert board.commands==[]
 elif mode=='check':assert 'ARM' not in board.commands
 else:assert board.commands.count('ARM')==1 and board.commands.count('TEST')==1
 if kind!='wrong_protocol':assert board.commands[-1]=='!'
 results.append({'case':kind,'observed':observed,'commands':board.commands})
(root/'results.json').write_text(json.dumps(results,indent=2))
print('PASS HOST: actual pseudo-terminal framing; disarmed check sends no ARM; one position test only; completion, fault, telemetry dropout and final STOP')
