#!/usr/bin/env python3
"""One bounded bench transaction. Default check never sends ARM. No retries.
Run from a verified run's source snapshot, not a mutable worktree. Raw RX and
command timestamps are retained even on failure. No DTR/RTS toggles.
"""
import argparse,datetime,hashlib,json,os,pathlib,re,select,termios,time

FIELDS='a v c i e s r b f m q n t p g x'.split()
PATTERN=re.compile(rb'O2 a=([0-9A-F]{3}) v=([01]) c=([01]) i=([01]) e=([0-9A-F]) s=([0-9A-F]{2}) r=([0-9A-F]{3}) b=([0-3]) f=([01]) m=([0-6]) q=([0-9A-F]) n=([0-9A-F]{4}) t=([0-9A-F]{4}) p=([0-9A-F]{2}) g=([01]) x=([0-9A-F]{4})\r\n')
RESULT={0:'none',1:'completed',2:'stopped',3:'arm expired',4:'driver fault',5:'ADC stale/clipped',6:'encoder invalid/stale',7:'angle excursion',8:'keepalive expired',9:'UART framing error'}

def healthy(row):
 return row['v']==1 and row['c']==0 and row['i']==1 and row['e']==0 and row['s']&0x20 and not row['s']&0x08

class Link:
 def __init__(self,port,out):
  self.fd=os.open(port,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
  self.out=out;self.buf=bytearray();self.identified=False;self.start=time.monotonic()
  self.raw=(out/'serial.raw').open('wb');self.rows=(out/'packets.jsonl').open('w');self.tx=(out/'commands.jsonl').open('w')
  self.packet_count=0;self.weak_count=0
  t=termios.tcgetattr(self.fd);t[0]=0;t[1]=0
  t[2]=(t[2]&~(termios.CSIZE|termios.PARENB|termios.CSTOPB|termios.HUPCL))|termios.CS8|termios.CLOCAL|termios.CREAD
  t[3]=0;t[4]=termios.B115200;t[5]=termios.B115200
  termios.tcsetattr(self.fd,termios.TCSANOW,t);termios.tcflush(self.fd,termios.TCIFLUSH)
 def send(self,command):
  if not self.identified: raise RuntimeError('O2 firmware not identified; refusing serial writes')
  data=command.encode('ascii')+b'\n'
  if os.write(self.fd,data)!=len(data): raise RuntimeError('Incomplete command write')
  self.tx.write(json.dumps({'seconds':time.monotonic()-self.start,'command':command})+'\n');self.tx.flush()
 def receive(self,timeout=.4):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   if b'\n' in self.buf:
    line,_,remaining=self.buf.partition(b'\n');self.buf=bytearray(remaining)
    m=PATTERN.fullmatch(line+b'\n')
    if m:
     self.identified=True;row=dict(zip(FIELDS,[int(x,16) for x in m.groups()]))
     self.packet_count+=1;self.weak_count+=bool(row['s']&0x10)
     self.rows.write(json.dumps({'seconds':time.monotonic()-self.start,**row})+'\n');self.rows.flush()
     return row
    continue
   if select.select([self.fd],[],[],max(0,min(.05,end-time.monotonic())))[0]:
    data=os.read(self.fd,65536)
    if not data: raise RuntimeError('Serial device disconnected')
    self.raw.write(data);self.raw.flush();self.buf.extend(data)
    if len(self.buf)>65536: raise RuntimeError('Invalid/unbounded serial stream')
  raise TimeoutError('No valid O2 telemetry within deadline')
 def close(self):
  os.close(self.fd);self.raw.close();self.rows.close();self.tx.close()

def wait_for(link,predicate,seconds=2):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  row=link.receive()
  if row['m']==6: raise RuntimeError('FPGA latched '+RESULT.get(row['q'],'unknown fault'))
  if predicate(row): return row
 raise TimeoutError('Expected FPGA state was not acknowledged')

def session(link,mode):
 row=link.receive(2)
 link.send('!')
 if mode=='stop':
  row=link.receive();return {'outcome':'stop_sent','last_packet':row,'note':'fault latch needs BTN0; stop does not clear it'}
 row=wait_for(link,lambda r:r['m']==0 and r['g']==0 and r['q']==2)
 if mode=='check':
  n,x=row['n'],row['x'];link.send('SWEEP')
  row=wait_for(link,lambda r:r['n']==((n+1)&65535) and r['x']==((x+1)&65535))
  if row['m']!=0 or row['g']!=0: raise RuntimeError('Unarmed SWEEP did not remain disabled')
  for _ in range(10):
   row=link.receive()
   if row['m']!=0 or row['g']!=0: raise RuntimeError('Unexpected activity during disarmed check')
  return {'outcome':'disarmed_check_passed','last_packet':row,'note':'telemetry verifies logic state, not physical pin voltages'}
 for _ in range(5):
  row=link.receive()
  if not healthy(row) or row['m']!=0 or row['g']!=0: raise RuntimeError('Sensors not ready while disarmed')
 link.send('ARM')
 row=wait_for(link,lambda r:r['m']==3 and r['g']==0)
 if not row['f'] or not healthy(row): raise RuntimeError('Driver/sensors not ready after ARM')
 link.send('SWEEP') # Exactly one start; no retry on any error.
 deadline=time.monotonic()+37;started=False
 while time.monotonic()<deadline:
  link.send('PING')
  row=link.receive(.4)
  if row['m']==6: raise RuntimeError('FPGA latched '+RESULT.get(row['q'],'unknown fault'))
  if row['m']==5:
   started=True
   if not row['g'] or not row['f'] or not healthy(row): raise RuntimeError('Invalid running telemetry')
  elif row['m']==0:
   if started and row['q']==1 and row['t']==35000 and row['g']==0:
    return {'outcome':'sweep_completed','last_packet':row,'note':'timed 1.75 electrical-turn sweep completed; actual shaft angle must be measured from telemetry'}
   raise RuntimeError('Sweep ended without a verified completed state')
  elif row['m'] not in (3,4): raise RuntimeError('Unexpected sweep state')
 raise TimeoutError('Host sweep deadline exceeded')

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--port',required=True);p.add_argument('--run',type=pathlib.Path,required=True)
 p.add_argument('--mode',choices=('check','sweep','stop'),default='check')
 p.add_argument('--motor-power-ready',action='store_true',help='For sweep: operator has checked 9V current-limited supply, wiring and clear shaft')
 a=p.parse_args()
 if a.mode=='sweep' and not a.motor_power_ready:p.error('sweep requires --motor-power-ready after the physical checks')
 meta=json.loads((a.run/'metadata.json').read_text());bit=a.run/'open_loop_90.bit'
 if meta['status']!='bitstream_built' or hashlib.sha256(bit.read_bytes()).hexdigest()!=meta['bitstream_sha256']:
  p.error('run must contain an intact, successfully built open_loop_90.bit')
 out=a.run/('bench_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'));out.mkdir()
 summary={'status':'running','mode':a.mode,'port':a.port,'build_sha256':meta['bitstream_sha256'],
  'loaded_image_identity':'O2 protocol only; operator must have programmed the specified bitstream',
  'host_sha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()}
 link=None
 try:
  link=Link(a.port,out);summary.update(session(link,a.mode));summary['status']='passed'
 except BaseException as exc:
  summary.update(status='failed',error=str(exc));raise
 finally:
  if link:
   try:
    if link.identified:link.send('!')
   except Exception as exc:summary['stop_write_error']=str(exc)
   summary.update(packets=link.packet_count,weak_field_packets=link.weak_count);link.close()
  (out/'summary.json').write_text(json.dumps(summary,indent=2));print(out);print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
