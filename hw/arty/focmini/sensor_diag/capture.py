#!/usr/bin/env python3
"""Read-only bounded capture. Never writes serial data or changes modem-control lines."""
import argparse,datetime,hashlib,json,os,pathlib,re,select,statistics,termios,time
p=argparse.ArgumentParser();p.add_argument('--port',required=True);p.add_argument('--seconds',type=float,default=10);p.add_argument('--run',type=pathlib.Path,required=True);a=p.parse_args()
assert 0<a.seconds<=60
out=a.run/('capture_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'));out.mkdir()
fd=os.open(a.port,os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
try:
 t=termios.tcgetattr(fd);t[0]=0;t[1]=0;t[2]=(t[2]&~(termios.CSIZE|termios.PARENB|termios.CSTOPB|termios.HUPCL))|termios.CS8|termios.CLOCAL|termios.CREAD;t[3]=0;t[4]=termios.B115200;t[5]=termios.B115200
 termios.tcsetattr(fd,termios.TCSANOW,t);termios.tcflush(fd,termios.TCIFLUSH)
 b=bytearray();end=time.monotonic()+a.seconds
 while time.monotonic()<end:
  if select.select([fd],[],[],.1)[0]:b.extend(os.read(fd,65536))
finally:os.close(fd)
(out/'serial.raw').write_bytes(b)
rows=[]
pattern=r'S1 a=([0-9A-F]{3}) v=([01]) c=([01]) i=([01]) e=([0-9A-F]) s=([0-9A-F]{2}) r=([0-9A-F]{3}) b=([0-3]) f=([01])'
for m in re.finditer(pattern,b.decode(errors='replace')):rows.append(dict(zip(['adc','fresh','clip','i2c_ok','error','status','angle','bus','fault'],[int(x,16) for x in m.groups()])))
meta={'port':a.port,'seconds':a.seconds,'packets':len(rows),'raw_sha256':hashlib.sha256(b).hexdigest(),'scope':'sensor-only diagnostic, no motion commands, raw ADC not calibrated','rows':rows}
if rows:
 v=[x['adc'] for x in rows];ok=[x for x in rows if x['i2c_ok']]
 meta.update(adc_mean=statistics.mean(v),adc_min=min(v),adc_max=max(v),adc_stddev=statistics.pstdev(v),fresh_packets=sum(x['fresh'] for x in rows),clip_packets=sum(x['clip'] for x in rows),i2c_successes=len(ok),errors=sorted(set(x['error'] for x in rows)),statuses=sorted(set(x['status'] for x in ok)),angle_min=min((x['angle'] for x in ok),default=None),angle_max=max((x['angle'] for x in ok),default=None))
(out/'metrics.json').write_text(json.dumps(meta,indent=2));print(out);print(json.dumps({k:v for k,v in meta.items() if k!='rows'},indent=2));print(b.decode(errors='replace')[:1200])
