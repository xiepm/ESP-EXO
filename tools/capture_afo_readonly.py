"""Read AFO status/counters via USB without reset, START, enable or config writes."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import time


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds',type=float,default=10)
    ap.add_argument('--port',help='Optional port; still verifies ESP32 USB VID/PID')
    ap.add_argument('--output',type=Path)
    args=ap.parse_args()
    if not 3<=args.seconds<=60:ap.error('seconds must be 3..60')
    import serial
    from serial.tools import list_ports
    ports=[p for p in list_ports.comports() if p.vid==0x303a and p.pid==0x1001
           and (not args.port or p.device.upper()==args.port.upper())]
    if len(ports)!=1:raise SystemExit('Expected exactly one matching ESP32 USB device; enumerate/select --port')
    device=ports[0]
    root=Path(__file__).resolve().parents[1]
    out=args.output or root/'output/afo_readonly'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True,exist_ok=False)
    (out/'device.json').write_text(json.dumps(dict(port=device.device,serial=device.serial_number,vid=device.vid,pid=device.pid),indent=2),encoding='utf-8')
    port=serial.Serial(port=None,baudrate=115200,timeout=.05,write_timeout=2)
    port.dtr=False;port.rts=False;port.port=device.device
    raw=(out/'uart.log').open('xb');responses=[]
    def receive(duration):
        end=time.monotonic()+duration;data=bytearray()
        while time.monotonic()<end:
            part=port.read(max(1,min(port.in_waiting,4096)));raw.write(part);data.extend(part)
        raw.flush()
        return re.sub(r'\x1b\[[0-9;]*m','',data.decode('utf-8',errors='replace')).replace('\r','')
    def query(command):
        assert command in ('system_status','control_status','adaptive_status')
        port.write((command+'\r').encode('ascii'))
        responses.append(dict(host_time=datetime.now().astimezone().isoformat(),command=command,response=receive(.8)))
    try:
        port.open();receive(.2);query('system_status')
        end=time.monotonic()+args.seconds
        while time.monotonic()<end:
            query('adaptive_status');receive(.2)
        query('control_status');query('system_status')
    finally:
        port.close();raw.close()
        (out/'responses.json').write_text(json.dumps(responses,ensure_ascii=False,indent=2),encoding='utf-8')
    columns=['side','permit_epoch','total','nonzero','hard','range','acquiring','stop','downstream_zero','profile_or_direction_zero','longest_zero_ticks']
    counts=[]
    for response in responses:
        for line in response['response'].splitlines():
            if line.startswith('ADCOUNTS,'):
                values=line.split(',')[1:]
                if len(values)==len(columns):counts.append(dict(host_time=response['host_time'],**dict(zip(columns,map(int,values)))))
    (out/'counts.json').write_text(json.dumps(counts,indent=2),encoding='utf-8')
    print(f'Read-only capture saved: {out}; counter snapshots={len(counts)}; no reset/START/enable/config writes')
    if not counts:print('No counters returned: the older candidate/firmware may not implement ADCOUNTS.')

if __name__=='__main__':main()
