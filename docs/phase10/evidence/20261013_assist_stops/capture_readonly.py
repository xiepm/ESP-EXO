"""本次助力中断诊断：只读状态，不复位、不启停、不改参数。"""
import pathlib
import re
import time
import serial
import serial.tools.list_ports

root = pathlib.Path(__file__).resolve().parent
stamp = time.strftime('%Y%m%d-%H%M%S')
ports = [p for p in serial.tools.list_ports.comports()
         if p.vid == 0x0403 and p.pid == 0x6001 and p.serial_number == 'A5069RR4A']
if len(ports) != 1:
    raise RuntimeError(f'Expected one known FTDI port, got {len(ports)}')
console = serial.Serial(port=None, baudrate=115200, timeout=0.08, write_timeout=1)
console.dtr = False
console.rts = False
console.port = ports[0].device
chunks = [f'READ ONLY {stamp}; port={console.port}; DTR/RTS=False before open\n']

def receive(seconds):
    until = time.monotonic() + seconds
    blocks = []
    while time.monotonic() < until:
        block = console.read(max(1, console.in_waiting))
        if block:
            blocks.append(block)
    return b''.join(blocks).decode('utf-8', errors='replace')

try:
    console.open()
    chunks.append(receive(0.3))
    for command in ('system_status', 'control_status', 'assist_status',
                    'sensor_status', 'gait_status', 'can_status', 'ble_status',
                    'joint_status left', 'joint_status right', 'system_status',
                    'assist_status', 'can_status'):
        chunks.append(f'\n>>> {command}\n')
        console.write((command + '\r\n').encode('ascii'))
        chunks.append(receive(0.8))
finally:
    console.close()
    chunks.append('\nSERIAL CLOSED\n')
    report = ''.join(chunks).replace('\r', '')
    path = root / f'{stamp}-uart.log'
    path.write_text(report, encoding='utf-8')
    print(f'Log: {path}')
    cleaned = re.sub(r'\x1b\[[0-9;]*m', '', report)
    summary = '\n'.join(line for line in cleaned.splitlines()
        if line.startswith(('READ ONLY', '>>>', '=====', '  ', 'SERIAL CLOSED'))
        and not any(tag in line for tag in ('NimBLE:', 'att_handle=')))
    (root / f'{stamp}-status-extract.log').write_text(summary, encoding='utf-8')
    print(summary)
