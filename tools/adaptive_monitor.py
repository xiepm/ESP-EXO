"""只读AFO诊断采集；不START、不使能、不修改参数。离线parse无需串口。

capture打开指定端口前将DTR/RTS设为False；仍须在台架确认USB桥不会复位。
UART输出在低优先级任务执行。原始日志永久保留，解析失败不伪造数值。
"""
import argparse
import csv
import re
import time
from pathlib import Path

FIELDS = ("timestamp_us,side,system_state,q_deg,v_deg_s,motor_estimate_nm,feedback_valid,"
          "raw_phase,corrected_phase,hz,period_s,confidence,event,locked,flex_nm,ext_nm,"
          "desired_nm,limited_preview_nm,final_ff_command_nm,command_valid,faults,rearm,"
          "generation,execution_us,dropped,event_timestamp_us,event_phase_before,"
          "event_phase_after,phase_offset_rad,accepted_events,rejected_events,lambda,"
          "assist_enabled,previous_control_execution_us").split(",")


def parse(source, destination):
    count, malformed = 0, 0
    with destination.open("w", encoding="utf-8", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(FIELDS)
        for line in source.read_text(encoding="utf-8", errors="replace").splitlines():
            match = re.search(r"ADAPT,(-?\d+,.*)", line)
            if not match:
                continue
            values = match[1].strip().split(",")
            try:
                if len(values) != len(FIELDS):
                    raise ValueError("column count")
                for value in values:
                    float(value)
                if int(values[1]) not in (0, 1):
                    raise ValueError("side")
            except ValueError:
                malformed += 1
                continue
            writer.writerow(values)
            count += 1
    print(f"parsed_rows={count} malformed_rows={malformed} csv={destination}")
    return count, malformed


def capture(port, seconds, destination):
    import serial
    if not 1 <= seconds <= 3600:
        raise ValueError("seconds must be 1..3600")
    # 显式端口，不扫描/自动重连；不复位以保留现场故障证据。
    connection = serial.Serial(port=None, baudrate=115200, timeout=0.1, write_timeout=1)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    try:
        connection.open()
        with destination.open("wb") as raw:
            start = time.monotonic()
            next_dump = start
            connection.write(b"\radaptive_status\r")
            while time.monotonic() - start < seconds:
                now = time.monotonic()
                if now >= next_dump:
                    connection.write(b"adaptive_dump\r")
                    next_dump = now + 1.0
                chunk = connection.read(max(1, min(connection.in_waiting, 4096)))
                raw.write(chunk)
    finally:
        connection.close()
    parse(destination, destination.with_suffix(".csv"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="action", required=True)
    offline = sub.add_parser("parse")
    offline.add_argument("source", type=Path)
    offline.add_argument("destination", type=Path)
    live = sub.add_parser("capture")
    live.add_argument("--port", required=True)
    live.add_argument("--seconds", type=int, default=60)
    live.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.action == "parse":
        parse(args.source, args.destination)
    else:
        capture(args.port, args.seconds, args.output)
