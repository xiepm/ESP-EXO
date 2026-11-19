"""Motion-free 20Hz synchronized capture; never enables motors or writes zeros."""
import asyncio
import json
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from bleak import BleakClient, BleakScanner
import phase10_ble_acceptance as p
from phase6_ble_acceptance import parse_system_status


async def main(dest=None, seconds=30, expected_boot=0x6A079617, announce_started=False):
    dest = dest or Path(__file__).resolve().parents[1]/'output/adaptive/20261104'/('stationary_ble_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    dest.mkdir()
    result = {'frames': [], 'requests': [], 'error': None, 'decode_errors': []}
    try:
        device = await p.resolve_device(BleakScanner, '44:1B:F6:FE:22:C2', 'ESP-EXO', 5)
        async with BleakClient(device, timeout=15) as client:
            transport = p.BleTransport(client, 5)
            ids = p.RequestIds()
            async def request(opcode, payload=b''):
                assert opcode in (1, 2, 8, 0x17, 0x18)
                frame = await p._transact_frame(transport, opcode, ids.take(), payload)
                result['requests'].append({'opcode': opcode, 'payload': payload.hex(), 'response_payload': frame.payload.hex()})
                return frame
            def callback(sender, data):
                try:
                    result['frames'].append({'wire': bytes(data).hex(), **asdict(p.parse_telemetry_packet(bytes(data)))})
                except Exception as exc:
                    result['decode_errors'].append({'wire': bytes(data).hex(), 'error': repr(exc)})
            await client.start_notify(p.RESPONSE_CHARACTERISTIC_UUID, transport.notification_callback)
            await client.start_notify(p.TELEMETRY_CHARACTERISTIC_UUID, callback)
            info = p.parse_info(await request(1))
            result['info'] = asdict(info)
            p.require(info.boot_id == expected_boot and info.firmware in ((0,11,4),(0,11,5)), 'Unexpected boot/version')
            def closed(frame):
                s = parse_system_status(frame)
                p.require(s.state == 1 and s.flags & 0x1c == 0, 'Not closed IDLE')
                return asdict(s)
            result['before'] = closed(await request(2))
            initial = p.parse_telemetry_status(await request(8))
            p.require(not initial.streaming, 'Another stream is active')
            attempted = False
            try:
                attempted = True
                ack = await request(0x17, p.make_start_payload(20, 7))
                p.require(ack.payload[0] == 0, 'Telemetry start rejected')
                result['capture_started_at'] = datetime.now().astimezone().isoformat()
                if announce_started:
                    print('TELEMETRY_STARTED: motor output remains disabled; recording for '+str(seconds)+' seconds', flush=True)
                await asyncio.sleep(seconds)
                result['running'] = asdict(p.parse_telemetry_status(await request(8)))
            finally:
                if attempted:
                    await request(0x18)
                    result['stopped'] = asdict(p.parse_telemetry_status(await request(8)))
            result['after'] = closed(await request(2))
            result['final_info'] = asdict(p.parse_info(await request(1)))
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        (dest/'capture.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps({'directory': str(dest), 'frames': len(result['frames']), 'error': result['error']}))
    return result


if __name__ == '__main__':
    asyncio.run(main())
