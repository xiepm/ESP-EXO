"""Capture stage-5 status without motion, reset, or configuration writes."""
from datetime import datetime
from pathlib import Path
from nominal_peak_run import Console
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', action='store_true')
    args = parser.parse_args()
    destination = Path(__file__).resolve().parents[1] / 'output/adaptive/20261104' / ('stage5_preflight_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    destination.mkdir()
    print(destination)
    console = Console(destination)
    try:
        console.receive(.5)
        for command in ('system_status', 'ble_status', 'control_status', 'assist_status', 'adaptive_status', 'bench_peak_status', 'can_status', 'joint_status left', 'joint_status right'):
            print(console.command(command))
        if args.baseline:
            from nominal_peak_run import verify_closed_idle
            verify_closed_idle(console.command('system_status'))
            for _ in range(40):
                for side in ('left', 'right'):
                    console.command('joint_status ' + side)
            print(console.command('can_status'))
            print(console.command('system_status'))
    finally:
        console.close()


if __name__ == '__main__':
    main()
