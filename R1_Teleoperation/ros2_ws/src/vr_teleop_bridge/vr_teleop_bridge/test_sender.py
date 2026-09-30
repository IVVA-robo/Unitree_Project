"""Protocol-v1 UDP generator for bridge tests without a VR headset."""

import argparse
import json
import socket
import time


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=9090)
    parser.add_argument('--rate', type=float, default=72.0)
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--deadman', action='store_true')
    parser.add_argument('--simulate-drop', action='store_true')
    parser.add_argument('--left-x', type=float, default=0.0)
    parser.add_argument('--left-y', type=float, default=0.0)
    parser.add_argument('--right-x', type=float, default=0.0)
    parser.add_argument('--right-y', type=float, default=0.0)
    parser.add_argument('--left-trigger', type=float, default=0.0)
    parser.add_argument('--right-trigger', type=float, default=0.0)
    parser.add_argument('--press-x', action='store_true')
    parser.add_argument('--press-b', action='store_true')
    parser.add_argument('--drop-left', action='store_true')
    parser.add_argument('--drop-right', action='store_true')
    parser.add_argument('--drop-head', action='store_true')
    args = parser.parse_args()
    if args.rate <= 0.0 or args.duration < 0.0:
        parser.error('--rate must be positive and --duration must be non-negative')
    return args


def _packet(args, sequence, pose, deadman):
    return {
        'v': 1,
        'seq': sequence,
        'client_time_ms': time.time_ns() // 1_000_000,
        'left': pose,
        'right': pose,
        'head': {'p': [0.0, 0.0, 1.65], 'q': [0.0, 0.0, 0.0, 1.0]},
        'sticks': {
            'left': [args.left_x, args.left_y],
            'right': [args.right_x, args.right_y],
        },
        'triggers': [args.left_trigger, args.right_trigger],
        'tracking': {
            'left': not args.drop_left,
            'right': not args.drop_right,
            'head': not args.drop_head,
        },
        'buttons': {
            'left_x': args.press_x,
            'right_b': args.press_b,
        },
        'deadman': deadman,
    }


def _send(sock, address, packet):
    payload = json.dumps(packet, separators=(',', ':')).encode()
    sock.sendto(payload, address)


def main():
    args = _arguments()
    pose = {'p': [0.35, 0.0, 1.2], 'q': [0.0, 0.0, 0.0, 1.0]}
    address = (args.host, args.port)
    period = 1.0 / args.rate
    deadline = time.monotonic() + args.duration
    sequence = 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        while time.monotonic() < deadline:
            started = time.monotonic()
            _send(sock, address, _packet(args, sequence, pose, args.deadman))
            sequence = (sequence + 1) % (1 << 32)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass

    if not args.simulate_drop:
        release = _packet(args, sequence, pose, False)
        release['sticks'] = {'left': [0.0, 0.0], 'right': [0.0, 0.0]}
        _send(sock, address, release)
    sock.close()


if __name__ == '__main__':
    main()
