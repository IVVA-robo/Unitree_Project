"""Pull fresh poses over ADB TCP and deliver them to a local diagnostic bridge.

ADB reverse only carries TCP, not the existing UDP protocol. Every pose must
answer a fresh, random challenge within 200 ms. Delayed TCP data is discarded
by closing the connection; reconnect never replays a buffered pose.
"""

import argparse
import asyncio
import json
import logging
from pathlib import Path
import secrets
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/vr_teleop_bridge'))
from vr_teleop_bridge.protocol import parse_packet  # noqa: E402

MAX_LINE = 4096
MAX_AGE = 0.20
LOG = logging.getLogger('r1-usb')


async def read_pose(reader, writer, max_age=MAX_AGE):
    """Return a newly requested valid packet, or raise without forwarding it."""
    token = secrets.token_hex(16).encode('ascii')
    started = time.monotonic()
    writer.write(token + b'\n')
    await asyncio.wait_for(writer.drain(), max_age)
    line = await asyncio.wait_for(reader.readline(), max_age)
    elapsed = time.monotonic() - started
    if elapsed > max_age:
        raise ValueError('USB pose exceeded freshness deadline')
    if not line.endswith(b'\n') or len(line) > MAX_LINE:
        raise ValueError('incomplete/oversized USB frame')
    received_token, separator, payload = line[:-1].partition(b' ')
    if not separator or received_token != token:
        raise ValueError('USB challenge mismatch; refusing queued/replayed pose')
    parse_packet(payload)
    return payload, elapsed


class Relay:
    def __init__(self, udp_port):
        self.target = ('127.0.0.1', udp_port)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(('127.0.0.1', 0))
        self.busy = False
        self.total = 0

    async def client(self, reader, writer):
        if self.busy:
            writer.close()
            await writer.wait_closed()
            return
        self.busy = True
        count, last_report, max_rtt = 0, time.monotonic(), 0.0
        LOG.info('USB client connected; target=%s:%s', *self.target)
        try:
            while True:
                payload, elapsed = await read_pose(reader, writer)
                self.udp.sendto(payload, self.target)
                self.total += 1
                count += 1
                max_rtt = max(max_rtt, elapsed)
                now = time.monotonic()
                if now - last_report >= 5:
                    data = json.loads(payload)
                    LOG.info('poses=%d rate=%.1fHz max_rtt=%.1fms tracking=%s deadman=%s',
                             self.total, count / (now - last_report), max_rtt * 1000,
                             data.get('tracking'), data.get('deadman'))
                    count, last_report, max_rtt = 0, now, 0.0
                # At most one outstanding request; no growing TCP pose queue.
                await asyncio.sleep(0.001)
        except (ValueError, OSError, asyncio.TimeoutError) as error:
            LOG.warning('USB stream stopped: %s; no further UDP poses emitted',
                        str(error) or type(error).__name__)
        finally:
            self.busy = False
            writer.close()
            try:
                await writer.wait_closed()
            except OSError:
                pass


async def serve(port, udp_port):
    relay = Relay(udp_port)
    try:
        server = await asyncio.start_server(relay.client, '127.0.0.1', port,
                                            limit=MAX_LINE)
        async with server:
            LOG.info('ADB TCP listener=127.0.0.1:%s, diagnostic UDP=127.0.0.1:%s',
                     port, udp_port)
            await server.serve_forever()
    finally:
        relay.udp.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=19092)
    parser.add_argument('--udp-port', type=int, default=19090)
    args = parser.parse_args()
    if not all(1024 <= port <= 65535 for port in (args.port, args.udp_port)):
        parser.error('ports must be in 1024..65535')
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    try:
        asyncio.run(serve(args.port, args.udp_port))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
