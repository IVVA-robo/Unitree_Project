"""Exercise real TCP framing and freshness rather than source-code patterns."""

import asyncio
import json

import pytest

from usb_link.relay import read_pose


def packet():
    pose = {'p': [0, 1, 0], 'q': [0, 0, 0, 1]}
    return json.dumps({'v': 1, 'seq': 1, 'client_time_ms': 1,
                       'left': pose, 'right': pose, 'head': pose,
                       'sticks': {'left': [0, 0], 'right': [0, 0]},
                       'triggers': [0, 0], 'deadman': False,
                       'tracking': {'left': True, 'right': True, 'head': True}}).encode()


async def exchange(mode):
    tasks = []

    async def headset(reader, writer):
        try:
            token = (await reader.readline()).strip()
            if mode == 'slow':
                await asyncio.sleep(0.08)
            if mode == 'replay':
                token = b'0' * 32
            payload = b'{}' if mode == 'invalid' else packet()
            frame = token + b' ' + payload + b'\n'
            # TCP may split both token and JSON anywhere.
            writer.write(frame[:13])
            await writer.drain()
            await asyncio.sleep(0.001)
            writer.write(frame[13:])
            await writer.drain()
        except ConnectionError:
            pass
        finally:
            writer.close()

    def connected(reader, writer):
        tasks.append(asyncio.create_task(headset(reader, writer)))

    server = await asyncio.start_server(connected, '127.0.0.1', 0)
    reader, writer = await asyncio.open_connection('127.0.0.1', server.sockets[0].getsockname()[1])
    try:
        return await read_pose(reader, writer, max_age=0.04 if mode == 'slow' else 0.2)
    finally:
        writer.close()
        await writer.wait_closed()
        server.close()
        await server.wait_closed()
        await asyncio.gather(*tasks)


def test_usb_fragmented_fresh_pose():
    payload, elapsed = asyncio.run(exchange('fresh'))
    assert json.loads(payload)['tracking']['head'] is True
    assert elapsed < 0.2


@pytest.mark.parametrize('mode', ['replay', 'invalid', 'slow'])
def test_usb_refuses_replayed_invalid_or_delayed_pose(mode):
    with pytest.raises((ValueError, asyncio.TimeoutError)):
        asyncio.run(exchange(mode))
