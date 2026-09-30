"""Real TCP relay → UDP bridge → ROS services/topics, with NO robot writer."""

import asyncio
import json
from pathlib import Path
import socket
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

from vr_teleop_bridge.node import VRBridgeNode

# usb_link belongs to the desktop project, not the ROS package installation.
sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from usb_link.relay import Relay  # noqa: E402


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def test_tcp_loss_pauses_real_ros_bridge_until_explicit_neutral_resume(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    udp, discovery = free_udp_port(), free_udp_port()
    while discovery == udp:
        discovery = free_udp_port()
    rclpy.init(domain_id=92, args=[
        '--ros-args', '-p', 'bind_address:=127.0.0.1',
        '-p', f'udp_port:={udp}', '-p', f'discovery_port:={discovery}',
        '-p', 'allowed_source_ip:=127.0.0.1',
        '-p', 'activation_mode:=session_arm',
        '-p', 'pause_on_packet_timeout:=true',
    ])
    node = VRBridgeNode()
    velocities = []
    node.create_subscription(TwistStamped, '/vr/cmd_vel', velocities.append, 10)
    clients = {action: node.create_client(Trigger, '/vr/teleop/' + action)
               for action in ('arm_session', 'resume_session')}

    async def check():
        relay = Relay(udp)
        server = await asyncio.start_server(relay.client, '127.0.0.1', 0)
        tcp = server.sockets[0].getsockname()[1]
        state = {'seq': 0, 'stick': 0.0}
        tasks = []

        async def headset():
            reader, writer = await asyncio.open_connection('127.0.0.1', tcp)
            try:
                while True:
                    token = (await reader.readline()).strip()
                    if not token:
                        return
                    state['seq'] += 1
                    pose = {'p': [0, 1, 0], 'q': [0, 0, 0, 1]}
                    packet = dict(v=1, seq=state['seq'], client_time_ms=int(time.time()*1000),
                                  left=pose, right=pose, head=pose,
                                  sticks={'left': [0, state['stick']], 'right': [0, 0]},
                                  triggers=[0, 0], deadman=False,
                                  tracking={'left': True, 'right': True, 'head': True})
                    writer.write(token + b' ' + json.dumps(packet).encode() + b'\n')
                    await writer.drain()
                    await asyncio.sleep(0.02)
            finally:
                writer.close()
                await writer.wait_closed()

        async def pump(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0)
                await asyncio.sleep(0.002)

        async def service(action):
            future = clients[action].call_async(Trigger.Request())
            for _ in range(100):
                await pump(0.02)
                if future.done():
                    return future.result()
            raise AssertionError('ROS service timed out: ' + action)

        try:
            tasks.append(asyncio.create_task(headset()))
            await pump(1.0)
            assert node._valid_packet_count > 20
            assert (await service('arm_session')).success
            state['stick'] = 0.7
            await pump(0.3)
            assert node._active and any(msg.twist.linear.x > 0 for msg in velocities[-5:])

            tasks[-1].cancel()  # emulate USB/TCP loss; no physical command path
            await asyncio.gather(tasks[-1], return_exceptions=True)
            await pump(0.6)
            assert node._session_gate.armed and not node._active
            assert velocities[-1].twist.linear.x == 0

            tasks.append(asyncio.create_task(headset()))
            await pump(0.5)
            assert time.monotonic() - node._last_valid_monotonic < 0.1
            assert not node._active and velocities[-1].twist.linear.x == 0
            assert not (await service('resume_session')).success  # stick held
            state['stick'] = 0.0
            await pump(0.2)
            assert not node._active  # neutral + fresh never auto-resumes
            assert (await service('resume_session')).success
            await pump(0.2)
            assert node._active
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            server.close()
            await server.wait_closed()
            await asyncio.sleep(0.02)
            relay.udp.close()

    try:
        asyncio.run(check())
    finally:
        node.destroy_node()
        rclpy.shutdown()
