"""Read-only SDK reader discovery without repeated ROS CLI startups."""

import argparse
import math
import os
from pathlib import Path
import time

from .readiness import parameters_match

POLICY = {'sdk_enabled': True, 'dry_run': True, 'hardware_enabled': False,
          'commissioning_interlock': False, 'arm_writer_enabled': False,
          'locomotion_writer_enabled': False}


def finite_feedback(message):
    return (len(message.name) == 26 and len(set(message.name)) == 26
            and all(message.name) and len(message.position) == 26
            and all(math.isfinite(value) for value in message.position))


def local_reader_exists(domain, root=Path('/proc')):
    """Undiscovered live reader in this domain is NOT permission to duplicate."""
    for proc in root.iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = (proc / 'cmdline').read_bytes().split(b'\0')
            if not argv or Path(os.fsdecode(argv[0])).name != 'r1_sdk_transport':
                continue
            # Inspect only our SDK reader's domain; never log its environment.
            try:
                values = (proc / 'environ').read_bytes().split(b'\0')
            except PermissionError:
                return True
            reader_domain = next((value.split(b'=', 1)[1] for value in values
                                  if value.startswith(b'ROS_DOMAIN_ID=')), b'0')
            if int(reader_domain) == domain:
                return True
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (OSError, ValueError):
            return True  # cannot establish absence safely
    return False


def probe_reader(node, executor, deadline, domain, absent_grace=2.5):
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState

    started = time.monotonic()
    observed_reader = False
    latest = [None]

    def sample(message):
        latest[0] = time.monotonic() if finite_feedback(message) else None

    subscription = node.create_subscription(JointState, '/r1/sdk/joint_states', sample, 10)
    client = node.create_client(GetParameters, '/r1_sdk_transport/get_parameters')
    future = None
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            matches = sum(name == 'r1_sdk_transport' and namespace == '/'
                          for name, namespace in node.get_node_names_and_namespaces())
            graph = dict(node.get_service_names_and_types())
            types = graph.get('/r1_sdk_transport/get_parameters', [])
            if matches > 1:
                return 2, 'duplicate reader node names'
            # ROS may list a service name because OUR client exists. Only an
            # actual matched server (or the target node) proves prior presence.
            server_ready = client.service_is_ready()
            observed_reader |= bool(matches or server_ready)
            if matches and set(types) == {'rcl_interfaces/srv/GetParameters'}:
                if future is None and server_ready:
                    request = GetParameters.Request()
                    request.names = list(POLICY)
                    future = client.call_async(request)
                if future is not None and future.done():
                    response = future.result() if future.exception() is None else None
                    if response is None or not parameters_match(response.values, POLICY):
                        return 2, 'reader read-only boolean policy was not confirmed'
                    if (latest[0] is not None and now - latest[0] <= 0.5
                            and set(dict(node.get_topic_names_and_types()).get(
                                '/r1/sdk/joint_states', [])) == {'sensor_msgs/msg/JointState'}):
                        return 10, 'live read-only reader and fresh finite 26-joint feedback confirmed'
            if now - started >= absent_grace and not observed_reader:
                if local_reader_exists(domain):
                    return 2, 'reader process exists but its typed ROS graph is unavailable'
                return 0, 'no reader node/service/process in this domain'
            executor.spin_once(timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
        return 2, 'inconsistent/stale reader discovery or missing fresh finite feedback'
    finally:
        node.destroy_subscription(subscription)
        node.destroy_client(client)


def main():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=8)
    args = parser.parse_args()
    if not 3 <= args.timeout_sec <= 30:
        parser.error('timeout must be 3..30 seconds')
    context = Context()
    executor = node = None
    try:
        domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
        deadline = time.monotonic() + args.timeout_sec
        rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node(f'r1_reader_probe_{os.getpid()}', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        code, detail = probe_reader(node, executor, deadline, domain)
        print(f'[{"BLOCKED" if code == 2 else "OK"}] reader probe: {detail}', flush=True)
        return code
    except Exception as error:
        print(f'[BLOCKED] reader probe failed: {error}', flush=True)
        return 2
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=0.1)
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
