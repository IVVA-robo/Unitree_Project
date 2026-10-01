"""Read-only typed ROS readiness with one discovery participant per action."""

import argparse
import os
import time

from .readiness_attestation import issue


def missing_graph(services, topics, expected_services, expected_topics):
    """Require the exact type, including rejection of conflicting type lists."""
    actual = {'service': dict(services), 'topic': dict(topics)}
    return [f'{kind} {name} [{kind_type}]'
            for kind, entries in (('service', expected_services), ('topic', expected_topics))
            for name, kind_type in entries
            if set(actual[kind].get(name, [])) != {kind_type}]


def parameters_match(values, expected):
    """Match exact ROS parameter types without stringifying secret values."""
    if len(values) != len(expected):
        return False
    for value, wanted in zip(values, expected.values()):
        # rcl_interfaces/msg/ParameterType: BOOL=1, STRING=4.
        if isinstance(wanted, bool):
            if value.type != 1 or value.bool_value is not wanted:
                return False
        elif isinstance(wanted, str):
            if value.type != 4 or value.string_value != wanted:
                return False
        else:
            return False
    return True


def wait_ready(node, executor, services, topics, parameters, deadline):
    from rcl_interfaces.srv import GetParameters

    client = node.create_client(GetParameters, '/r1_live_writer/get_parameters') if parameters else None
    future = None
    next_query = 0.0
    missing = ['DDS discovery']
    while time.monotonic() < deadline:
        missing = missing_graph(node.get_service_names_and_types(),
                                node.get_topic_names_and_types(), services, topics)
        if not missing:
            if not parameters:
                return True, []
            if future is None and time.monotonic() >= next_query and client.service_is_ready():
                request = GetParameters.Request()
                request.names = list(parameters)
                future = client.call_async(request)
            if future is not None and future.done():
                response = future.result() if future.exception() is None else None
                if response is not None and parameters_match(response.values, parameters):
                    # Confirm the graph is still complete after the response.
                    missing = missing_graph(node.get_service_names_and_types(),
                                            node.get_topic_names_and_types(), services, topics)
                    if not missing:
                        return True, []
                future = None
                next_query = time.monotonic() + 0.25
            missing = ['live writer parameters not confirmed']
        executor.spin_once(timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
    return False, missing


def graph_entry(raw):
    fields = raw.split('|')
    if len(fields) != 2 or not fields[0].startswith('/') or not fields[1]:
        raise argparse.ArgumentTypeError('expected /name|package/type/Name')
    return tuple(fields)


def parameter_entry(raw):
    fields = raw.split('|')
    if len(fields) != 2 or not fields[0] or fields[1] not in {'true', 'false'}:
        raise argparse.ArgumentTypeError('expected parameter|true or parameter|false')
    return fields[0], fields[1] == 'true'


def string_parameter_entry(raw):
    fields = raw.split('|', 1)
    if len(fields) != 2 or not fields[0] or not fields[1]:
        raise argparse.ArgumentTypeError('expected parameter|nonempty-string')
    return fields[0], fields[1]


def main():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, required=True)
    parser.add_argument('--mode', choices=('static', 'control'), required=True)
    parser.add_argument('--service', action='append', type=graph_entry, default=[])
    parser.add_argument('--topic', action='append', type=graph_entry, default=[])
    parser.add_argument('--parameter', action='append', type=parameter_entry, default=[])
    parser.add_argument(
        '--string-parameter', action='append', type=string_parameter_entry,
        default=[],
    )
    parser.add_argument('--token-parameter')
    parser.add_argument('--attestation-path')
    parser.add_argument('--session-id')
    parser.add_argument('--domain-id', type=int)
    args = parser.parse_args()
    if not 0 < args.timeout_sec <= 120 or not args.service or not args.topic:
        parser.error('bounded timeout and nonempty typed graph are required')
    if args.mode == 'static' and not args.parameter:
        parser.error('static readiness requires featureless parameter checks')
    attestation_fields = (
        args.attestation_path, args.session_id, args.domain_id,
        args.token_parameter,
    )
    if any(value is not None for value in attestation_fields) \
            and not all(value is not None for value in attestation_fields):
        parser.error('attestation path/session/domain/token parameter are atomic')
    if args.token_parameter not in {None, 'commissioning_token'}:
        parser.error('only commissioning_token may bind readiness')
    parameters = dict(args.parameter)
    for name, value in args.string_parameter:
        if name in parameters:
            parser.error(f'duplicate parameter expectation: {name}')
        parameters[name] = value
    token = ''
    if args.token_parameter:
        token = os.environ.get('ROBOT_COMMISSIONING_TOKEN', '')
        if len(token) < 16:
            parser.error('commissioning token is unavailable')
        if args.token_parameter in parameters:
            parser.error('commissioning token expectation must not be on argv')
        parameters[args.token_parameter] = token
    deadline = time.monotonic() + args.timeout_sec
    context = Context()
    node = executor = None
    try:
        rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node(f'r1_readiness_{os.getpid()}', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        okay, missing = wait_ready(node, executor, args.service, args.topic,
                                   parameters, deadline)
        if not okay:
            print('[BLOCKED] graph readiness timed out; missing: ' + ', '.join(missing), flush=True)
            return 2
        if args.attestation_path:
            issue(
                args.attestation_path,
                token=token,
                session_id=args.session_id,
                mode=args.mode,
                domain_id=args.domain_id,
            )
        print(f'[OK] exhibition {args.mode} graph ready; typed services={len(args.service)} '
              f'topics={len(args.topic)}; single discovery participant', flush=True)
        return 0
    except Exception as error:
        print(f'[BLOCKED] graph readiness failed: {error}', flush=True)
        return 2
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=0.1)
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
