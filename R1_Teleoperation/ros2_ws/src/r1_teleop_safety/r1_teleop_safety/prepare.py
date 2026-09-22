"""Fail-closed stand/ready preparation command for a future R1 transport."""

import argparse
import sys

from .environment import ActuationPolicy


def prepare_result(policy, requested_mode):
    """Return ``(exit_code, text)`` without creating a robot command client."""
    if requested_mode not in ('ready', 'stand', 'damp'):
        return 2, f'unsupported prepare mode: {requested_mode}'
    if policy.dry_run:
        return (
            0,
            f'DRY-RUN: would request official R1 high-level {requested_mode}; '
            'no SDK client or motor command was created. ' + policy.summary(),
        )
    if not policy.live_authorized:
        missing = ', '.join(policy.missing_live_interlocks())
        return 2, f'LIVE BLOCKED: missing {missing}; no command was sent.'
    return (
        3,
        'LIVE BLOCKED: physical R1 prepare transport is intentionally not '
        'implemented yet; no command was sent. ' + policy.summary(),
    )


def main(argv=None):
    """Print a safe prepare plan; never instantiate a Unitree writer."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--mode', choices=('ready', 'stand', 'damp'), default='ready'
    )
    arguments = parser.parse_args(argv)
    try:
        policy = ActuationPolicy.from_environment()
    except ValueError as exc:
        print(f'INVALID ENVIRONMENT: {exc}', file=sys.stderr)
        return 2
    code, text = prepare_result(policy, arguments.mode)
    print(text)
    return code


if __name__ == '__main__':  # pragma: no cover - console entry point
    raise SystemExit(main())
