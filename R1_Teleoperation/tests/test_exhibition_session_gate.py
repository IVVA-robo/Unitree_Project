"""Gate decisions without ROS participants or a connected robot."""
from pathlib import Path
import runpy

import pytest

pytest.importorskip('rclpy')
client_module = runpy.run_path(str(
    Path(__file__).resolve().parents[1] / 'scripts/r1-exhibition-ros-client'
))
run_gate = client_module['run_gate']


class Client:
    def __init__(self, samples):
        self.samples = iter(samples)
        self.calls = []
        self.active = self.armed = self.writer_armed = self.kill = None

    def spin_until(self, predicate, deadline):
        for key, value in next(self.samples, {}).items():
            setattr(self, key, value)
        return predicate()

    def trigger(self, service, deadline):
        self.calls.append(service)
        return True, 'accepted'


def test_healthy_resume_requires_fresh_writer_and_kill_observations():
    client = Client([{'kill': False}, {
        'active': True, 'armed': True, 'writer_armed': True, 'kill': False,
    }])
    assert run_gate(client, 'resume_ready', 1) == 0
    assert client.calls == ['/vr/teleop/resume_session']


@pytest.mark.parametrize('missing', ['writer_armed', 'kill', 'active', 'armed'])
def test_missing_fresh_observation_does_not_authorize_run(missing):
    sample = dict(active=True, armed=True, writer_armed=True, kill=False)
    del sample[missing]
    client = Client([{'kill': False}, sample])
    assert run_gate(client, 'resume_ready', 1) == 2
    assert client.calls == ['/vr/teleop/resume_session']


def test_latched_stop_requires_explicit_owner_recovery_before_resume():
    client = Client([{'kill': True}])
    assert run_gate(client, 'resume_ready', 1) == 3
    assert client.calls == []


def test_missing_safety_state_never_requests_emergency_clear():
    client = Client([{}])
    assert run_gate(client, 'resume_ready', 1) == 2
    assert client.calls == []


def test_emergency_during_resume_cannot_be_reported_as_ready():
    client = Client([{'kill': False}, {
        'active': True, 'armed': True, 'writer_armed': True, 'kill': True,
    }])
    assert run_gate(client, 'resume_ready', 1) == 2


def test_preparation_resume_does_not_require_already_prepared_writer():
    client = Client([{'active': True, 'armed': True, 'writer_armed': False}])
    assert run_gate(client, 'resume', 1) == 0


def test_emergency_clear_acknowledges_already_clear_but_paused_bridge():
    client = Client([])
    client.active = False
    assert run_gate(client, 'clear_emergency', 1) == 0
    assert client.calls == ['/vr/teleop/clear_emergency_stop']


def test_pause_needs_fresh_state_after_reply():
    client = Client([{}])
    client.active, client.armed = False, True
    assert run_gate(client, 'pause', 1) == 2
    assert client.calls == ['/vr/teleop/pause_session']


@pytest.mark.parametrize('sample', [{'active': False, 'armed': True},
                                   {'active': True, 'armed': True},
                                   {'active': False, 'armed': False}])
def test_pause_confirmation_requires_both_flags(sample):
    assert run_gate(Client([sample]), 'pause', 1) == (0 if sample == {'active': False, 'armed': True} else 2)


def make_trigger_client(responses):
    from types import SimpleNamespace
    import time

    class Future:
        def __init__(self, response):
            self.response = response

        def done(self):
            return self.response is not None

        def exception(self):
            return None

        def result(self):
            return self.response

    class Service:
        def __init__(self):
            self.requests = 0

        def service_is_ready(self):
            return True

        def call_async(self, _request):
            value = responses[self.requests]
            self.requests += 1
            return Future(None if value is None else SimpleNamespace(success=value, message='reply'))

    service = Service()
    destroyed = []
    client = client_module['ExhibitionClient'].__new__(client_module['ExhibitionClient'])
    client.node = SimpleNamespace(create_client=lambda *_: service,
                                  destroy_client=lambda target: destroyed.append(target))
    client.cancelled = False
    client.spin_until = lambda predicate, deadline: predicate()
    return client, service, destroyed, time.monotonic() + 8


def test_lost_pause_reply_retried_once_and_client_cleaned():
    client, service, destroyed, deadline = make_trigger_client([None, True])
    assert client.trigger('/vr/teleop/pause_session', deadline) == (True, 'reply')
    assert service.requests == 2
    assert destroyed == [service]


def test_both_pause_replies_missing_stays_blocked():
    client, service, destroyed, deadline = make_trigger_client([None, None])
    assert not client.trigger('/vr/teleop/pause_session', deadline)[0]
    assert service.requests == 2
    assert destroyed == [service]


def test_rejected_pause_is_not_retried():
    client, service, _, deadline = make_trigger_client([False])
    assert client.trigger('/vr/teleop/pause_session', deadline) == (False, 'reply')
    assert service.requests == 1


@pytest.mark.parametrize('name', ['/vr/teleop/resume_session', '/vr/teleop/arm_session',
                                  '/vr/teleop/clear_emergency_stop', '/vr/calibrate_body'])
def test_lost_motion_enabling_or_calibration_reply_never_retried(name):
    client, service, destroyed, deadline = make_trigger_client([None])
    assert not client.trigger(name, deadline)[0]
    assert service.requests == 1
    assert destroyed == [service]


def test_cancelled_pause_cannot_retry():
    client, service, _, deadline = make_trigger_client([None])
    client.cancelled = True
    assert not client.trigger('/vr/teleop/pause_session', deadline)[0]
    assert service.requests == 0
