"""Tests for the read-only Unitree camera RPC source."""

from collections import deque
import threading
import time

import cv2
import numpy as np
import pytest

from r1_robot_pov.frame_pipeline import FrameHub, FrameProcessor
import r1_robot_pov.sources as sources
from r1_robot_pov.sources import (
    UnitreeVideoSource,
    _next_unitree_reconnect_delay,
)


def _jpeg(colour=(20, 80, 160), width=8, height=6):
    image = np.full((height, width, 3), colour, dtype=np.uint8)
    ok, encoded = cv2.imencode('.jpg', image)
    assert ok
    return encoded.tobytes()


def _wait_for(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.002)
    raise AssertionError('timed out waiting for video source worker')


class _FakeClient:
    def __init__(self, outcomes, *, repeat_last=False, delay_s=0.002):
        self._outcomes = deque(outcomes)
        self._last = outcomes[-1] if outcomes else None
        self._repeat_last = repeat_last
        self._delay_s = delay_s
        self.timeout = None
        self.init_calls = 0
        self.poll_calls = 0

    def SetTimeout(self, timeout):
        self.timeout = timeout

    def Init(self):
        self.init_calls += 1

    def GetImageSample(self):
        self.poll_calls += 1
        if self._delay_s:
            time.sleep(self._delay_s)
        if self._outcomes:
            outcome = self._outcomes.popleft()
            self._last = outcome
        elif self._repeat_last:
            outcome = self._last
        else:
            raise RuntimeError('fake response list exhausted')
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome


def test_unitree_reconnect_backoff_is_exponential_and_bounded():
    delays = []
    previous = None
    for _ in range(6):
        previous = _next_unitree_reconnect_delay(0.5, previous)
        delays.append(previous)

    assert delays == [0.5, 1.0, 2.0, 4.0, 5.0, 5.0]


def test_unitree_reconnect_backoff_resets_after_successful_frame():
    unavailable = (3102, b'')
    client = _FakeClient(
        [unavailable, unavailable, (0, _jpeg()), unavailable],
    )

    class RecordingStopEvent:
        def __init__(self):
            self.waits = []
            self.stopped = False

        def is_set(self):
            return self.stopped

        def wait(self, timeout):
            self.waits.append(timeout)
            if len(self.waits) == 3:
                self.stopped = True
            return self.stopped

    hub = FrameHub()
    source = UnitreeVideoSource(
        hub,
        network_interface='enx-test',
        rpc_timeout_s=0.05,
        reconnect_delay_s=0.5,
        channel_initializer=lambda *_: None,
        video_client_factory=lambda: client,
    )
    stop_event = RecordingStopEvent()
    source._stop_event = stop_event

    source._run()

    assert stop_event.waits == [0.5, 1.0, 0.5]
    assert client.poll_calls == 4
    assert hub.status()['total_updates'] == 1


def test_constructor_is_inert_and_worker_keeps_only_latest_processed_frame():
    first_jpeg = _jpeg((10, 40, 90))
    latest_jpeg = _jpeg((170, 70, 20))
    client = _FakeClient(
        [(0, first_jpeg), (0, latest_jpeg)],
        repeat_last=True,
    )
    calls = []
    main_thread = threading.get_ident()

    def initialize(domain_id, interface):
        calls.append(
            ('initialize', domain_id, interface, threading.get_ident())
        )

    def create_client():
        calls.append(('client', threading.get_ident()))
        return client

    hub = FrameHub()
    processor = FrameProcessor(flip_horizontal=True)
    source = UnitreeVideoSource(
        hub,
        processor,
        network_interface='enx-test',
        domain_id=3,
        rpc_timeout_s=0.05,
        reconnect_delay_s=0.001,
        channel_initializer=initialize,
        video_client_factory=create_client,
    )

    assert source.available
    assert source.availability_error is None
    assert calls == []
    assert client.init_calls == 0

    source.start()
    _wait_for(lambda: hub.status()['total_updates'] >= 2)
    source.stop(timeout=0.2)

    expected = processor.process(
        cv2.imdecode(
            np.frombuffer(latest_jpeg, dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
    )
    snapshot = hub.snapshot()
    assert snapshot is not None
    np.testing.assert_array_equal(snapshot.frame, expected)
    assert snapshot.source == 'unitree-video'
    assert calls[0][:3] == ('initialize', 3, 'enx-test')
    assert calls[0][3] != main_thread
    assert calls[1][0] == 'client'
    assert calls[1][1] != main_thread
    assert client.timeout == pytest.approx(0.05)
    assert client.init_calls == 1
    assert not source.running

    metrics = hub.status()['sources']['unitree-video']
    assert metrics['frames'] >= 2
    assert metrics['connected'] is False
    assert metrics['disconnects'] == 1


@pytest.mark.parametrize(
    ('failure', 'error_text'),
    [
        ((27, b''), 'code 27'),
        ((0, b''), 'empty JPEG'),
        ((0, b'not a JPEG'), 'malformed JPEG'),
        (RuntimeError('video RPC exploded'), 'video RPC exploded'),
    ],
)
def test_bad_samples_and_rpc_errors_reconnect_with_metrics(
    failure,
    error_text,
):
    client = _FakeClient([failure, (0, _jpeg())], repeat_last=True)
    factory_calls = []

    def create_client():
        factory_calls.append(True)
        return client

    hub = FrameHub()
    source = UnitreeVideoSource(
        hub,
        network_interface='enx-test',
        rpc_timeout_s=0.05,
        reconnect_delay_s=0.001,
        channel_initializer=lambda *_: None,
        video_client_factory=create_client,
    )

    source.start()
    _wait_for(lambda: hub.snapshot())
    source.stop(timeout=0.2)

    metrics = hub.status()['sources']['unitree-video']
    assert metrics['errors'] == 1
    assert metrics['reconnects'] == 1
    assert metrics['successful_reconnects'] == 1
    assert metrics['reported_dropped_frames'] == 1
    assert error_text in metrics['last_error']
    assert metrics['connected'] is False
    assert metrics['disconnects'] == 1
    assert len(factory_calls) == 1
    assert client.init_calls == 1
    assert client.poll_calls >= 2


def test_repeated_unavailable_service_recycles_stale_dds_client():
    unavailable = (3102, b'')
    client = _FakeClient(
        [unavailable, unavailable, unavailable, (0, _jpeg())],
        repeat_last=True,
    )
    factory_calls = []

    def create_client():
        factory_calls.append(True)
        return client

    hub = FrameHub()
    source = UnitreeVideoSource(
        hub,
        network_interface='enx-test',
        rpc_timeout_s=0.05,
        reconnect_delay_s=0.001,
        channel_initializer=lambda *_: None,
        video_client_factory=create_client,
    )

    source.start()
    _wait_for(lambda: hub.snapshot())
    source.stop(timeout=0.2)

    metrics = hub.status()['sources']['unitree-video']
    assert len(factory_calls) == 2
    assert client.init_calls == 2
    assert client.poll_calls >= 4
    assert metrics['errors'] == 3
    assert metrics['reconnects'] == 3
    assert metrics['successful_reconnects'] == 1
    assert metrics['reported_dropped_frames'] == 3


def test_client_recycle_closes_private_sdk_channels_when_available():
    calls = []

    class Channel:
        def CloseWriter(self):
            calls.append('writer')

        def CloseReader(self):
            calls.append('reader')

    class Stub:
        pass

    class Client:
        pass

    stub = Stub()
    setattr(stub, '_ClientStub__sendChannel', Channel())
    setattr(stub, '_ClientStub__recvChannel', Channel())
    client = Client()
    setattr(client, '_ClientBase__stub', stub)

    UnitreeVideoSource._close_client(client)

    assert calls == ['writer', 'reader']


def test_stop_during_bounded_rpc_poll_is_clean_and_does_not_reconnect():
    poll_started = threading.Event()

    class BlockingClient(_FakeClient):
        def GetImageSample(self):
            poll_started.set()
            time.sleep(0.04)
            return 0, _jpeg()

    client = BlockingClient([])
    hub = FrameHub()
    source = UnitreeVideoSource(
        hub,
        network_interface='enx-test',
        rpc_timeout_s=0.05,
        reconnect_delay_s=0.001,
        channel_initializer=lambda *_: None,
        video_client_factory=lambda: client,
    )

    source.start()
    assert poll_started.wait(0.2)
    started = time.monotonic()
    source.stop(timeout=0.2)

    assert time.monotonic() - started < 0.2
    assert not source.running
    assert hub.snapshot() is None
    metrics = hub.status()['sources']['unitree-video']
    assert metrics['connected'] is False
    assert metrics['errors'] == 0
    assert metrics['reconnects'] == 0


def test_missing_optional_sdk_is_reported_before_worker_start(monkeypatch):
    monkeypatch.setattr(sources, '_ChannelFactoryInitialize', None)
    monkeypatch.setattr(sources, '_VideoClient', None)
    source = UnitreeVideoSource(
        FrameHub(),
        network_interface='enx-test',
    )

    assert not source.available
    assert source.availability_error is not None
    with pytest.raises(RuntimeError, match='requires unitree_sdk2py'):
        source.start()
    assert not source.running
