import threading
from types import SimpleNamespace

import pytest
from sensor_msgs.msg import JointState

from r1_hardware_adapter import state_monitor
from r1_hardware_adapter.state_monitor import (
    R1_IDL_MOTOR_INDICES,
    R1_JOINT_NAMES,
    R1StateMonitor,
    extract_joint_feedback,
    extract_joint_positions,
    state_summary,
)


def _state(values, velocities=None, efforts=None):
    if velocities is None:
        velocities = [0.0] * len(values)
    if efforts is None:
        efforts = [0.0] * len(values)
    motors = [
        SimpleNamespace(
            q=0.0,
            dq=0.0,
            tau_est=0.0,
            temperature=(),
            mode=0,
            motorstate=0,
            vol=0.0,
        )
        for _ in range(max(R1_IDL_MOTOR_INDICES) + 1)
    ]
    for joint, (index, value) in enumerate(
        zip(R1_IDL_MOTOR_INDICES, values)
    ):
        motors[index].q = value
        motors[index].dq = velocities[joint]
        motors[index].tau_est = efforts[joint]
    return SimpleNamespace(motor_state=motors)


def test_r1_mapping_has_26_unique_joint_names():
    assert len(R1_JOINT_NAMES) == 26
    assert len(set(R1_JOINT_NAMES)) == 26
    assert len(R1_IDL_MOTOR_INDICES) == 26


def test_extract_joint_positions_uses_reserved_idl_slots():
    values = [float(index) for index in range(26)]
    assert extract_joint_positions(_state(values)) == pytest.approx(values)


def test_extract_joint_feedback_maps_position_velocity_and_effort():
    positions = [float(index) for index in range(26)]
    velocities = [index / 10.0 for index in range(26)]
    efforts = [-float(index) for index in range(26)]
    feedback = extract_joint_feedback(
        _state(positions, velocities=velocities, efforts=efforts)
    )
    assert feedback is not None
    assert feedback[0] == pytest.approx(positions)
    assert feedback[1] == pytest.approx(velocities)
    assert feedback[2] == pytest.approx(efforts)


def test_extract_joint_feedback_ignores_reserved_idl_slots():
    state = _state([0.0] * 26)
    state.motor_state[14].q = float('nan')
    state.motor_state[14].dq = float('inf')
    state.motor_state[14].tau_est = -float('inf')
    assert extract_joint_feedback(state) is not None


def test_extract_rejects_short_or_nonfinite_state():
    assert extract_joint_positions(SimpleNamespace(motor_state=[])) is None
    bad = _state([0.0] * 26)
    bad.motor_state[R1_IDL_MOTOR_INDICES[3]].q = float('nan')
    assert extract_joint_positions(bad) is None


@pytest.mark.parametrize(
    ('field', 'value'),
    (
        ('q', float('nan')),
        ('dq', float('inf')),
        ('tau_est', -float('inf')),
    ),
)
def test_extract_joint_feedback_rejects_nonfinite_array(field, value):
    state = _state([0.0] * 26)
    setattr(state.motor_state[R1_IDL_MOTOR_INDICES[3]], field, value)
    assert extract_joint_feedback(state) is None


def test_extract_joint_feedback_rejects_missing_field():
    state = _state([0.0] * 26)
    del state.motor_state[R1_IDL_MOTOR_INDICES[3]].dq
    assert extract_joint_feedback(state) is None


def test_publish_populates_joint_state_velocity_and_effort(monkeypatch):
    positions = [float(index) for index in range(26)]
    velocities = [index / 10.0 for index in range(26)]
    efforts = [-float(index) for index in range(26)]
    published = []
    statuses = []
    monitor = SimpleNamespace(
        _lock=threading.Lock(),
        _latest_state=_state(positions, velocities, efforts),
        _latest_arrival=10.0,
        _sample_count=7,
        _timeout=0.5,
        _joint_publisher=SimpleNamespace(publish=published.append),
        _publish_status=statuses.append,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: JointState().header.stamp
            )
        ),
    )
    monkeypatch.setattr(state_monitor.time, 'monotonic', lambda: 10.01)

    R1StateMonitor._publish(monitor)

    assert len(published) == 1
    assert published[0].name == list(R1_JOINT_NAMES)
    assert len(published[0].name) == 26
    assert published[0].position == pytest.approx(positions)
    assert published[0].velocity == pytest.approx(velocities)
    assert published[0].effort == pytest.approx(efforts)
    assert len(published[0].position) == 26
    assert len(published[0].velocity) == 26
    assert len(published[0].effort) == 26
    assert statuses[0].startswith('lowstate=ok samples=7 ')


def test_publish_rejects_invalid_feedback_without_joint_message(monkeypatch):
    state = _state([0.0] * 26)
    state.motor_state[R1_IDL_MOTOR_INDICES[3]].dq = float('nan')
    published = []
    statuses = []
    monitor = SimpleNamespace(
        _lock=threading.Lock(),
        _latest_state=state,
        _latest_arrival=10.0,
        _sample_count=7,
        _timeout=0.5,
        _joint_publisher=SimpleNamespace(publish=published.append),
        _publish_status=statuses.append,
    )
    monkeypatch.setattr(state_monitor.time, 'monotonic', lambda: 10.01)

    R1StateMonitor._publish(monitor)

    assert published == []
    assert statuses == ['lowstate=invalid_joint_array']


def test_state_summary_exposes_safety_relevant_read_only_fields():
    velocities = [(-1.0) ** joint * joint / 10.0 for joint in range(26)]
    efforts = [joint / 2.0 for joint in range(26)]
    state = _state([0.0] * 26, velocities, efforts)
    for joint, index in enumerate(R1_IDL_MOTOR_INDICES):
        state.motor_state[index].mode = 1
        state.motor_state[index].motorstate = 0
        state.motor_state[index].vol = 36.5
        state.motor_state[index].temperature = [20 + joint, 40 + joint]
    state.motor_state[R1_IDL_MOTOR_INDICES[1]].motorstate = 4
    state.motor_state[R1_IDL_MOTOR_INDICES[20]].motorstate = 7
    state.motor_state[14].motorstate = 99
    state.motor_state[14].temperature = [-1000, 1000]
    state.mode_machine = 1
    state.mode_pr = 0
    state.wireless_remote = [0] * 40
    state.imu_state = SimpleNamespace(temperature=80)
    text = state_summary(state, sample_count=10, age_sec=0.01)
    assert 'mode_machine=1' in text
    assert 'enabled_modes=26' in text
    assert 'motorstate_nonzero=2' in text
    assert 'motorstate=1:4,23:7' in text
    assert 'remote_nonzero=False' in text
    assert 'max_abs_dq=2.500' in text
    assert 'max_abs_tau_est=12.500' in text
    assert 'motor_temp=20..65' in text
    assert 'writer=disabled' in text


def test_state_summary_marks_unavailable_temperature_range():
    text = state_summary(_state([0.0] * 26), sample_count=1, age_sec=0.0)
    assert 'motor_temp=n/a' in text
