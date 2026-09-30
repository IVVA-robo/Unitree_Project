"""Contracts shared by the checked-in and buildable Pico pose senders."""

from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
SENDERS = (
    PROJECT / 'unity' / 'VRUdpSender.cs',
    PROJECT.parent / 'Unity_Projects' / 'Unitree_VR_Controller'
    / 'Assets' / 'VRUdpSender.cs',
)


def test_pico_sender_prefers_component_pose_validity_over_coarse_tracking():
    """Valid OpenXR prediction near the hips must not be discarded."""
    for path in SENDERS:
        source = path.read_text(encoding='utf-8')
        assert 'CommonUsages.trackingState' in source
        assert 'InputTrackingState.Position | InputTrackingState.Rotation' in source
        assert 'reportsPoseFlags ? completePose' in source
        assert 'runtimeTracked && hasPosition && hasRotation' in source


def test_pico_sender_still_requires_both_position_and_rotation():
    """A partial or stale pose must continue into bridge hold/zero handling."""
    for path in SENDERS:
        source = path.read_text(encoding='utf-8')
        assert 'CommonUsages.devicePosition' in source
        assert 'CommonUsages.deviceRotation' in source
        assert '(poseFlags & required) == required' in source


def test_pico_sender_probes_stick_aliases_and_logs_raw_axes():
    """A valid Pico controller must not silently collapse an alias to zero."""
    for path in SENDERS:
        source = path.read_text(encoding='utf-8')
        assert 'StickAxisUsages' in source
        assert 'CommonUsages.primary2DAxis' in source
        assert 'new InputFeatureUsage<Vector2>("Joystick")' in source
        assert 'new InputFeatureUsage<Vector2>("Thumbstick")' in source
        assert 'TryReadAxis' in source
        assert '#if ENABLE_INPUT_SYSTEM' in source
        assert 'TryReadInputSystemAxis' in source
        assert 'UnityEngine.InputSystem.InputSystem.devices' in source
        assert 'UnityEngine.InputSystem.CommonUsages.LeftHand' in source
        assert 'UnityEngine.InputSystem.CommonUsages.RightHand' in source
        assert 'leftStick=' in source
        assert 'rightStick=' in source


def test_pico_sender_maps_left_x_and_right_b_into_optional_v1_buttons():
    for path in SENDERS:
        source = path.read_text(encoding='utf-8')
        assert 'CommonUsages.primaryButton' in source
        assert 'CommonUsages.secondaryButton' in source
        assert 'left_x = leftX' in source
        assert 'right_b = rightB' in source
        assert 'public ButtonsWire buttons;' in source
        assert 'VR action buttons: left_x=' in source


def test_pico_sender_shares_discovered_host_with_robot_pov_video():
    for path in SENDERS:
        source = path.read_text(encoding='utf-8')
        assert 'public static event Action<string> EndpointDiscovered;' in source
        assert 'public static string LastDiscoveredHost' in source
        assert 'NotifyEndpointDiscovered(host);' in source
