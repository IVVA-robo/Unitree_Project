"""Execute only the script's authorization decision, never its robot path."""

import os
from pathlib import Path
import subprocess

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/r1-robot-prepare'


def fast_gate(**overrides):
    text = SCRIPT.read_text()
    gate = text[text.index('FAST_PREPARE=false'):text.index('if ! head_enabled=')]
    environment = dict(os.environ, R1_EXHIBITION_FAST_PREPARE='1',
                       ROBOT_CONFIRM_STATIC_PREPARE='1', TOKEN='test-only-token',
                       WRITER_TOKEN='test-only-token',
                       STATIC_READINESS_ATTESTED='true',
                       static_prepare_mode='Boolean value is: true',
                       exhibition_session_mode='Boolean value is: false', BAD_FEATURE='')
    environment.update(overrides)
    prelude = '''
die() { echo "blocked: $*"; exit 2; }
require_exact() { [[ ${!1-} == "$2" ]] || die "missing $1"; }
require_param() { [[ $1 != "$BAD_FEATURE" ]] || die "enabled $1"; }
dump_scalar() { [[ $1 == commissioning_token ]] && echo "$WRITER_TOKEN"; }
'''
    return subprocess.run(['bash', '-eu', '-c', prelude + gate + '\n[[ $FAST_PREPARE == true ]]'],
                          env=environment, capture_output=True, text=True, timeout=2)


def test_matching_featureless_static_writer_can_reuse_completed_preflight():
    assert fast_gate().returncode == 0


@pytest.mark.parametrize('overrides', [
    {'ROBOT_CONFIRM_STATIC_PREPARE': '0'},
    {'exhibition_session_mode': 'Boolean value is: true'},
    *({'BAD_FEATURE': feature} for feature in (
        'enable_head', 'enable_arms', 'enable_locomotion', 'prepare_enter_locomotion')),
])
def test_fast_static_rejects_wrong_authority_or_enabled_motion(overrides):
    assert fast_gate(**overrides).returncode == 2


def test_fast_control_still_requires_exhibition_ack():
    assert fast_gate(static_prepare_mode='Boolean value is: false',
                     exhibition_session_mode='Boolean value is: true',
                     STATIC_READINESS_ATTESTED='false',
                     ROBOT_EXHIBITION_SESSION='0').returncode == 2
    assert fast_gate(static_prepare_mode='Boolean value is: false',
                     exhibition_session_mode='Boolean value is: true',
                     STATIC_READINESS_ATTESTED='false',
                     ROBOT_EXHIBITION_SESSION='1').returncode == 0


def test_static_attestation_only_removes_duplicate_observation_not_safety_gates():
    text = SCRIPT.read_text()
    attestation = text.index('readiness_attestation.py" consume')
    service_reuse = text.index("if [[ ${STATIC_READINESS_ATTESTED} == true ]]", attestation)
    traffic_gate = text.index('r1-arm-sdk-traffic-check', service_reuse)
    kill_release = text.index('/r1/safety/set_kill', traffic_gate)
    prepare = text.index('/r1/live_writer/prepare', kill_release)

    assert attestation < service_reuse < traffic_gate < kill_release < prepare
    assert '--expected-writers 1' in text
    assert 'arm_sdk_gate_duration=2.25' in text
    assert 'trap emergency_relock ERR' in text
    assert 'wait_for_prepare_confirmation' in text
