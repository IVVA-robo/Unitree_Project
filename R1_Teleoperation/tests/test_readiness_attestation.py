"""Runtime readiness proof tests; no ROS graph or robot access."""

import json
from pathlib import Path
import stat

import pytest

from exhibition.readiness_attestation import (
    AttestationError,
    consume,
    issue,
)


TOKEN = "test-commissioning-token-1234"
SESSION_ID = "a" * 32


def proof_path(tmp_path: Path) -> Path:
    tmp_path.chmod(0o700)
    return tmp_path / "static-ready.json"


def test_private_hmac_proof_contains_no_token_and_is_consumed_once(tmp_path):
    path = proof_path(tmp_path)
    issue(
        path,
        token=TOKEN,
        session_id=SESSION_ID,
        mode="static",
        domain_id=88,
        now=100.0,
        boot_id="test-boot",
    )

    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert TOKEN not in path.read_text(encoding="utf-8")
    consume(
        path,
        token=TOKEN,
        session_id=SESSION_ID,
        mode="static",
        domain_id=88,
        now=105.0,
        boot_id="test-boot",
    )

    assert not path.exists()
    with pytest.raises(OSError):
        consume(
            path,
            token=TOKEN,
            session_id=SESSION_ID,
            mode="static",
            domain_id=88,
            now=105.0,
            boot_id="test-boot",
        )


@pytest.mark.parametrize(
    "change",
    ("token", "session", "domain", "stale"),
)
def test_mismatched_or_stale_proof_fails_closed_and_cannot_replay(
    tmp_path, change
):
    path = proof_path(tmp_path)
    issue(
        path,
        token=TOKEN,
        session_id=SESSION_ID,
        mode="static",
        domain_id=88,
        now=100.0,
        boot_id="test-boot",
    )
    values = {
        "token": TOKEN,
        "session_id": SESSION_ID,
        "domain_id": 88,
        "now": 105.0,
    }
    if change == "token":
        values["token"] = "different-token-123456789"
    elif change == "session":
        values["session_id"] = "b" * 32
    elif change == "domain":
        values["domain_id"] = 89
    else:
        values["now"] = 111.0

    with pytest.raises(AttestationError):
        consume(
            path,
            mode="static",
            boot_id="test-boot",
            **values,
        )
    assert not path.exists()


def test_tampered_policy_cannot_pass_hmac(tmp_path):
    path = proof_path(tmp_path)
    issue(
        path,
        token=TOKEN,
        session_id=SESSION_ID,
        mode="static",
        domain_id=88,
        now=100.0,
        boot_id="test-boot",
    )
    envelope = json.loads(path.read_text(encoding="utf-8"))
    envelope["claims"]["static_policy"]["enable_arms"] = True
    path.write_text(json.dumps(envelope), encoding="utf-8")
    path.chmod(0o600)

    with pytest.raises(AttestationError, match="signature"):
        consume(
            path,
            token=TOKEN,
            session_id=SESSION_ID,
            mode="static",
            domain_id=88,
            now=105.0,
            boot_id="test-boot",
        )
    assert not path.exists()
