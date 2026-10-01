"""One-shot proof that the current exhibition graph was already validated.

The file is runtime-only and contains no commissioning token.  Its HMAC binds
the readiness result to that token, the manager session, this boot and the ROS
domain.  A successful verification always consumes the file, preventing a
later prepare attempt from replaying an old graph observation.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import os
from pathlib import Path
import re
import secrets
import stat
import tempfile
import time
from typing import Optional


SCHEMA = 1
PURPOSE = "r1-exhibition-static-readiness"
MAX_AGE_SEC = 10.0
SESSION_PATTERN = re.compile(r"^[0-9a-f]{32}$")
STATIC_POLICY = {
    "enable_arms": False,
    "enable_head": False,
    "enable_locomotion": False,
    "enable_prepare": True,
    "exhibition_session_mode": False,
    "head_recenter_enabled": False,
    "prepare_enter_locomotion": False,
    "profile": "slow-safe",
    "send_commands": True,
    "static_prepare_mode": True,
    "transport": "sdk",
}


class AttestationError(ValueError):
    """Raised when a readiness proof is absent, stale or does not match."""


def _boot_id() -> str:
    return Path("/proc/sys/kernel/random/boot_id").read_text(
        encoding="utf-8"
    ).strip()


def _validate_inputs(
    *, token: str, session_id: str, mode: str, domain_id: int
) -> None:
    if len(token) < 16:
        raise AttestationError("commissioning token is unavailable")
    if mode != "static":
        raise AttestationError("only static readiness may be attested")
    if not SESSION_PATTERN.fullmatch(session_id):
        raise AttestationError("invalid exhibition session id")
    if not 0 <= domain_id <= 232:
        raise AttestationError("ROS domain must be 0..232")


def _canonical(payload: dict) -> bytes:
    return json.dumps(
        payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def _signature(payload: dict, token: str) -> str:
    return hmac.new(
        token.encode("utf-8"), _canonical(payload), hashlib.sha256
    ).hexdigest()


def _private_parent(path: Path) -> None:
    parent = path.parent
    metadata = parent.stat(follow_symlinks=False)
    if not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != os.getuid():
        raise AttestationError("runtime directory owner/type mismatch")
    if stat.S_IMODE(metadata.st_mode) & 0o077:
        raise AttestationError("runtime directory is not private")


def issue(
    path: Path,
    *,
    token: str,
    session_id: str,
    mode: str,
    domain_id: int,
    now: Optional[float] = None,
    boot_id: Optional[str] = None,
) -> None:
    """Atomically write one private, short-lived static readiness proof."""
    _validate_inputs(
        token=token, session_id=session_id, mode=mode, domain_id=domain_id
    )
    path = Path(path)
    if not path.is_absolute():
        raise AttestationError("attestation path must be absolute")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    _private_parent(path)
    issued_at = time.time() if now is None else float(now)
    claims = {
        "boot_id": _boot_id() if boot_id is None else boot_id,
        "expires_at": issued_at + MAX_AGE_SEC,
        "issued_at": issued_at,
        "mode": mode,
        "nonce": secrets.token_hex(16),
        "purpose": PURPOSE,
        "ros_domain_id": int(domain_id),
        "schema": SCHEMA,
        "session_id": session_id,
        "static_policy": STATIC_POLICY,
    }
    envelope = {
        "claims": claims,
        "hmac_sha256": _signature(claims, token),
    }
    fd, temporary_name = tempfile.mkstemp(
        prefix=".readiness-", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(envelope, stream, ensure_ascii=False, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o600)
    finally:
        temporary.unlink(missing_ok=True)


def _read_once(path: Path) -> dict:
    """Read a private regular file and unlink the exact inode before use."""
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(path, flags)
    try:
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_uid != os.getuid()
        ):
            raise AttestationError("attestation owner/type mismatch")
        if (
            stat.S_IMODE(metadata.st_mode) != 0o600
            or metadata.st_nlink != 1
        ):
            raise AttestationError(
                "attestation permissions/link count mismatch"
            )
        if metadata.st_size <= 0 or metadata.st_size > 16384:
            raise AttestationError("attestation size is invalid")
        raw = bytearray()
        while True:
            chunk = os.read(descriptor, 4096)
            if not chunk:
                break
            raw.extend(chunk)
            if len(raw) > 16384:
                raise AttestationError("attestation is too large")
        path_metadata = path.stat(follow_symlinks=False)
        if (
            path_metadata.st_dev != metadata.st_dev
            or path_metadata.st_ino != metadata.st_ino
        ):
            raise AttestationError("attestation changed while being read")
        # Consume before verification.  A failed/tampered proof cannot be
        # retried or replayed with a different environment.
        path.unlink()
    finally:
        os.close(descriptor)
    try:
        return json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AttestationError("attestation JSON is invalid") from error


def consume(
    path: Path,
    *,
    token: str,
    session_id: str,
    mode: str,
    domain_id: int,
    now: Optional[float] = None,
    boot_id: Optional[str] = None,
) -> None:
    """Verify and permanently consume a matching readiness proof."""
    _validate_inputs(
        token=token, session_id=session_id, mode=mode, domain_id=domain_id
    )
    path = Path(path)
    if not path.is_absolute():
        raise AttestationError("attestation path must be absolute")
    _private_parent(path)
    envelope = _read_once(path)
    if set(envelope) != {"claims", "hmac_sha256"}:
        raise AttestationError("attestation envelope is invalid")
    claims = envelope.get("claims")
    signature = envelope.get("hmac_sha256")
    if not isinstance(claims, dict) or not isinstance(signature, str):
        raise AttestationError("attestation fields are invalid")
    if not hmac.compare_digest(signature, _signature(claims, token)):
        raise AttestationError("attestation signature does not match")
    expected = {
        "boot_id": _boot_id() if boot_id is None else boot_id,
        "mode": mode,
        "purpose": PURPOSE,
        "ros_domain_id": int(domain_id),
        "schema": SCHEMA,
        "session_id": session_id,
        "static_policy": STATIC_POLICY,
    }
    for name, value in expected.items():
        if claims.get(name) != value:
            raise AttestationError(f"attestation claim changed: {name}")
    checked_at = time.time() if now is None else float(now)
    try:
        issued_at = float(claims["issued_at"])
        expires_at = float(claims["expires_at"])
    except (KeyError, TypeError, ValueError) as error:
        raise AttestationError("attestation timestamps are invalid") from error
    if not issued_at <= checked_at <= expires_at:
        raise AttestationError("attestation is stale or from the future")
    if abs((expires_at - issued_at) - MAX_AGE_SEC) > 1e-6:
        raise AttestationError("attestation lifetime was changed")
    if not isinstance(claims.get("nonce"), str) or len(claims["nonce"]) != 32:
        raise AttestationError("attestation nonce is invalid")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("consume",))
    parser.add_argument("--path", type=Path, required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--mode", choices=("static",), required=True)
    parser.add_argument("--domain-id", type=int, required=True)
    arguments = parser.parse_args(argv)
    token = os.environ.get("ROBOT_COMMISSIONING_TOKEN", "")
    try:
        consume(
            arguments.path,
            token=token,
            session_id=arguments.session_id,
            mode=arguments.mode,
            domain_id=arguments.domain_id,
        )
    except (AttestationError, OSError) as error:
        print(
            f"[BLOCKED] static readiness attestation failed: {error}",
            flush=True,
        )
        return 2
    print(
        "[OK] one-shot static graph readiness confirmed; duplicate DDS "
        "discovery is skipped.",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
