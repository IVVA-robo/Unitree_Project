"""Wake-name filtering for the offline R1 voice pipeline.

The gate is deliberately independent of Vosk, Ollama, TTS, and robot command
code.  It can therefore be tested with mock ASR text before it is connected to
hardware.  Only text that starts with the configured name (or an alias) is
returned as accepted.
"""

from dataclasses import dataclass
import logging
import os
import re
import time
from typing import Callable, Iterable, Mapping, Optional, Tuple


LOGGER = logging.getLogger(__name__)
_LEADING_NAME_PUNCTUATION = re.compile(r"^[\s,.;:!?…—–\-]+")


def normalize_text(text: Optional[str]) -> str:
    """Normalize ASR text for matching and downstream processing."""

    if not text:
        return ""
    # Vosk can emit either ё or е.  Keeping one spelling makes aliases stable.
    text = text.replace("ё", "е").replace("Ё", "Е")
    return " ".join(text.casefold().strip().split())


@dataclass(frozen=True)
class WakeNameResult:
    """Result returned by :meth:`WakeNameGate.process`."""

    accepted: bool
    stripped_text: str
    wake_name: Optional[str]
    reason: str

    def as_dict(self) -> dict:
        """Return a JSON-friendly representation for mock/debug tooling."""

        return {
            "accepted": self.accepted,
            "stripped_text": self.stripped_text,
            "wake_name": self.wake_name,
            "reason": self.reason,
        }


class WakeNameGate:
    """Require a wake name at the beginning of every recognized phrase.

    ``followup_window_sec`` is zero by default.  When it is positive, a phrase
    containing only the wake name opens a temporary window in which subsequent
    phrases may omit the name.  The clock is injectable so this behavior is
    deterministic in unit tests.
    """

    def __init__(
        self,
        wake_name: str = "Добрыня",
        aliases: Optional[Iterable[str]] = None,
        followup_window_sec: float = 0.0,
        *,
        clock: Optional[Callable[[], float]] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        normalized_wake_name = normalize_text(wake_name)
        if not normalized_wake_name:
            raise ValueError("wake_name must not be empty")

        alias_values = self._coerce_aliases(aliases)
        all_names = {normalized_wake_name}
        all_names.update(alias_values)
        # Longest first prevents a short name from stealing a configured
        # multi-word alias such as "добрыня робот".
        self._names: Tuple[str, ...] = tuple(
            sorted(all_names, key=lambda value: (-len(value), value))
        )
        self.wake_name = normalized_wake_name
        self.aliases = tuple(
            name for name in self._names if name != normalized_wake_name
        )
        self.followup_window_sec = max(0.0, float(followup_window_sec))
        self._clock = clock or time.monotonic
        self._logger = logger or LOGGER
        self._followup_until: Optional[float] = None

    @staticmethod
    def _coerce_aliases(aliases: Optional[Iterable[str]]) -> Tuple[str, ...]:
        if aliases is None:
            return ()
        if isinstance(aliases, str):
            aliases = aliases.split(",")
        normalized = {normalize_text(alias) for alias in aliases}
        return tuple(alias for alias in normalized if alias)

    @classmethod
    def from_env(
        cls,
        environ: Optional[Mapping[str, str]] = None,
        **kwargs,
    ) -> "WakeNameGate":
        """Build a gate from ``VOICE_WAKE_*`` environment variables."""

        env = os.environ if environ is None else environ
        wake_name = env.get("VOICE_WAKE_NAME", "Добрыня")
        aliases = env.get("VOICE_WAKE_ALIASES", "")
        followup_raw = env.get("VOICE_FOLLOWUP_WINDOW_SEC", "0").strip()
        try:
            followup_window_sec = float(followup_raw or "0")
        except ValueError:
            LOGGER.warning(
                "invalid VOICE_FOLLOWUP_WINDOW_SEC=%r; using 0", followup_raw
            )
            followup_window_sec = 0.0
        return cls(
            wake_name=wake_name,
            aliases=aliases,
            followup_window_sec=followup_window_sec,
            **kwargs,
        )

    def reset_followup(self) -> None:
        """Close an active follow-up window."""

        self._followup_until = None

    def _log_ignored(self, raw_text: Optional[str], normalized: str) -> None:
        self._logger.debug(
            "ignored_no_wake_name raw=%r normalized=%r", raw_text, normalized
        )

    @staticmethod
    def _starts_with_name(text: str, name: str) -> bool:
        if not text.startswith(name):
            return False
        rest = text[len(name):]
        if not rest:
            return True
        # Do not accept a name embedded in another word.
        return not (rest[0].isalnum() or rest[0] == "_")

    @staticmethod
    def _strip_name_punctuation(rest: str) -> str:
        # Punctuation immediately after the name is a separator, not part of
        # the command.  Preserve punctuation in the actual question.
        rest = _LEADING_NAME_PUNCTUATION.sub("", rest.strip())
        return " ".join(rest.split())

    def process(
        self, raw_text: Optional[str], now: Optional[float] = None
    ) -> WakeNameResult:
        """Filter one ASR phrase and return the downstream text if accepted."""

        normalized = normalize_text(raw_text)
        current_time = self._clock() if now is None else now
        if not normalized:
            self._log_ignored(raw_text, normalized)
            return WakeNameResult(False, "", None, "empty")

        for name in self._names:
            if not self._starts_with_name(normalized, name):
                continue
            stripped = self._strip_name_punctuation(normalized[len(name):])
            if not stripped and self.followup_window_sec > 0:
                self._followup_until = current_time + self.followup_window_sec
            reason = "wake_only" if not stripped else "accepted_wake_name"
            self._logger.debug(
                "%s wake_name=%r stripped=%r",
                reason,
                name,
                stripped,
            )
            return WakeNameResult(True, stripped, name, reason)

        if (
            self._followup_until is not None
            and current_time < self._followup_until
        ):
            self._logger.debug("accepted_followup stripped=%r", normalized)
            return WakeNameResult(True, normalized, None, "accepted_followup")

        self._followup_until = None
        self._log_ignored(raw_text, normalized)
        return WakeNameResult(False, "", None, "ignored_no_wake_name")
