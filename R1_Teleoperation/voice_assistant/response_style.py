"""Response-length policy and fast Russian small-talk templates."""

from dataclasses import dataclass
import logging
import os
import re
from typing import Iterable, Mapping, Optional, Tuple

try:
    from .wake_name_gate import normalize_text
except ImportError:  # standalone copy used by /home/unitree/VoiceAssistant
    from wake_name_gate import normalize_text


LOGGER = logging.getLogger(__name__)
SHORT = "short"
NORMAL = "normal"
LONG = "long"
_VALID_STYLES = {SHORT, NORMAL, LONG}
_DEFAULT_VERBOSE_KEYWORDS = (
    "расскажи",
    "объясни",
    "опиши",
    "подробнее",
    "почему",
    "сравни",
    "придумай",
    "стих",
    "историю",
    "рассказ",
    "научи",
    "инструкция",
    "как ты работаешь",
    "что ты умеешь",
    "твои возможности",
    "как устроено",
    "развернуто",
)
_ROBOT_COMMAND_RE = re.compile(
    r"^(?:"
    r"иди\b|двигайся\b|остановись\b|стой\b|повернись\b|"
    r"включи\b|выключи\b|подними\b|опусти\b|встань\b|"
    r"сядь\b|наклонись\b|сделай\s+шаг\b|покажи\s+камеру\b|"
    r"сними\b|приготовься\b|приготовь\b"
    r")"
)


@dataclass(frozen=True)
class ResponseLengthDecision:
    """Style selected for one already wake-name-filtered request."""

    style: str
    reason: str

    def as_dict(self) -> dict:
        return {"style": self.style, "reason": self.reason}


class ResponseLengthPolicy:
    """Classify a cleaned request as ``short``, ``normal``, or ``long``."""

    def __init__(
        self,
        default_length: str = NORMAL,
        smalltalk_max_words: int = 12,
        enabled: bool = True,
        verbose_keywords: Optional[Iterable[str]] = None,
        *,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        default_length = normalize_text(default_length)
        if default_length not in _VALID_STYLES:
            LOGGER.warning(
                "invalid default response length %r; using normal",
                default_length,
            )
            default_length = NORMAL
        self.default_length = default_length
        self.smalltalk_max_words = max(1, int(smalltalk_max_words))
        self.enabled = bool(enabled)
        values = (
            _DEFAULT_VERBOSE_KEYWORDS
            if verbose_keywords is None
            else verbose_keywords
        )
        if isinstance(values, str):
            values = values.split(",")
        normalized_values = {
            normalize_text(value) for value in values if normalize_text(value)
        }
        self.verbose_keywords: Tuple[str, ...] = tuple(
            sorted(
                normalized_values,
                key=lambda value: (-len(value), value),
            )
        )
        self._logger = logger or LOGGER

    @classmethod
    def from_env(
        cls,
        environ: Optional[Mapping[str, str]] = None,
        **kwargs,
    ) -> "ResponseLengthPolicy":
        """Build policy from the ``VOICE_*`` environment settings."""

        env = os.environ if environ is None else environ
        enabled_raw = env.get(
            "VOICE_ENABLE_LENGTH_POLICY", "1"
        ).strip().lower()
        enabled = enabled_raw not in {"0", "false", "no", "off"}
        max_words_raw = env.get("VOICE_SMALLTALK_MAX_WORDS", "12").strip()
        try:
            max_words = int(max_words_raw or "12")
        except ValueError:
            LOGGER.warning(
                "invalid VOICE_SMALLTALK_MAX_WORDS=%r; using 12", max_words_raw
            )
            max_words = 12
        keywords = env.get("VOICE_VERBOSE_KEYWORDS", "")
        return cls(
            default_length=env.get("VOICE_DEFAULT_RESPONSE_LENGTH", NORMAL),
            smalltalk_max_words=max_words,
            enabled=enabled,
            verbose_keywords=keywords or None,
            **kwargs,
        )

    def decide(self, cleaned_text: str) -> ResponseLengthDecision:
        """Choose a response style for text after the wake-name gate."""

        normalized = normalize_text(cleaned_text)
        # Robot commands stay short even when the optional policy is disabled.
        if normalized and _ROBOT_COMMAND_RE.search(normalized):
            decision = ResponseLengthDecision(SHORT, "robot_command")
            self._logger.debug(
                "response_style text=%r decision=%s", normalized, decision
            )
            return decision
        if not self.enabled:
            decision = ResponseLengthDecision(
                self.default_length, "policy_disabled"
            )
            self._logger.debug(
                "response_style text=%r decision=%s", normalized, decision
            )
            return decision
        if not normalized:
            decision = ResponseLengthDecision(SHORT, "empty_request")
            self._logger.debug(
                "response_style text=%r decision=%s", normalized, decision
            )
            return decision

        for keyword in self.verbose_keywords:
            if keyword in normalized:
                decision = ResponseLengthDecision(
                    LONG, f"verbose_keyword:{keyword}"
                )
                self._logger.debug(
                    "response_style text=%r decision=%s", normalized, decision
                )
                return decision

        if self._is_smalltalk(normalized):
            decision = ResponseLengthDecision(SHORT, "smalltalk")
            self._logger.debug(
                "response_style text=%r decision=%s", normalized, decision
            )
            return decision

        decision = ResponseLengthDecision(self.default_length, "default")
        self._logger.debug(
            "response_style text=%r decision=%s", normalized, decision
        )
        return decision

    def _is_smalltalk(self, normalized: str) -> bool:
        exact_phrases = (
            "привет",
            "здравствуй",
            "доброе утро",
            "добрый день",
            "добрый вечер",
            "как дела",
            "как настроение",
            "ты здесь",
            "ты меня слышишь",
            "слышишь меня",
            "спасибо",
            "благодарю",
            "пока",
            "до свидания",
            "как погода",
            "как тебе погода",
        )
        if normalized.rstrip("?!.,") in exact_phrases:
            return True
        words = normalized.rstrip("?!.,").split()
        # A short question without an explanation request is small talk.
        return (
            normalized.endswith("?")
            and len(words) <= self.smalltalk_max_words
        )


def quick_template_answer(cleaned_text: str) -> Optional[str]:
    """Return a fast answer for obvious small-talk requests, if available."""

    normalized = normalize_text(cleaned_text).rstrip("?!.,")
    if normalized in {
        "привет",
        "здравствуй",
        "доброе утро",
        "добрый день",
        "добрый вечер",
    }:
        return "Привет!"
    if normalized in {"как дела", "как настроение"}:
        return "Всё хорошо, я готов."
    if normalized in {"ты меня слышишь", "слышишь меня"}:
        return "Да, слышу."
    if normalized == "ты здесь":
        return "Да, я здесь."
    if normalized in {"спасибо", "благодарю"}:
        return "Пожалуйста."
    if normalized in {"пока", "до свидания"}:
        return "Пока!"
    if normalized in {"как погода", "как тебе погода"}:
        return "Я не вижу погоду без датчиков."
    return None
