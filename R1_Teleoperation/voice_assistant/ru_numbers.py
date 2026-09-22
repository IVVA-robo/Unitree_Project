"""Convert numeric tokens to Russian words before text-to-speech."""

from __future__ import annotations

import re
from typing import List


_UNDER_20 = (
    "ноль",
    "один",
    "два",
    "три",
    "четыре",
    "пять",
    "шесть",
    "семь",
    "восемь",
    "девять",
    "десять",
    "одиннадцать",
    "двенадцать",
    "тринадцать",
    "четырнадцать",
    "пятнадцать",
    "шестнадцать",
    "семнадцать",
    "восемнадцать",
    "девятнадцать",
)
_UNDER_20_FEMININE = list(_UNDER_20)
_UNDER_20_FEMININE[1:3] = ["одна", "две"]
_TENS = (
    "", "", "двадцать", "тридцать", "сорок", "пятьдесят",
    "шестьдесят", "семьдесят", "восемьдесят", "девяносто",
)
_HUNDREDS = (
    "", "сто", "двести", "триста", "четыреста", "пятьсот",
    "шестьсот", "семьсот", "восемьсот", "девятьсот",
)
_SCALES = (
    ("", "", "", False),
    ("тысяча", "тысячи", "тысяч", True),
    ("миллион", "миллиона", "миллионов", False),
    ("миллиард", "миллиарда", "миллиардов", False),
    ("триллион", "триллиона", "триллионов", False),
)
_NUMBER_RE = re.compile(r"(?<![\w-])-?\d+(?:[.,]\d+)?(?![\w])")


def _plural_index(value: int) -> int:
    last_two = value % 100
    if 11 <= last_two <= 14:
        return 2
    last = value % 10
    if last == 1:
        return 0
    if 2 <= last <= 4:
        return 1
    return 2


def _under_thousand(value: int, feminine: bool = False) -> List[str]:
    if not 0 <= value < 1000:
        raise ValueError("value must be between 0 and 999")
    words: List[str] = []
    if value >= 100:
        words.append(_HUNDREDS[value // 100])
        value %= 100
    if value >= 20:
        words.append(_TENS[value // 10])
        value %= 10
    if value:
        words.append((_UNDER_20_FEMININE if feminine else _UNDER_20)[value])
    return words


def number_to_words_ru(value: int) -> str:
    """Render a signed integer in Russian words."""

    if not isinstance(value, int) or isinstance(value, bool):
        raise TypeError("value must be an integer")
    if value == 0:
        return "ноль"

    prefix = "минус " if value < 0 else ""
    value = abs(value)
    groups = []
    while value:
        groups.append(value % 1000)
        value //= 1000
    if len(groups) > len(_SCALES):
        raise ValueError("integer is too large to render in Russian words")

    words: List[str] = []
    for scale_index in range(len(groups) - 1, -1, -1):
        group = groups[scale_index]
        if not group:
            continue
        scale = _SCALES[scale_index]
        words.extend(_under_thousand(group, feminine=scale[3]))
        if scale_index:
            words.append(scale[_plural_index(group)])
    return prefix + " ".join(words)


def _decimal_to_words(token: str) -> str:
    sign = token.startswith("-")
    unsigned = token[1:] if sign else token
    unsigned = unsigned.replace(",", ".")
    if "." not in unsigned:
        rendered = number_to_words_ru(int(unsigned))
        return f"минус {rendered}" if sign else rendered

    whole_text, fraction_text = unsigned.split(".", 1)
    fraction_value = int(fraction_text)
    if fraction_value == 0:
        rendered = number_to_words_ru(int(whole_text))
        return f"минус {rendered}" if sign else rendered

    whole_value = int(whole_text)
    whole_words = (
        "одна" if whole_value == 1 else number_to_words_ru(whole_value)
    )
    fraction_words = number_to_words_ru(fraction_value)
    if len(fraction_text) == 1:
        denominator = "десятая" if fraction_value == 1 else "десятых"
    elif len(fraction_text) == 2:
        denominator = "сотая" if fraction_value == 1 else "сотых"
    elif len(fraction_text) == 3:
        denominator = "тысячная" if fraction_value == 1 else "тысячных"
    else:
        # Long fractions are uncommon in voice answers.  Spell their digits
        # individually instead of returning unreadable numerals to TTS.
        fraction_words = " ".join(
            _UNDER_20[int(digit)] for digit in fraction_text
        )
        denominator = "после запятой"
    whole_adjective = "целая" if whole_value == 1 else "целых"
    rendered = (
        f"{whole_words} {whole_adjective} {fraction_words} {denominator}"
    )
    return f"минус {rendered}" if sign else rendered


def verbalize_numbers(text: str) -> str:
    """Replace standalone integer and decimal tokens with Russian words."""

    if not text:
        return text
    return _NUMBER_RE.sub(
        lambda match: _decimal_to_words(match.group(0)), text
    )
