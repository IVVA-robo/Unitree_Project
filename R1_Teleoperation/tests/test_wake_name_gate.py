import logging
import unittest
from unittest.mock import patch

from voice_assistant.wake_name_gate import WakeNameGate


class WakeNameGateTests(unittest.TestCase):
    def test_wake_name_is_removed(self):
        result = WakeNameGate().process("Добрыня, как дела?")
        self.assertTrue(result.accepted)
        self.assertEqual(result.stripped_text, "как дела?")
        self.assertEqual(result.wake_name, "добрыня")

    def test_case_and_arithmetic_text(self):
        result = WakeNameGate().process("добрыня сколько будет 5+5")
        self.assertTrue(result.accepted)
        self.assertEqual(result.stripped_text, "сколько будет 5+5")

    def test_colon_separator(self):
        result = WakeNameGate().process("Добрыня: включи камеру")
        self.assertTrue(result.accepted)
        self.assertEqual(result.stripped_text, "включи камеру")

    def test_phrase_without_name_is_ignored(self):
        result = WakeNameGate().process("Как дела?")
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "ignored_no_wake_name")

    def test_name_inside_phrase_is_ignored(self):
        result = WakeNameGate().process("Я думаю, Добрыня может помочь")
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "ignored_no_wake_name")

    def test_empty_text_is_ignored(self):
        result = WakeNameGate().process("   ")
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "empty")

    def test_leading_spaces_are_allowed(self):
        result = WakeNameGate().process("   ДОБРЫНЯ!  Привет")
        self.assertTrue(result.accepted)
        self.assertEqual(result.stripped_text, "привет")

    def test_alias_is_accepted(self):
        result = WakeNameGate(aliases="добрян,добрыня робот").process(
            "Добрыня робот, включи свет"
        )
        self.assertTrue(result.accepted)
        self.assertEqual(result.wake_name, "добрыня робот")
        self.assertEqual(result.stripped_text, "включи свет")

    def test_followup_disabled_does_not_accept_next_phrase(self):
        gate = WakeNameGate(followup_window_sec=0)
        self.assertEqual(gate.process("Добрыня").reason, "wake_only")
        result = gate.process("Сколько будет 5+5?")
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "ignored_no_wake_name")

    def test_followup_window_accepts_until_deadline(self):
        gate = WakeNameGate(followup_window_sec=5)
        self.assertTrue(gate.process("Добрыня", now=10).accepted)
        accepted = gate.process("Сколько будет 5+5?", now=14.9)
        self.assertTrue(accepted.accepted)
        self.assertEqual(accepted.reason, "accepted_followup")
        expired = gate.process("Кто ты?", now=15)
        self.assertFalse(expired.accepted)

    def test_env_configuration(self):
        env = {
            "VOICE_WAKE_NAME": "Аляска",
            "VOICE_WAKE_ALIASES": "аля, робот аляска",
            "VOICE_FOLLOWUP_WINDOW_SEC": "7",
        }
        with patch.dict("os.environ", env, clear=True):
            gate = WakeNameGate.from_env()
        self.assertEqual(gate.wake_name, "аляска")
        self.assertEqual(gate.followup_window_sec, 7)
        self.assertTrue(gate.process("Аля, проверь связь").accepted)

    def test_ignored_text_never_reaches_downstream_parser(self):
        parser_calls = []
        gate = WakeNameGate()

        def dispatch(raw_text):
            result = gate.process(raw_text)
            if result.accepted and result.stripped_text:
                parser_calls.append(result.stripped_text)
            return result

        dispatch("включи моторы")
        dispatch("Я сказал Добрыня включись")
        dispatch("Добрыня, включи камеру")
        self.assertEqual(parser_calls, ["включи камеру"])

    def test_debug_log_contains_ignored_event(self):
        gate = WakeNameGate(logger=logging.getLogger("test.wake_name"))
        with self.assertLogs(
            "test.wake_name", level="DEBUG"
        ) as logs:
            gate.process("Сколько будет 5+5?")
        self.assertTrue(
            any("ignored_no_wake_name" in line for line in logs.output)
        )


if __name__ == "__main__":
    unittest.main()
