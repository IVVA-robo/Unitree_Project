import unittest
from unittest.mock import patch

from voice_assistant.response_style import (
    LONG,
    NORMAL,
    SHORT,
    ResponseLengthPolicy,
    quick_template_answer,
)
from voice_assistant.wake_name_gate import WakeNameGate


class ResponseLengthPolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = ResponseLengthPolicy()

    def test_smalltalk_is_short_and_has_template(self):
        decision = self.policy.decide("привет")
        self.assertEqual(decision.style, SHORT)
        self.assertEqual(decision.reason, "smalltalk")
        self.assertEqual(quick_template_answer("привет"), "Привет!")

    def test_everyday_phrases_are_short(self):
        for text in (
            "как дела?", "ты меня слышишь?", "спасибо", "как погода?"
        ):
            self.assertEqual(self.policy.decide(text).style, SHORT)

    def test_explanation_requests_are_long(self):
        for text in (
            "расскажи стих",
            "расскажи как ты работаешь",
            "объясни зрение робота",
            "опиши свои возможности",
            "расскажи подробнее",
            "почему ты так сделал",
        ):
            self.assertEqual(self.policy.decide(text).style, LONG)

    def test_robot_commands_are_short(self):
        decision = self.policy.decide("иди вперед")
        self.assertEqual(decision.style, SHORT)
        self.assertEqual(decision.reason, "robot_command")

    def test_default_is_normal(self):
        decision = self.policy.decide("какой у тебя любимый цвет")
        self.assertEqual(decision.style, NORMAL)

    def test_policy_can_be_disabled(self):
        policy = ResponseLengthPolicy(default_length=LONG, enabled=False)
        decision = policy.decide("расскажи стих")
        self.assertEqual(decision.style, LONG)
        self.assertEqual(decision.reason, "policy_disabled")
        self.assertEqual(policy.decide("иди вперед").style, SHORT)

    def test_env_overrides_default_and_keywords(self):
        env = {
            "VOICE_DEFAULT_RESPONSE_LENGTH": "short",
            "VOICE_SMALLTALK_MAX_WORDS": "4",
            "VOICE_ENABLE_LENGTH_POLICY": "1",
            "VOICE_VERBOSE_KEYWORDS": "развернуто,поясни",
        }
        with patch.dict("os.environ", env, clear=True):
            policy = ResponseLengthPolicy.from_env()
        self.assertEqual(policy.default_length, SHORT)
        self.assertEqual(policy.smalltalk_max_words, 4)
        self.assertEqual(policy.decide("поясни работу камеры").style, LONG)

    def test_wake_gate_runs_before_policy(self):
        gate = WakeNameGate()
        raw = gate.process("расскажи стих")
        self.assertFalse(raw.accepted)
        policy_calls = []
        if raw.accepted:
            policy_calls.append(self.policy.decide(raw.stripped_text))
        self.assertEqual(policy_calls, [])


if __name__ == "__main__":
    unittest.main()
