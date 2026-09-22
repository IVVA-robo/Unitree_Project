import unittest

from voice_assistant.ru_numbers import number_to_words_ru, verbalize_numbers


class RussianNumberTests(unittest.TestCase):
    def test_integer_words(self):
        self.assertEqual(number_to_words_ru(0), "ноль")
        self.assertEqual(number_to_words_ru(1200), "одна тысяча двести")
        self.assertEqual(number_to_words_ru(50), "пятьдесят")
        self.assertEqual(number_to_words_ru(-24), "минус двадцать четыре")

    def test_scale_gender_and_plural(self):
        self.assertEqual(number_to_words_ru(2000), "две тысячи")
        self.assertEqual(number_to_words_ru(1000000), "один миллион")
        self.assertEqual(number_to_words_ru(5000000), "пять миллионов")

    def test_decimal_words(self):
        self.assertEqual(
            verbalize_numbers("Температура 1,5"),
            "Температура одна целая пять десятых",
        )

    def test_answer_digits_become_speakable_text(self):
        self.assertEqual(
            verbalize_numbers("Ответ: 1200."),
            "Ответ: одна тысяча двести.",
        )

    def test_multiple_numbers_are_converted(self):
        self.assertEqual(
            verbalize_numbers("5 + 5 = 10"),
            "пять + пять = десять",
        )


if __name__ == "__main__":
    unittest.main()
