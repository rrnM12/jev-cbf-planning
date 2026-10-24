"""Budget tests use fabricated responses and never contact a provider."""

from concurrent.futures import ThreadPoolExecutor
from decimal import Decimal
import tempfile
from pathlib import Path
import unittest

from jev_cbf.selection.budget import JevBudget, RESERVATION
from jev_cbf.io import read


class BudgetTests(unittest.TestCase):
    def test_reservation_precedes_call_and_replay_is_not_an_implicit_refund(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"ledger.json"
            def ask(request):
                self.assertEqual(read(path)["attempts"][0]["status"], "reserved")
                return {"model": "FABRICATED", "usage": {"input_tokens": 1000, "output_tokens": 20}}
            budget = JevBudget(path, ask=ask)
            budget({"model": "jev-1.13.0", "state": {}, "questions": {}})
            item = read(path)["attempts"][0]
            self.assertEqual(Decimal(item["reserved_usd"]), RESERVATION)
            self.assertEqual(Decimal(item["reported_usage_estimated_usd"]), Decimal("0.000042"))

    def test_exhaustion_and_failed_call_keep_reservations(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"ledger.json"
            def fail(_):
                raise ValueError("fabricated failure")
            budget = JevBudget(path, cap_usd=str(RESERVATION), ask=fail)
            request = {"model": "jev-1.13.0", "state": {}, "questions": {}}
            with self.assertRaisesRegex(ValueError, "fabricated"):
                budget(request)
            with self.assertRaisesRegex(ValueError, "exhausted"):
                budget(request)
            self.assertEqual(len(read(path)["attempts"]), 1)
            self.assertEqual(read(path)["attempts"][0]["status"], "failed_or_unknown")

    def test_concurrent_reservations_and_cap_model_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"ledger.json"
            budget = JevBudget(path, ask=lambda _: {"model": "FABRICATED"})
            request = {"model": "jev-1.13.0", "state": {}, "questions": {}}
            with ThreadPoolExecutor(max_workers=4) as pool:
                list(pool.map(budget, [request]*8))
            self.assertEqual(len(read(path)["attempts"]), 8)
            with self.assertRaises(ValueError):
                JevBudget(path, cap_usd="8.00")(request)
            with self.assertRaises(ValueError):
                budget(dict(request, model="jev-latest"))
