"""Conservative, persistent reservations for the user-approved Jev dollar cap.

Prices are estimates from the direct provider, not a billing-console balance.
The ledger charges the full upper bound for all three possible SDK attempts,
even when a call succeeds immediately. Unknown/interrupted attempts keep their
reservation. Saved-answer replay never calls this adapter and costs nothing new.
"""

from contextlib import contextmanager
from decimal import Decimal
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
from jev_cbf.selection.client import ask_jev

PRICE_SOURCE = "https://docs.typesafe.ai/models"

INPUT_USD_PER_MILLION = Decimal("0.042")

MAX_INPUT_TOKENS = 65_536  # Conservative interpretation of the documented 64k.

SDK_MAX_ATTEMPTS = 3  # Installed 0.7.5 RetryPolicy default is two retries.

RESERVATION = INPUT_USD_PER_MILLION*MAX_INPUT_TOKENS*SDK_MAX_ATTEMPTS/1_000_000

class JevBudget:
    def __init__(self, path, *, cap_usd="7.50", ask=ask_jev):
        self.path, self.cap, self.ask = Path(path), Decimal(cap_usd), ask
        if not self.cap.is_finite() or self.cap <= 0:
            raise ValueError("budget must be finite and positive")

    @contextmanager
    def ledger(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.with_suffix(".lock").open("a+") as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            raw = self.path.read_text() if self.path.exists() else ""
            state = json.loads(raw) if raw else {
                "cap_usd": str(self.cap), "price_source": PRICE_SOURCE,
                "price_checked_date": "2026-10-21", "input_usd_per_million": str(INPUT_USD_PER_MILLION),
                "output_usd_per_million": "0", "attempts": [],
                "note": "Upper-bound reservations, not exact invoices. Includes full context for all default SDK retries. Other processes/account usage outside this project is not included.",
            }
            if Decimal(state["cap_usd"]) != self.cap:
                raise ValueError("existing ledger cap differs; do not silently change approval")
            try:
                yield state
                with tempfile.NamedTemporaryFile(mode="w", dir=self.path.parent, prefix=".budget-", delete=False) as pending:
                    pending.write(json.dumps(state, indent=2, allow_nan=False)+"\n")
                    pending.flush()
                    os.fsync(pending.fileno())
                os.replace(pending.name, self.path)
            finally:
                fcntl.flock(stream, fcntl.LOCK_UN)

    def __call__(self, request):
        if request.get("model") != "jev-1.13.0" or set(request) != {"model", "state", "questions"}:
            raise ValueError("budgeted experiment requires pinned Jev 1.13/default-context request")
        request_hash = hashlib.sha256(json.dumps(request, sort_keys=True).encode()).hexdigest()
        with self.ledger() as state:
            reserved = sum((Decimal(a["reserved_usd"]) for a in state["attempts"]), Decimal(0))
            if reserved+RESERVATION > self.cap:
                raise ValueError("Jev budget exhausted: no request sent")
            index = len(state["attempts"])
            state["attempts"].append({"status": "reserved", "request_sha256": request_hash,
                                       "reserved_usd": str(RESERVATION), "started_unix_s": time.time()})
        try:
            response = self.ask(request)
        except Exception as error:
            with self.ledger() as state:
                state["attempts"][index].update(status="failed_or_unknown", error_type=type(error).__name__)
            raise
        with self.ledger() as state:
            usage = response.get("usage", {})
            entry = state["attempts"][index]
            entry.update(status="returned", usage=usage, actual_model=response.get("model"))
            tokens = usage.get("input_tokens")
            if isinstance(tokens, int) and not isinstance(tokens, bool) and tokens >= 0:
                entry["reported_usage_estimated_usd"] = str(INPUT_USD_PER_MILLION*tokens/1_000_000)
        return response
