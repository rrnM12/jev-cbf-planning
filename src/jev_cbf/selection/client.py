import os

def ask_jev(request):
    """Thin SDK adapter; uses environment credentials and ordinary SDK defaults."""
    if not os.environ.get("TYPESAFE_API_KEY", "").strip():
        raise ValueError("TYPESAFE_API_KEY is missing; no request sent. Configure it locally, not in chat.")
    try:
        from typesafe_sdk import TypeSafeClient, TypeSafeError
    except ImportError as error:
        raise ValueError("Install the optional SDK with: uv sync --extra jev --cache-dir .uv-cache") from error
    try:
        with TypeSafeClient() as client:
            return client.system_one(**request).model_dump(mode="json")
    except TypeSafeError as error:
        # Don't copy SDK error bodies or authentication details into experiment
        # logs. No guessed/fallback selection is returned on a failed request.
        status = getattr(error, "status_code", None)
        detail = f", HTTP {status}" if status is not None else ""
        raise ValueError(f"TypeSafe request failed ({type(error).__name__}{detail}); no proposal produced") from error
