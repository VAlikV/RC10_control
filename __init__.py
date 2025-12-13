import sys
from pathlib import Path

# Allow imports like `API.rc_api` when the packaged sources reside in ./API.
_pkg_dir = Path(__file__).parent
_inner = _pkg_dir / "API"
if _inner.is_dir():
    __path__.append(str(_inner))
