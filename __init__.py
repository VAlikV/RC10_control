import sys
from pathlib import Path

# Allow imports like `rc10_api.rc_api` when the packaged sources reside in ./rc10_api.
_pkg_dir = Path(__file__).parent
_inner = _pkg_dir / "rc10_api"
if _inner.is_dir():
    __path__.append(str(_inner))
