"""Development wrapper for the installed ``dm-h65-webui`` command."""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SRC = PROJECT_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from dm_h65.webui import ChassisService, ServiceError, main, make_handler  # noqa: E402,F401


if __name__ == "__main__":
    raise SystemExit(main())
