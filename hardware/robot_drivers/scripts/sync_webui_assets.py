"""Copy the built React client into the Python package before building a wheel."""

from __future__ import annotations

import shutil
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
SOURCE = PROJECT_DIR / "webui" / "frontend" / "dist" / "client"
DESTINATION = PROJECT_DIR / "src" / "dm_h65" / "webui_static"


def main() -> int:
    if not (SOURCE / "index.html").is_file():
        raise SystemExit(f"Frontend build not found: {SOURCE}. Run npm run build first.")
    if DESTINATION.name != "webui_static" or DESTINATION.parent.name != "dm_h65":
        raise SystemExit(f"Refusing unsafe destination: {DESTINATION}")
    if DESTINATION.exists():
        shutil.rmtree(DESTINATION)
    shutil.copytree(SOURCE, DESTINATION)
    print(f"Synced WebUI assets: {SOURCE} -> {DESTINATION}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
