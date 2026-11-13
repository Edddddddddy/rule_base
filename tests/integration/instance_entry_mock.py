#!/usr/bin/env python3
"""S1 integration-test wrapper: run the repository's real instance entry as a mock instance.

``tools/mocks/fixtures/fake_inference_manager.py --instance-script <path>`` spawns the
given script as ``[sys.executable, <path>]`` and only injects the contracts §1.2
environment variables. This wrapper therefore pins this checkout's ``src`` at the front
of ``sys.path`` (the machine also has an editable ``so101-rule-base`` install pointing
at another copy) and then calls the frozen entry CLI in ``--dry-run`` mode.

The wrapper is a test asset: it is never installed and never used on hardware.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "src"
CONFIG = REPO_ROOT / "configs" / "pick_place" / "mock_ros2_instance.yaml"


def main() -> int:
    sys.path.insert(0, str(SRC_DIR))
    from rule_base.instance.entry import main as entry_main

    return entry_main(["--config", str(CONFIG), "--dry-run"])


if __name__ == "__main__":
    raise SystemExit(main())
