"""JSON lines for manager stdout collection; no authority credentials."""

from __future__ import annotations

import json
import logging
from typing import Any

log = logging.getLogger("rule_base")


def event(name: str, **fields: Any) -> None:
    log.info(json.dumps({"event": name, **fields}, ensure_ascii=False, default=str))
