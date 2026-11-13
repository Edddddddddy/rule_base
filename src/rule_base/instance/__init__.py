"""026 inference-instance entry for the SO-101 rule_base program.

Modules:

* :mod:`rule_base.instance.protocol` - pure env parsing, False->True reset edge gate
  and exit-code classification (contracts §1.2/§2, no ROS imports);
* :mod:`rule_base.instance.keepalive` - pure hold-command builder and 2-5 Hz heartbeat
  state used to keep the arm lease alive while idle (contracts §2 D4);
* :mod:`rule_base.instance.runner` - one-round ``rule_base.interfaces.node`` child
  process: frozen argv/env, single-round timeout, SIGTERM forwarding, exit codes;
* :mod:`rule_base.instance.entry` - the rclpy entrypoint launched by the inference
  manager (``python3 -m rule_base.instance.entry --config <yaml> [--dry-run]``).

Docs: ``rule_base/docs/instance-mode.md``.
"""

from __future__ import annotations

__all__ = ["entry", "keepalive", "protocol", "runner"]
