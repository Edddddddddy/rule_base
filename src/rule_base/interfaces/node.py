"""CLI entrypoint for the SO-101 Pick station."""

from __future__ import annotations

import argparse
import asyncio
import logging
from pathlib import Path

from rule_base.application.runner import serve
from rule_base.config.load import resolve_config_path
from rule_base.observability.events import event


def _positive_rounds(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError("--rounds must be a positive integer") from None
    if parsed < 1:
        raise argparse.ArgumentTypeError("--rounds must be a positive integer")
    return parsed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument(
        "--dry-run",
        action="store_true",
        help="Run offline rounds with the selected transport (this is the default mode)",
    )
    modes.add_argument(
        "--run",
        action="store_true",
        help=(
            "Live arm execution; defaults to the ROS 2 transport and needs the real arm "
            "server"
        ),
    )
    parser.add_argument(
        "--transport",
        choices=("fake", "ros2"),
        default=None,
        help=(
            "Arm transport: fake (in-process fakes; default for --dry-run) or ros2 "
            "(arm_msgs + ROS 2 topics; default for --run)"
        ),
    )
    parser.add_argument(
        "--rounds",
        type=_positive_rounds,
        default=1,
        help="Number of consecutive rounds; each round emits one round_result",
    )
    parser.add_argument(
        "--round-id",
        default=None,
        metavar="ID",
        help=(
            "Optional round id supplied by the capture orchestrator (L6 binds the recorder "
            "metadata to the same id); default is an internal uuid4 hex. With --rounds > 1 "
            "the id applies to the first round only and later rounds generate their own."
        ),
    )
    parser.add_argument(
        "--target",
        help="Retired Press fixture selector; rejected because the port only keeps Pick",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Run a single round and exit (kept for CLI compatibility; --rounds controls rounds)",
    )
    parser.epilog = (
        "Pick dry-run defaults to the in-process fake transport; --transport ros2 runs the "
        "same offline round against a live ROS 2 arm server. Live execution (--run) defaults "
        "to --transport ros2 when --transport is omitted; an explicit --transport fake is "
        "rejected. TF frames come from the config, never from a hardcoded station prefix."
    )
    args = parser.parse_args()
    if args.run and args.transport == "fake":
        parser.error("--run requires the ROS 2 transport (use --transport ros2 or omit it)")
    if args.run:
        # Live execution has no in-process fake implementation: --run without an
        # explicit transport must mean the ROS 2 transport, not a fatal fallback.
        args.transport = "ros2"
    target_id = None
    if args.target is not None:
        if not args.target.isdecimal():
            parser.error("--target must be a decimal ID")
        target_id = int(args.target)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        config_path, station = resolve_config_path(args.config)
        return asyncio.run(
            serve(
                config_path,
                live=args.run,
                once=args.once,
                target_id=target_id,
                station=station,
                transport=args.transport or "fake",
                rounds=args.rounds,
                round_id=args.round_id,
            )
        )
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        event("fatal", error=str(exc))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
