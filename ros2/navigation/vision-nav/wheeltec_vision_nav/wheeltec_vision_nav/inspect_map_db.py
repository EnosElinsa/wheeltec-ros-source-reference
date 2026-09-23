#!/usr/bin/env python3
"""Report loop-closure consistency and map correction of an RTAB-Map database."""
from __future__ import annotations

import argparse
import os
import sys

import yaml

from wheeltec_vision_nav.map_db_logic import load_graph, summarize, verdict


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "database",
        nargs="?",
        default=os.path.expanduser("~/.ros/vision_rtabmap_mapping.db"),
        help="RTAB-Map database (opened read-only)",
    )
    args = parser.parse_args(argv)
    summary = summarize(load_graph(args.database))
    problems = verdict(summary)
    summary["result"] = "PASS" if not problems else "WARN"
    summary["problems"] = problems
    yaml.safe_dump(summary, sys.stdout, allow_unicode=True, sort_keys=False)
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
