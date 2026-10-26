"""
intervention_report.py -- 3.99: interventions per hour, from the brain's
intervention log (`brain/intervention_log.py`).

    python -m control.intervention_report [dir]

The directory defaults to the brain's own `intervention_dir`, resolved as the
brain server resolves it (`load_brain_config()`: `$INTERVENTION_DIR`, then
`config/robot.yaml`, then the default).
"""

import argparse
import sys
from typing import Optional

from brain.intervention_log import report
from control.brain_config import load_brain_config


def main(argv: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(description="Interventions per hour of autonomy (3.99).")
    parser.add_argument("directory", nargs="?",
                        help="default: the brain config's intervention_dir")
    parser.add_argument("--config", help="a brain config other than config/robot.yaml")
    args = parser.parse_args(argv)
    print(report(args.directory or load_brain_config(args.config)["intervention_dir"]))


if __name__ == "__main__":
    main(sys.argv[1:])
