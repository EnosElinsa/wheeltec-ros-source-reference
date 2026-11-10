"""Create and summarize operator trial records; never starts robot processes."""
import argparse
import json
import math
from pathlib import Path

from wheeltec_vision_nav.vins_metrics import distribution

SCENARIOS = ("stationary", "straight", "turning", "loop", "weak_texture", "occlusion",
             "dynamic_obstacle", "narrow_passage", "supervised_reverse")


def trial_template():
    return {"schema_version": 1, "hardware_validation": "not_run",
            "trials": [{"scenario": scenario, "repeat": repeat, "result": "not_run",
                        "run_directory": None, "elapsed_s": None, "minimum_clearance_m": None,
                        "interventions": None, "stopping_distance_m": None, "failure_reason": None}
                       for scenario in SCENARIOS for repeat in (1, 2, 3)]}


def summarize_trials(payload):
    if payload.get("schema_version") != 1 or not isinstance(payload.get("trials"), list):
        raise ValueError("trial record requires schema_version=1 and trials list")
    rows = payload["trials"]
    seen = set()
    for row in rows:
        key = (row.get("scenario"), row.get("repeat"))
        if key[0] not in SCENARIOS or type(key[1]) is not int or key[1] not in (1, 2, 3) or key in seen:
            raise ValueError("trials require unique scenario/repeat in the fixed three-repeat protocol")
        seen.add(key)
        if row.get("result") not in ("pass", "fail", "not_run"):
            raise ValueError("trial result must be pass, fail or not_run")
        for field in ("elapsed_s", "minimum_clearance_m", "interventions", "stopping_distance_m"):
            value = row.get(field)
            if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                      or not math.isfinite(value) or value < 0):
                raise ValueError(f"{field} must be finite and nonnegative, or null when unmeasured")
        if row.get("result") != "not_run":
            if not row.get("run_directory"):
                raise ValueError("completed trial requires its runtime evidence directory")
            if row.get("result") == "fail" and not row.get("failure_reason"):
                raise ValueError("failed trial requires failure_reason")
    completed = [r for r in rows if r["result"] in ("pass", "fail")]
    navigation = [r for r in completed if r["scenario"] != "stationary"]
    failures = [r["failure_reason"] for r in completed if r["result"] == "fail"]
    expected = {(scenario, repeat) for scenario in SCENARIOS for repeat in (1, 2, 3)}
    return {"expected_trials": len(expected), "completed_trials": len(completed),
            "missing_trials": len(expected - {(r["scenario"], r["repeat"]) for r in completed}),
            "protocol_complete": len(completed) == len(expected),
            "navigation_success_fraction": (sum(r["result"] == "pass" for r in navigation) / len(navigation)
                                            if navigation else None),
            "interventions_recorded": sum(r.get("interventions") is not None for r in completed),
            "interventions_total": sum(r["interventions"] for r in completed if r.get("interventions") is not None),
            "elapsed_s": distribution([r["elapsed_s"] for r in completed if r.get("elapsed_s") is not None]),
            "minimum_clearance_m": min((r["minimum_clearance_m"] for r in completed
                                         if r.get("minimum_clearance_m") is not None), default=None),
            "stopping_distance_m": distribution([r["stopping_distance_m"] for r in completed
                                                 if r.get("stopping_distance_m") is not None]),
            "failure_reasons": failures, "hardware_validation": payload.get("hardware_validation", "not_run")}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--init", type=Path, help="Create a new three-repeat operator record")
    group.add_argument("--report", type=Path, help="Summarize a completed/partial operator record")
    args = parser.parse_args(argv)
    if args.init:
        with args.init.open("x", encoding="utf-8") as stream:
            json.dump(trial_template(), stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        print(f"Created trial template: {args.init}; no robot processes started")
    else:
        result = summarize_trials(json.loads(args.report.read_text(encoding="utf-8")))
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
