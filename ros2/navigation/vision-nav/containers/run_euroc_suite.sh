#!/usr/bin/env bash
# Run inside the installed container. Retain failed runs and aggregate every row.
set -uo pipefail
root="${WHEELTEC_WORK_ROOT:-/work}"
group="${1:-$root/results/euroc-$(date -u +%Y%m%dT%H%M%SZ)}"
case "${2:-all}" in
  all) sequences=(MH_01_easy MH_03_medium MH_05_difficult);;
  MH_01_easy|MH_03_medium|MH_05_difficult) sequences=("$2");;
  *) echo 'Second argument must be all or an implemented EuRoC sequence' >&2; exit 2;;
esac
mkdir "$group" || exit 2
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}" ROS_LOCALHOST_ONLY=1
for sequence in "${sequences[@]}"; do
  for mode in stereo vio; do
    for repeat in 1 2 3; do
      name="$sequence-$mode-$repeat"
      timeout --signal=INT --kill-after=30s 600s ros2 launch wheeltec_vision_benchmark euroc.launch.py \
        "dataset:=$root/data/$sequence" "output:=$group/$name" "mode:=$mode" \
        > "$group/$name.log" 2>&1
      code=$?
      printf '%s\t%d\n' "$name" "$code" >> "$group/exit-codes.tsv"
    done
  done
done
python3 - "$group" <<'PY'
import json, pathlib, sys
group = pathlib.Path(sys.argv[1])
rows = []
for line in (group/'exit-codes.tsv').read_text().splitlines():
    name, code = line.split('\t')
    metrics_path = group/name/'metrics.json'
    row = dict(run=name, exit_code=int(code), metrics=None)
    if metrics_path.is_file():
        row['metrics'] = json.loads(metrics_path.read_text())
        row['outcome'] = ('runtime_failed' if int(code) else
            {'ok': 'scored_complete', 'partial_output': 'scored_partial'}.get(
                row['metrics'].get('status'), 'unscorable'))
    else:
        row['outcome'] = 'runtime_failed' if int(code) else 'missing_metrics'
        row['failure'] = 'No metrics produced; inspect preserved launch log'
    rows.append(row)
(group/'suite.json').write_text(json.dumps(dict(schema=1,
    meaning='Execution/scoring evidence only; partial coverage and accuracy require separate review. No algorithm acceptance implied.',
    runs=rows), indent=2)+'\n')
print('Suite saved:', group/'suite.json')
sys.exit(0 if all(r['exit_code'] == 0 and r['metrics'] and r['metrics']['status'] in ('ok','partial_output') for r in rows) else 1)
PY
