#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

DOMAIN=${DOMAIN:-AF_INET}
SERVER=${SERVER:-localhost}
RUNS=${RUNS:-3}

KP=2.0
KI=0.1
KD=0.5

for ((run=1; run<=RUNS; run++)); do
    LABEL="Kp${KP}_Ki${KI}_Kd${KD}_curved_run${run}"
    echo "=== E2: $LABEL ==="
    ./run_experiment.sh "$KP" "$KI" "$KD" "$LABEL" curved 0.0 "$DOMAIN" "$SERVER"
    cp "results_${LABEL}.csv" "e2_results_${LABEL}.csv"
    cp "trajectory_${LABEL}.csv" "e2_traj_${LABEL}.csv"
done

echo "=== E2 COMPLETE (runs: $RUNS) ==="