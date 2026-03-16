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

for NOISE in 0.0 0.01 0.05 0.1 0.2; do
    for ((run=1; run<=RUNS; run++)); do
        LABEL="Kp${KP}_Ki${KI}_Kd${KD}_noise${NOISE}_run${run}"
        echo "=== E3: $LABEL ==="
        ./run_experiment.sh "$KP" "$KI" "$KD" "$LABEL" straight "$NOISE" "$DOMAIN" "$SERVER"
        cp "results_${LABEL}.csv" "e3_results_${LABEL}.csv"
        cp "trajectory_${LABEL}.csv" "e3_traj_${LABEL}.csv"
    done
done

echo "=== E3 COMPLETE (runs per noise: $RUNS) ==="