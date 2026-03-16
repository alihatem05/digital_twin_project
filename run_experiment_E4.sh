#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

DOMAIN=${DOMAIN:-AF_INET}
SERVER=${SERVER:-localhost}
RUNS=${RUNS:-3}

PATH_TYPE=curved
NOISE=0.05
KP=2.0
KD=0.5

for KI in 0.0 0.1; do
    MODE="PD"
    if [[ "$KI" != "0.0" ]]; then
        MODE="PID"
    fi

    for ((run=1; run<=RUNS; run++)); do
        LABEL="${MODE}_Kp${KP}_Ki${KI}_Kd${KD}_curved_noise${NOISE}_run${run}"
        echo "=== E4: $LABEL ==="
        ./run_experiment.sh "$KP" "$KI" "$KD" "$LABEL" "$PATH_TYPE" "$NOISE" "$DOMAIN" "$SERVER"
        cp "results_${LABEL}.csv" "e4_results_${LABEL}.csv"
        cp "trajectory_${LABEL}.csv" "e4_traj_${LABEL}.csv"
    done
done

echo "=== E4 COMPLETE (runs per mode: $RUNS) ==="