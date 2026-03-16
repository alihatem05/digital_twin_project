#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

RUNS_PER_GAIN=${RUNS_PER_GAIN:-3}
DOMAIN=${DOMAIN:-AF_INET}
SERVER=${SERVER:-localhost}

run_gain() {
    local kp=$1
    local ki=$2
    local kd=$3
    local base=$4

    for ((run=1; run<=RUNS_PER_GAIN; run++)); do
        local label="${base}_run${run}"
        echo "=== E1: $label ==="
        ./run_experiment.sh "$kp" "$ki" "$kd" "$label" straight 0.0 "$DOMAIN" "$SERVER"
        cp "results_${label}.csv" "e1_results_${label}.csv"
        cp "trajectory_${label}.csv" "e1_traj_${label}.csv"
    done
}

run_gain 0.5 0.05 0.1 Kp0.5_Ki0.05_Kd0.1
run_gain 2.0 0.1 0.5 Kp2.0_Ki0.1_Kd0.5
run_gain 3.0 0.2 0.8 Kp3.0_Ki0.2_Kd0.8
run_gain 5.0 0.5 1.0 Kp5.0_Ki0.5_Kd1.0
run_gain 10.0 1.0 2.0 Kp10.0_Ki1.0_Kd2.0

echo "=== E1 COMPLETE (runs per gain: $RUNS_PER_GAIN) ==="
