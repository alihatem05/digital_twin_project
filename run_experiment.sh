#!/bin/bash
source /data/tools/pave/innexis_home/vsi_2025.2/env_vsi.bash 2>/dev/null || true
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

KP=${1:-2.0}
KI=${2:-0.1}
KD=${3:-0.5}
LABEL=${4:-run}
PATH_TYPE=${5:-straight}
NOISE=${6:-0.0}
DOMAIN=${7:-AF_INET}
SERVER=${8:-localhost}

echo "============================================"
echo "Running: $LABEL  path=$PATH_TYPE"
echo "Kp=$KP Ki=$KI Kd=$KD Noise=$NOISE"
echo "Domain=$DOMAIN Server=$SERVER"
echo "============================================"

cat > Makefile.client1 << EOF
all: build sim
compile:
	@echo "No compile needed"
build:
	@echo "No build needed"
sim:
	cd $ROOT_DIR && PYTHONPATH=. python3 src/controller/controller.py --domain=$DOMAIN --server-url=$SERVER --Kp=$KP --Ki=$KI --Kd=$KD --path=$PATH_TYPE; bash
clean:
	@echo "Nothing to clean"
EOF

cat > Makefile.client0 << EOF
all: build sim
compile:
	@echo "No compile needed"
build:
	@echo "No build needed"
sim:
	cd $ROOT_DIR && PYTHONPATH=. python3 src/plant/plant.py --domain=$DOMAIN --server-url=$SERVER --noise=$NOISE --path=$PATH_TYPE; bash
clean:
	@echo "Nothing to clean"
EOF

cat > Makefile.client2 << EOF
DOMAIN ?= $DOMAIN
SERVER ?= $SERVER
PATH_TYPE ?= $PATH_TYPE
LABEL ?= $LABEL

all: build sim
compile:
	@echo "No compile needed"
build:
	@echo "No build needed"
sim:
	cd $ROOT_DIR && PYTHONPATH=. python3 src/visualizer/visualizer.py --domain=\$(DOMAIN) --server-url=\$(SERVER) --label=\$(LABEL) --path=\$(PATH_TYPE); bash
clean:
	@echo "Nothing to clean"
EOF

vsiSim LineFollowingRobot.dt
sleep 2
cp visualizer_log.csv "results_${LABEL}.csv"
cp trajectory_data.csv "trajectory_${LABEL}.csv"
echo ">>> Saved: $LABEL"
