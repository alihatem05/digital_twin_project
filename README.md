# Line-Following Robot with PID Control

This project implements a 3-client IVSI line-following system:
- Plant/Simulator: `src/plant/plant.py`
- PID Controller: `src/controller/controller.py`
- Visualizer/Logger: `src/visualizer/visualizer.py`

## Project Layout

- Experiment runners:
  - `run_all_experiments.sh` (E1)
  - `run_experiment_E2.sh`
  - `run_experiment_E3.sh`
  - `run_experiment_E4.sh`
- Single-run helper: `run_experiment.sh`
- Analysis:
  - `aggregate_results.py`
  - `plot_all.py`
  - `plot_results.py`
- Report: `REPORT.md`

## Requirements

- Linux with Bash
- Python 3
- Matplotlib and NumPy
- Innexis VSI tools available at:
  - `/data/tools/pave/innexis_home/vsi_2025.2/`

## Quick Start

1. Open terminal in project root:
```bash
cd /home/ubuntu/capstone_ali/ali_hatem
```

2. Run one test experiment:
```bash
./run_experiment.sh 2.0 0.1 0.5 quick_test straight 0.0 AF_INET localhost
```

3. Generate KPIs and plots:
```bash
python3 aggregate_results.py
python3 plot_all.py
```

## Running All Experiments

Run in this order:
```bash
./run_all_experiments.sh
./run_experiment_E2.sh
./run_experiment_E3.sh
./run_experiment_E4.sh
```

After completion:
```bash
python3 aggregate_results.py
python3 plot_all.py
```

## Reduced Runs (Faster)

For quick validation:
```bash
RUNS_PER_GAIN=1 ./run_all_experiments.sh
RUNS=1 ./run_experiment_E2.sh
RUNS=1 ./run_experiment_E3.sh
RUNS=1 ./run_experiment_E4.sh
```

## Simulator Interaction

When simulator/client terminals open, in the `vsiSim` console type:
```text
reset
run
```

If a run appears stuck, issue `reset` then `run` again in `vsiSim`.

## Output Files

- Per-run archives:
  - `results_<label>.csv`
  - `trajectory_<label>.csv`
- Experiment archives:
  - `e1_results_*.csv`, `e1_traj_*.csv`
  - `e2_results_*.csv`, `e2_traj_*.csv`
  - `e3_results_*.csv`, `e3_traj_*.csv`
  - `e4_results_*.csv`, `e4_traj_*.csv`
- KPI summary:
  - `kpi_summary.csv`
- Plots:
  - `plot_<label>.png`
  - `plot_E1_gain_sweep.png`
  - `plot_E2_curved_path.png`
  - `plot_E3_noise_rejection.png`
  - `plot_E4_pd_vs_pid.png`

## Notes

- Some long runs may be interrupted if a batch is stopped manually.
- `aggregate_results.py` skips empty or invalid CSV files and still writes a summary from valid runs.
