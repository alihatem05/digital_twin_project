# Line-Following Robot with PID Control

## 1. Overview

This project implements a differential-drive line-following robot using a PID controller over the Innexis Virtual System Interconnect (IVSI) backplane.

System structure:
- Client 0: Plant/Simulator (`src/plant/plant.py`)
- Client 1: Controller (`src/controller/controller.py`)
- Client 2: Visualizer/Logger (`src/visualizer/visualizer.py`)

## 2. Modeling and Assumptions

Robot kinematics use the unicycle model:

```text
dx/dt = v cos(theta)
dy/dt = v sin(theta)
dtheta/dt = omega
```

Assumptions used in implementation:
- Constant forward speed: `v = 1.0 m/s`
- Discrete integration timestep: `dt = 0.001 s`
- Steering command saturation: `omega in [-3.0, 3.0] rad/s`
- Noise model (E3): additive Gaussian disturbance on lateral velocity

## 3. Control Design

Lateral tracking is controlled by PID:

```text
e(t) = y_ref(x) - y
omega = Kp*e + Ki*integral(e) + Kd*de/dt
```

Implemented path references:
- Straight: `y_ref = 0`
- Curved: piecewise circular-arc profile (rise/fall/rise sections)

## 4. IVSI/CAN Interface

CAN IDs:
- `20`: x
- `21`: y
- `22`: theta
- `23`: omega

Each client connects through VSI Python gateways and runs in lockstep with simulation time.

## 5. Experiments

### E1: PID Gain Sweep (Straight Path)

Script: `run_all_experiments.sh`

Gain sets:
- `(0.5, 0.05, 0.1)`
- `(2.0, 0.1, 0.5)`
- `(3.0, 0.2, 0.8)`
- `(5.0, 0.5, 1.0)`
- `(10.0, 1.0, 2.0)`

Each set runs multiple random spawns (`RUNS_PER_GAIN`, default 3).

### E2: Curved Path Robustness

Script: `run_experiment_E2.sh`

Runs the baseline best controller on curved reference with multiple random spawns.

### E3: Noise and Disturbance Rejection

Script: `run_experiment_E3.sh`

Noise levels tested: `0.0, 0.01, 0.05, 0.1, 0.2` (m/s), each with multiple runs.

### E4: PD vs PID Ablation

Script: `run_experiment_E4.sh`

Compares:
- PD: `Ki = 0.0`
- PID: `Ki = 0.1`

Both tested on curved path with noise `0.05`.

## 6. Output Files

Per-run logs:
- `trajectory_data.csv`
- `visualizer_log.csv`

Archived experiment outputs:
- `e1_results_*.csv`, `e1_traj_*.csv`
- `e2_results_*.csv`, `e2_traj_*.csv`
- `e3_results_*.csv`, `e3_traj_*.csv`
- `e4_results_*.csv`, `e4_traj_*.csv`

Plots:
- `results_<label>.png`

## 7. KPIs

Computed by visualizer and analysis scripts:
- Max overshoot
- Final error
- Steady-state error
- Settling time (5 cm band)

Use `aggregate_results.py` to summarize all experiment CSV files.

## 8. Results Summary Table

Populate this section after running all experiments.

| Experiment | Best/Compared Config | Overshoot (m) | Settling Time (s) | Steady-State Error (m) | Notes |
|---|---|---:|---:|---:|---|
| E1 | TODO | TODO | TODO | TODO | TODO |
| E2 | TODO | TODO | TODO | TODO | TODO |
| E3 | TODO | TODO | TODO | TODO | TODO |
| E4 | PD vs PID | TODO | TODO | TODO | TODO |

## 9. Discussion

Suggested discussion points:
- Effect of increasing `Kp` on rise speed and overshoot
- Effect of `Ki` on removing residual bias
- Effect of `Kd` on damping oscillations
- Why curved tracking is harder than straight tracking
- Why PID outperforms PD for steady-state bias rejection under noise

## 10. Conclusion

The repository now contains a complete 3-client IVSI line-following implementation with automated E1-E4 experiment scripts, curved-path support, noise testing, and KPI generation. Final numerical conclusions should be filled from the generated CSV outputs and `aggregate_results.py` summary.