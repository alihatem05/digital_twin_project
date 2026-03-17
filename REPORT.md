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
- `plot_<label>.png`
- `plot_E1_gain_sweep.png`
- `plot_E2_curved_path.png`
- `plot_E3_noise_rejection.png`
- `plot_E4_pd_vs_pid.png`

## 7. KPIs

Computed by visualizer and analysis scripts:
- Max overshoot
- Final error
- Steady-state error
- Settling time (5 cm band)

Use `aggregate_results.py` to summarize all experiment CSV files.

## 8. Results Summary Table
| Experiment | Best/Compared Config | Overshoot (m) | Settling Time (s) | Steady-State Error (m) | Notes |
|---|---|---:|---:|---:|---|
| E1 | Kp=2.0, Ki=0.1, Kd=0.5 (best observed) | 0.5048 | 9.71 | 0.0451 | Better than Kp=0.5, Ki=0.05, Kd=0.1 (SSE 0.3587, settling 15.0s). |
| E2 | Kp=2.0, Ki=0.1, Kd=0.5 on curved path | 0.5046 | 11.76 | 0.0491 | Curved path increases settling time and SSE versus straight path. |
| E3 | Kp=2.0, Ki=0.1, Kd=0.5 across noise levels | 0.5048 avg | 9.83 avg | 0.0449 avg | Robust to 0.0-0.1 noise; one run at noise 0.1 settled slower (11.61s). |
| E4 | PD vs PID on curved+noise | PD: 0.5047 / PID: 0.5044 | PD: 11.70 / PID: 11.69 | PD: 0.0435 / PID: 0.0478 | Similar transient metrics in this sampled run set; more runs needed for stable ranking. |

Data quality notes:
- The current dataset is partially complete because some long batches were interrupted.
- `aggregate_results.py` reported invalid/empty files for: `e1_results_Kp2.0_Ki0.1_Kd0.5_run2.csv`, `e1_results_Kp2.0_Ki0.1_Kd0.5_run3.csv`, and `e3_results_Kp2.0_Ki0.1_Kd0.5_noise0.1_run2.csv`.
- Conclusions above are based on valid rows in `kpi_summary.csv`.

## 9. Discussion

1. Straight-path gain sweep (E1) showed clear improvement from low gains to moderate gains.
2. The low-gain case (`Kp=0.5, Ki=0.05, Kd=0.1`) had large residual tracking error and did not settle by the end of the run window.
3. The moderate-gain case (`Kp=2.0, Ki=0.1, Kd=0.5`) reduced SSE substantially and settled faster, making it the best observed candidate in current runs.
4. Curved tracking (E2) was harder than straight tracking: settling time increased from about 9.7s to 11.8s and SSE increased from about 0.045 to 0.049.
5. Noise robustness (E3) was acceptable up to tested noise levels; KPI drift was small, with the largest impact seen in settling time at one 0.1-noise run.
6. PD vs PID (E4) produced close transient behavior in the sampled runs; additional repetitions are recommended before claiming strong superiority in this setup.

## 10. Conclusion

The repository provides a working 3-client IVSI line-following system with automated E1-E4 experiment scripts, curved-path support, noise testing, KPI aggregation, and plotting. Current measured data confirms that the moderate PID tuning (`Kp=2.0, Ki=0.1, Kd=0.5`) significantly outperforms the lower-gain case in straight-line tracking, while curved-path and noise conditions degrade settling behavior as expected. The framework is complete and reproducible; remaining work is only to rerun missing/invalid trials if fully dense statistics are required.