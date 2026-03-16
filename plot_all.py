import csv
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_csv_dict(path):
    if not os.path.exists(path):
        return None
    out = {}
    with open(path) as handle:
        for row in csv.DictReader(handle):
            for key, val in row.items():
                out.setdefault(key, []).append(float(val))
    return out


def pick_result_file(label):
    root_name = f"results_{label}.csv"
    nested_name = os.path.join("results", root_name)
    if os.path.exists(root_name):
        return root_name
    if os.path.exists(nested_name):
        return nested_name
    return None


def get_kpis(data):
    if not data:
        return None
    y_vals = data.get('y', data.get('lateral_error', []))
    t_vals = [t * 1e-9 for t in data.get('time_ns', [])]
    abs_err = [abs(v) for v in y_vals]
    mid = len(abs_err) // 2
    return {
        'time': t_vals,
        'y': y_vals,
        'max_overshoot': max(abs_err) if abs_err else 0.0,
        'steady_state': float(np.mean(abs_err[mid:])) if abs_err[mid:] else 0.0,
        'final_error': abs_err[-1] if abs_err else 0.0,
    }


def run_section(title, rows, out_name):
    print(f"\n=== {title} ===")
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(title, fontsize=13)
    print(f"{'Case':<28} {'Max Overshoot':>15} {'Steady-State Err':>18} {'Final Err':>12}")
    print("-" * 78)

    for label, display, color in rows:
        in_file = pick_result_file(label)
        if not in_file:
            continue
        data = read_csv_dict(in_file)
        k = get_kpis(data)
        if not k:
            continue
        print(f"{display:<28} {k['max_overshoot']:>15.4f} {k['steady_state']:>18.4f} {k['final_error']:>12.4f}")
        axes[0].plot(k['time'], k['y'], color=color, label=display, linewidth=1.5)
        axes[1].plot(k['time'], [abs(v) for v in k['y']], color=color, label=display, linewidth=1.5)

    axes[0].axhline(0, color='black', linestyle='--', linewidth=0.8)
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Lateral Error (m)')
    axes[0].set_title('Lateral Error vs Time')
    if axes[0].lines:
        axes[0].legend()
    axes[0].grid(True)

    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('|Error| (m)')
    axes[1].set_title('Absolute Error vs Time')
    if axes[1].lines:
        axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig(out_name, dpi=150)
    print(f"Saved: {out_name}")


def main():
    run_section(
        'E1: PID Gain Sweep - Straight Path',
        [
            ('E1_Kp0.5_Ki0_Kd0.1', 'Kp=0.5 Ki=0 Kd=0.1', 'red'),
            ('E1_Kp2_Ki0.1_Kd0.5', 'Kp=2 Ki=0.1 Kd=0.5', 'blue'),
            ('E1_Kp5_Ki0.2_Kd1', 'Kp=5 Ki=0.2 Kd=1', 'green'),
            ('E1_Kp10_Ki1_Kd2', 'Kp=10 Ki=1 Kd=2', 'orange'),
        ],
        'plot_E1_gain_sweep.png',
    )

    run_section(
        'E2: Curved Path Robustness',
        [
            ('E2_curved_Kp2_Ki0.1_Kd0.5', 'Kp=2 Ki=0.1 Kd=0.5', 'blue'),
            ('E2_curved_Kp5_Ki0.2_Kd1', 'Kp=5 Ki=0.2 Kd=1', 'green'),
        ],
        'plot_E2_curved_path.png',
    )

    run_section(
        'E3: Noise and Disturbance Rejection',
        [
            ('E3_noise0.01', 'Noise=0.01', 'blue'),
            ('E3_noise0.05', 'Noise=0.05', 'green'),
            ('E3_noise0.1', 'Noise=0.1', 'orange'),
            ('E3_noise0.2', 'Noise=0.2', 'red'),
        ],
        'plot_E3_noise_rejection.png',
    )

    run_section(
        'E4: PD vs PID - Curved Path with Noise',
        [
            ('E4_PD_curved_noise', 'PD (Ki=0)', 'red'),
            ('E4_PID_curved_noise', 'PID (Ki=0.1)', 'blue'),
        ],
        'plot_E4_pd_vs_pid.png',
    )

    print('\n=== ALL PLOTS SAVED IN PROJECT ROOT ===')


if __name__ == '__main__':
    main()
