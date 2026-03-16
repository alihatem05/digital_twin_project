#!/usr/bin/env python3
import csv
import os
import glob

SUMMARY_FILE = 'kpi_summary.csv'

def compute_kpis(csv_file):
    times, xs, ys, thetas, omegas, errors = [], [], [], [], [], []
    with open(csv_file) as f:
        for row in csv.DictReader(f):
            times.append(float(row['time_ns']) * 1e-9)
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            thetas.append(float(row['theta']))
            omegas.append(float(row['omega']))
            errors.append(float(row['lateral_error']))

    if not times:
        return None

    max_overshoot = max(abs(e) for e in errors)
    final_error = abs(errors[-1])
    steady_start = 5.0
    steady_errors = [abs(e) for t, e in zip(times, errors) if t > steady_start]
    sse = sum(steady_errors) / len(steady_errors) if steady_errors else 0

    threshold = 0.05
    settling_time = times[-1]
    for i, (t, e) in enumerate(zip(times, errors)):
        if all(abs(err) < threshold for err in errors[i:]):
            settling_time = t
            break

    return {
        'max_overshoot': max_overshoot,
        'final_error': final_error,
        'steady_state_error': sse,
        'settling_time': settling_time,
        'duration': times[-1]
    }

def main():
    experiments = {
        'E1': 'e1_results_*.csv',
        'E2': 'e2_results_*.csv',
        'E3': 'e3_results_*.csv',
        'E4': 'e4_results_*.csv'
    }

    summary_rows = []

    for exp, pattern in experiments.items():
        files = glob.glob(pattern)
        if not files:
            continue
        print(f"\n=== {exp} Results ===")
        for f in sorted(files):
            label = os.path.basename(f).replace('e1_results_', '').replace('e2_results_', '').replace('e3_results_', '').replace('e4_results_', '').replace('.csv', '')
            kpis = compute_kpis(f)
            if kpis is None:
                print(f"{label}: empty or invalid CSV")
                continue
            print(f"{label}: Overshoot={kpis['max_overshoot']:.4f}, SSE={kpis['steady_state_error']:.4f}, Settling={kpis['settling_time']:.1f}s")
            summary_rows.append({
                'experiment': exp,
                'label': label,
                'max_overshoot': f"{kpis['max_overshoot']:.6f}",
                'final_error': f"{kpis['final_error']:.6f}",
                'steady_state_error': f"{kpis['steady_state_error']:.6f}",
                'settling_time': f"{kpis['settling_time']:.6f}",
                'duration': f"{kpis['duration']:.6f}",
                'source_file': f,
            })

    if summary_rows:
        with open(SUMMARY_FILE, 'w', newline='') as f:
            w = csv.DictWriter(
                f,
                fieldnames=['experiment', 'label', 'max_overshoot', 'final_error', 'steady_state_error', 'settling_time', 'duration', 'source_file'],
            )
            w.writeheader()
            w.writerows(summary_rows)
        print(f"\nSaved KPI summary: {SUMMARY_FILE}")
    else:
        print("\nNo experiment result files found.")

if __name__ == '__main__':
    main()