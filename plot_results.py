import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

times, xs, ys, thetas, omegas, errors = [], [], [], [], [], []
with open('visualizer_log.csv') as f:
    for row in csv.DictReader(f):
        times.append(float(row['time_ns']) * 1e-9)
        xs.append(float(row['x']))
        ys.append(float(row['y']))
        thetas.append(float(row['theta']))
        omegas.append(float(row['omega']))
        errors.append(float(row['lateral_error']))

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Line-Following Robot - PID Control Results', fontsize=14)

axes[0,0].plot(xs, ys, 'b-', linewidth=1)
axes[0,0].axhline(y=0, color='r', linestyle='--', label='Reference (y=0)')
axes[0,0].set_xlabel('x (m)'); axes[0,0].set_ylabel('y (m)')
axes[0,0].set_title('Robot Trajectory'); axes[0,0].legend(); axes[0,0].grid(True)

axes[0,1].plot(times, ys, 'b-', linewidth=1)
axes[0,1].axhline(y=0, color='r', linestyle='--', label='Reference')
axes[0,1].set_xlabel('Time (s)'); axes[0,1].set_ylabel('Lateral Error y (m)')
axes[0,1].set_title('Lateral Error vs Time'); axes[0,1].legend(); axes[0,1].grid(True)

axes[1,0].plot(times, omegas, 'g-', linewidth=1)
axes[1,0].axhline(y=0, color='r', linestyle='--')
axes[1,0].set_xlabel('Time (s)'); axes[1,0].set_ylabel('omega (rad/s)')
axes[1,0].set_title('Steering Command vs Time'); axes[1,0].grid(True)

axes[1,1].plot(times, thetas, 'm-', linewidth=1)
axes[1,1].axhline(y=0, color='r', linestyle='--')
axes[1,1].set_xlabel('Time (s)'); axes[1,1].set_ylabel('theta (rad)')
axes[1,1].set_title('Heading Angle vs Time'); axes[1,1].grid(True)

plt.tight_layout()
plt.savefig('plot_single_run.png', dpi=150)
print("Saved: plot_single_run.png")

print("\n=== KPI SUMMARY (Kp=2.0, Ki=0.1, Kd=0.5) ===")
print(f"Max overshoot:       {max(abs(y) for y in ys):.4f} m")
print(f"Final lateral error: {abs(ys[-1]):.4f} m")
steady = errors[len(errors)//2:]
print(f"Steady-state error:  {np.mean(steady):.4f} m")
print(f"Simulation time:     {times[-1]:.1f} s")
