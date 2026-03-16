#!/usr/bin/env python3
from __future__ import print_function
import argparse
import csv
import math
import struct
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.append('pythonGateways/')
import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

CAN_X, CAN_Y, CAN_THETA, CAN_OMEGA = 20, 21, 22, 23


def curved_reference(x):
    radius = 20.0
    seg = 15.0

    def arc_height(u):
        u = max(0.0, min(seg, u))
        return radius - math.sqrt(max(0.0, radius * radius - u * u))

    if x <= seg:
        return arc_height(x)
    if x <= 2.0 * seg:
        return arc_height(seg) - arc_height(x - seg)
    if x <= 3.0 * seg:
        return arc_height(x - 2.0 * seg)
    return arc_height(seg)


class Visualizer:
    def __init__(self, args):
        self.componentId = 2
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = 50103
        self.label = args.label
        self.path_type = args.path
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.omega = 0.0
        self.simulationStep = 0
        self.totalSimulationTime = 0
        self.time_hist = []
        self.x_hist = []
        self.y_hist = []
        self.theta_hist = []
        self.omega_hist = []
        self.max_overshoot = 0.0
        self.steady_errors = []
        self.steady_start_ns = 5_000_000_000

    def _unpack(self, payload):
        return struct.unpack('=d', payload[:8])[0]

    def reference(self, x):
        if self.path_type == 'curved':
            return curved_reference(x)
        return 0.0

    def recv_signal(self, can_id):
        payload = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, can_id)
        return self._unpack(payload)

    def update_clock(self):
        self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
        self.simulationStep = vsiCommonPythonApi.getSimulationStep()

    def settling_time(self, errors):
        threshold = 0.05
        settle = self.time_hist[-1]
        for i, err in enumerate(errors):
            if all(abs(v) < threshold for v in errors[i:]):
                settle = self.time_hist[i]
                break
        return settle

    def save_plot(self):
        errors = [y - self.reference(x) for y, x in zip(self.y_hist, self.x_hist)]
        settle = self.settling_time(errors)
        sse = float(np.mean(self.steady_errors)) if self.steady_errors else 0.0

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(f'Line-Following Robot - {self.label}', fontsize=14, fontweight='bold')

        axes[0, 0].plot(self.x_hist, self.y_hist, 'b-', linewidth=1, label='Robot path')
        if self.path_type == 'straight':
            axes[0, 0].axhline(0, color='r', linestyle='--', linewidth=2, label='Reference')
        else:
            x_min, x_max = min(self.x_hist), max(self.x_hist)
            x_line = np.linspace(x_min, x_max, 100)
            y_line = [self.reference(v) for v in x_line]
            axes[0, 0].plot(x_line, y_line, 'r--', linewidth=2, label='Reference')
        axes[0, 0].set_xlabel('x (m)')
        axes[0, 0].set_ylabel('y (m)')
        axes[0, 0].set_title('Trajectory')
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        axes[0, 1].plot(self.time_hist, errors, 'b-', linewidth=1)
        axes[0, 1].axhline(0, color='r', linestyle='--', linewidth=2)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Lateral Error (m)')
        axes[0, 1].set_title('Error vs Time')
        axes[0, 1].grid(True)

        axes[1, 0].plot(self.time_hist, self.omega_hist, 'g-', linewidth=1)
        axes[1, 0].axhline(0, color='r', linestyle='--')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('w (rad/s)')
        axes[1, 0].set_title('Steering Command')
        axes[1, 0].grid(True)

        angles = [math.degrees(v) for v in self.theta_hist]
        axes[1, 1].plot(self.time_hist, angles, 'm-', linewidth=1)
        axes[1, 1].axhline(0, color='r', linestyle='--')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('theta (deg)')
        axes[1, 1].set_title('Heading Angle')
        axes[1, 1].grid(True)

        kpi = (
            f"Label: {self.label}\n"
            f"Max Overshoot : {self.max_overshoot:.4f} m\n"
            f"Final Error   : {abs(errors[-1]):.4f} m\n"
            f"Steady-State  : {sse:.4f} m\n"
            f"Settling Time : {settle:.1f} s"
        )
        fig.text(0.02, 0.01, kpi, fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        out_name = f'plot_{self.label}.png'
        plt.savefig(out_name, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"[VIZ] Saved: {out_name}")

    def mainThread(self):
        session = vsiCommonPythonApi.connectToServer(
            self.localHost,
            self.domain,
            self.portNum,
            self.componentId,
        )
        vsiCanPythonGateway.initialize(session, self.componentId)

        with open('visualizer_log.csv', 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['time_ns', 'x', 'y', 'theta', 'omega', 'lateral_error'])

            try:
                vsiCommonPythonApi.waitForReset()
                self.update_clock()
                if self.totalSimulationTime == 0:
                    self.totalSimulationTime = 30_000_000_000

                next_time = vsiCommonPythonApi.getSimulationTimeInNs()
                while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime:
                    self.update_clock()
                    if vsiCommonPythonApi.isStopRequested():
                        raise Exception('stopRequested')

                    self.x = self.recv_signal(CAN_X)
                    self.y = self.recv_signal(CAN_Y)
                    self.theta = self.recv_signal(CAN_THETA)
                    self.omega = self.recv_signal(CAN_OMEGA)

                    t_ns = vsiCommonPythonApi.getSimulationTimeInNs()
                    err = self.y - self.reference(self.x)
                    self.max_overshoot = max(self.max_overshoot, abs(err))
                    if t_ns > self.steady_start_ns:
                        self.steady_errors.append(abs(err))

                    self.time_hist.append(t_ns * 1e-9)
                    self.x_hist.append(self.x)
                    self.y_hist.append(self.y)
                    self.theta_hist.append(self.theta)
                    self.omega_hist.append(self.omega)
                    writer.writerow([t_ns, self.x, self.y, self.theta, self.omega, err])

                    if len(self.time_hist) % 100 == 0:
                        print(f"[VIZ] t={t_ns * 1e-9:.1f}s y={self.y:.4f} err={err:.4f}")

                    self.update_clock()
                    if vsiCommonPythonApi.isStopRequested():
                        raise Exception('stopRequested')
                    next_time += self.simulationStep
                    now = vsiCommonPythonApi.getSimulationTimeInNs()
                    if now >= next_time:
                        continue
                    if next_time > self.totalSimulationTime:
                        vsiCommonPythonApi.advanceSimulation(self.totalSimulationTime - now)
                        break
                    vsiCommonPythonApi.advanceSimulation(next_time - now)
            except Exception as exc:
                if str(exc) == 'stopRequested':
                    print('[VIZ] Stop requested.')
                    vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
                else:
                    print(f'[VIZ] ERROR: {exc}')
                    raise

        if len(self.time_hist) > 10:
            self.save_plot()

        sse = float(np.mean(self.steady_errors)) if self.steady_errors else 0.0
        final_error = abs(self.y_hist[-1] - self.reference(self.x_hist[-1])) if self.y_hist else 0.0
        sim_time = self.time_hist[-1] if self.time_hist else 0.0
        print(f"\n=== KPI SUMMARY ({self.label}) ===")
        print(f"Max overshoot    : {self.max_overshoot:.4f} m")
        print(f"Final error      : {final_error:.4f} m")
        print(f"Steady-state err : {sse:.4f} m")
        print(f"Sim duration     : {sim_time:.1f} s")


def main():
    parser = argparse.ArgumentParser('Visualizer')
    parser.add_argument('--domain', default='AF_UNIX')
    parser.add_argument('--server-url', default='localhost')
    parser.add_argument('--label', default='run')
    parser.add_argument('--path', default='straight', choices=['straight', 'curved'])
    Visualizer(parser.parse_args()).mainThread()


if __name__ == '__main__':
    main()
