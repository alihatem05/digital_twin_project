#!/usr/bin/env python3
from __future__ import print_function
import argparse
import csv
import math
import random
import struct
import sys

sys.path.append('pythonGateways/')
import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

CAN_X, CAN_Y, CAN_THETA, CAN_OMEGA = 20, 21, 22, 23
V = 1.0
DT = 0.001


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


def reference_y(x, path_type):
    if path_type == 'curved':
        return curved_reference(x)
    return 0.0


class Plant:
    def __init__(self, args):
        self.componentId = 0
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = 50101
        self.noise_std = args.noise
        self.path_type = args.path
        self.x = 0.0
        self.y = 0.5
        self.theta = 0.1
        self.omega = 0.0
        self.simulationStep = 0
        self.totalSimulationTime = 0

    def _pack(self, val):
        return struct.pack('=d', val)

    def _unpack(self, payload):
        return struct.unpack('=d', payload[:8])[0]

    def send_signal(self, can_id, value):
        vsiCanPythonGateway.setCanId(can_id)
        vsiCanPythonGateway.setDataLengthInBits(64)
        vsiCanPythonGateway.setCanPayloadBits(self._pack(value), 0, 64)
        vsiCanPythonGateway.sendCanPacket()

    def recv_signal(self, can_id):
        payload = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, can_id)
        return self._unpack(payload)

    def update_clock(self):
        self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
        self.simulationStep = vsiCommonPythonApi.getSimulationStep()

    def update_kinematics(self):
        noise = random.gauss(0.0, self.noise_std) if self.noise_std > 0.0 else 0.0
        self.x += V * math.cos(self.theta) * DT
        self.y += (V * math.sin(self.theta) + noise) * DT
        self.theta += self.omega * DT
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def mainThread(self):
        session = vsiCommonPythonApi.connectToServer(
            self.localHost,
            self.domain,
            self.portNum,
            self.componentId,
        )
        vsiCanPythonGateway.initialize(session, self.componentId)

        with open('trajectory_data.csv', 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['time_ns', 'x', 'y', 'theta', 'omega', 'y_ref', 'error'])

            try:
                vsiCommonPythonApi.waitForReset()
                self.update_clock()
                if self.totalSimulationTime == 0:
                    self.totalSimulationTime = 60_000_000_000

                next_time = vsiCommonPythonApi.getSimulationTimeInNs()
                while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime:
                    self.update_clock()
                    if vsiCommonPythonApi.isStopRequested():
                        raise Exception('stopRequested')

                    y_ref = reference_y(self.x, self.path_type)
                    self.send_signal(CAN_X, self.x)
                    self.send_signal(CAN_Y, self.y)
                    self.send_signal(CAN_THETA, self.theta)

                    self.omega = self.recv_signal(CAN_OMEGA)
                    self.update_kinematics()

                    t = vsiCommonPythonApi.getSimulationTimeInNs()
                    err = self.y - y_ref
                    writer.writerow([t, self.x, self.y, self.theta, self.omega, y_ref, err])
                    print(f"[PLANT] t={t}ns x={self.x:.3f} y={self.y:.4f} y_ref={y_ref:.4f} err={err:.4f} w={self.omega:.4f}")

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
                    print('[PLANT] Stop requested.')
                    vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
                else:
                    print(f'[PLANT] ERROR: {exc}')
                    raise

        print('[PLANT] Done.')


def main():
    parser = argparse.ArgumentParser('Plant')
    parser.add_argument('--domain', default='AF_UNIX')
    parser.add_argument('--server-url', default='localhost')
    parser.add_argument('--noise', type=float, default=0.0)
    parser.add_argument('--path', default='straight', choices=['straight', 'curved'])
    Plant(parser.parse_args()).mainThread()


if __name__ == '__main__':
    main()
