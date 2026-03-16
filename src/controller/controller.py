#!/usr/bin/env python3
from __future__ import print_function
import argparse
import math
import struct
import sys

sys.path.append('pythonGateways/')
import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

CAN_X, CAN_Y, CAN_THETA, CAN_OMEGA = 20, 21, 22, 23
DT = 0.001


class PID:
    def __init__(self, kp, ki, kd, limit=3.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, target, measured, dt=DT):
        error = target - measured
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        cmd = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.limit, min(self.limit, cmd))


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


class Controller:
    def __init__(self, args):
        self.componentId = 1
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = 50102
        self.path_type = args.path
        self.pid = PID(args.Kp, args.Ki, args.Kd)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.omega = 0.0
        self.simulationStep = 0
        self.totalSimulationTime = 0

    def _pack(self, val):
        return struct.pack('=d', val)

    def _unpack(self, payload):
        return struct.unpack('=d', payload[:8])[0]

    def get_reference(self, x):
        if self.path_type == 'curved':
            return curved_reference(x)
        return 0.0

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

    def mainThread(self):
        session = vsiCommonPythonApi.connectToServer(
            self.localHost,
            self.domain,
            self.portNum,
            self.componentId,
        )
        vsiCanPythonGateway.initialize(session, self.componentId)

        try:
            vsiCommonPythonApi.waitForReset()
            self.update_clock()
            if self.totalSimulationTime == 0:
                self.totalSimulationTime = 10_000_000_000

            next_time = vsiCommonPythonApi.getSimulationTimeInNs()
            while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime:
                self.update_clock()
                if vsiCommonPythonApi.isStopRequested():
                    raise Exception('stopRequested')

                self.x = self.recv_signal(CAN_X)
                self.y = self.recv_signal(CAN_Y)
                self.theta = self.recv_signal(CAN_THETA)

                y_ref = self.get_reference(self.x)
                self.omega = self.pid.step(y_ref, self.y)
                self.send_signal(CAN_OMEGA, self.omega)

                t = vsiCommonPythonApi.getSimulationTimeInNs()
                print(f"[CTRL] t={t}ns y={self.y:.4f} y_ref={y_ref:.4f} err={y_ref - self.y:.4f} w={self.omega:.4f}")

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
                print('[CTRL] Stop requested.')
                vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
            else:
                print(f'[CTRL] ERROR: {exc}')
                raise


def main():
    parser = argparse.ArgumentParser('Controller')
    parser.add_argument('--domain', default='AF_UNIX')
    parser.add_argument('--server-url', default='localhost')
    parser.add_argument('--Kp', type=float, default=2.0)
    parser.add_argument('--Ki', type=float, default=0.1)
    parser.add_argument('--Kd', type=float, default=0.5)
    parser.add_argument('--path', default='straight', choices=['straight', 'curved'])
    Controller(parser.parse_args()).mainThread()


if __name__ == '__main__':
    main()
