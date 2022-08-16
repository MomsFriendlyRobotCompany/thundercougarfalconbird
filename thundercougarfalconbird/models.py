# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2022 Kevin Walchko
# see LICENSE for full details
##############################################
from dataclasses import dataclass
import numpy as np

@dataclass
class DifferentialDriveKinematics:
    """
    https://www.mathworks.com/help/robotics/ug/mobile-robot-kinematics-equations.html

    Differential Drive Kinematics

    (x,y): position [m]
    theta: heading [rad]
    phiL|R: wheel speed [m/s]
    r: wheel radius [r]
    d: track width, or distance between wheels [m]
    v: speed [m/s]
    w: vehicle heading angular velocity [rad/s]
    """
    r: float
    d: float

    def eqns(self, t, x, u):
        theta = x[2]

        phiL = u[0]
        phiR = u[1]

        v = self.r*(phiR + phiL)/2
        w = self.r/(2*self.d) * (phiR - phiL)

        ret = np.zeros(3)
        ret[0] = np.cos(theta)*v # dx
        ret[1] = np.sin(theta)*v # dy
        ret[2] = w               # dtheta

        return ret


class RoombaCreate(DifferentialDriveKinematics):
    def __init__(self):
        super().__init__(0.070, 0.235)


@dataclass
class PointMass:

    def eqns(self, dt, x, u=None):
        """
        x = [pos, vel, quaternion]
        u = [accels, gyros]
        """
        if u is None:
            u = np.zeros(6)
        else:
            a = u[:3]
            w = u[3:6]

        q = Quaternion(*x[6:])

        dp = x[3:6]

        rot = np.array(q.to_rot())
        g = np.array([0,0,9.8])
        dv = rot @ a - g

        W = Quaternion(0,*w)
        dq = 0.5*q*W

        xx = np.array([0,0,0, 0,0,0, dq.w,dq.x,dq.y,dq.z])
        xx[:3] = dp
        xx[3:6] = dv

        return xx
