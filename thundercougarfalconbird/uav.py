# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2022 Kevin Walchko
# see LICENSE for full details
##############################################
from dataclasses import dataclass
import numpy as np
from squaternion import Quaternion


# @dataclass
class Drone:
    def __init__(self, params):
        """
        m: mass [kg]
        l: motor arm length [m]
        J: rotational moment of inertia [kg*m^2]
        kf: motor force const
        km: motor torque const
        """
        # if params is None:
        #     # params = {}
        #     # params = {
        #     #     'm': 0.329,
        #     #     'l': 0.1785,
        #     #     'J': [2.238e-3, 2.986e-3, 4.804e-3],
        #     #     'kf': 7.00e-7,
        #     #     'km': 2.423e-6,
        #     #     'tau': 4.718e-3,
        #     #     'nmax': 1047
        #     # }
        #     raise Exception("No parameters given")
        if not isinstance(params, dict):
            raise Exception("Parameters are not a dictionary")

        self.m = params.get('m', 1)
        self.gg = params['km'] / params['kf']
        self.J = params['J']
        self.kf = params['kf']
        self.L = params['l']

        # self.q = Quaternion()

    def eqns(self, t, x, u):
        """
        x = [p,v,w,q] => 13
        """
        gg = self.gg
        kf = self.kf
        L = self.L
        m = self.m

        p,q,r = x[6:9]

        n1,n2,n3,n4 = u

        # R converts body to inertial
        # R = np.array(self.q.to_rot())
        qq = Quaternion(*x[9:])
        R = np.array(qq.to_rot())

        # WARN: should I be converting to inertial space?
        # convert inertial gravity to body frame, hence R.T
        # g = R.T @ np.array([0,0,9.8])
        g = np.array([0,0,9.8])

        Jx, Jy, Jz = self.J

        F1 = kf*n1**2
        F2 = kf*n2**2
        F3 = kf*n3**2
        F4 = kf*n4**2
        F = np.array([0,0,-F1-F2-F3-F4])

        MJ = np.array([
            L*((F2+F3)-(F1+F4))/Jx,
            L*((F1+F3)-(F2+F4))/Jy,
            0.1*(F1+F2-F3-F4)/Jz
        ])

        wJw = np.array([
            (Jy - Jz)/Jx*q*r,
            (Jz - Jx)/Jy*p*r,
            (Jx - Jy)/Jz*p*q,
        ])

        # ans = [p,v,w,q] => 13
        ans = np.zeros(13)

        v = x[3:6]
        w = x[6:9]

        ans[:3] = v  # dot pos
        ans[3:6] = R.dot(F)/m+g # - np.cross(w,v) # dot vel
        ans[6:9] = MJ-wJw                # dot w

        # q = Quaternion(*x[9:])
        w = Quaternion(0,*w)
        ans[9:] = 0.5*qq*w # dot q

        return ans


class ParrotDrone(Drone):
    def __init__(self):
        params = {
            'm': 0.329,
            'l': 0.1785,
            'J': [2.238e-3, 2.986e-3, 4.804e-3],
            'kf': 7.00e-7,
            'km': 2.423e-6,
            'tau': 4.718e-3,
            'nmax': 1047
        }
        super().__init__(params)