# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2022 Kevin Walchko
# see LICENSE for full details
##############################################
from dataclasses import dataclass
import numpy as np

@dataclass
class Vanderpol:
    """
    Vanderpol Oscillator

    mu: dampening parameter
    """
    dampeningCoeff: float

    def eqns(self, dt, xi, u):
        dx, x = xi
        # mu = 4.0 # damping

        ddx = self.dampeningCoeff*(1-x**2)*dx-x
        dx = dx

        return np.array([ddx, dx])


@dataclass
class MassSpringDamper:
    """
    state: [velocity, position]
    """
    mass: float
    springConst: float
    dampingCoeff: float

    def eqns(self, dt, xi, u):
        """
        t: time
        xi: states
        u: control signal
        """
        vel, pos = xi

        m = self.mass
        k = self.springConst
        c = self.dampingCoeff

        # m*accel + c*vel + k*pos = u
        accel = (u - c*vel - k*pos)/m

        return np.array([accel, vel])