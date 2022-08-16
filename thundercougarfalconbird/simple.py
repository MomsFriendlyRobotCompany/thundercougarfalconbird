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
class MassSpringDampener:
    mass: float
    springConst: float
    dampeningCoeff: float

    def eqns(self, dt, xi, u):
        """
        t: time
        xi: states
        u: control signal
        """
        dx, x = xi

        m = self.mass
        k = self.springConst
        d = self.dampenerCoeff

        ddx = (u - d*dx - k*x)/m
        dx = dx

        return np.array([ddx, dx])