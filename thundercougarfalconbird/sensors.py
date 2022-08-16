# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2022 Kevin Walchko
# see LICENSE for full details
##############################################
import numpy as np
from dataclasses import dataclass


@dataclass
class SensorNoise:
    mu: float
    sigma: float

    def noise(self, num=1000, dim=1):
        return np.random.normal(self.mu, self.sigma, (num, dim))