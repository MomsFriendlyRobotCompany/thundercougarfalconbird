# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2022 Kevin Walchko
# see LICENSE for full details
##############################################
from importlib.metadata import version

from .models import DifferentialDriveKinematics, RoombaCreate
from .simple import Vanderpol, MassSpringDampener
from .uav import Drone, ParrotDrone

__license__ = 'MIT'
__author__ = 'Kevin Walchko'
__version__ = version("thundercougarfalconbird")