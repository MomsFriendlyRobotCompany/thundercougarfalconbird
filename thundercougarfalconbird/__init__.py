# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2022 Kevin Walchko
# see LICENSE for full details
##############################################
from importlib.metadata import version

from .models import DifferentialDriveKinematics, RoombaCreate
from .simple import Vanderpol, MassSpringDamper
from .uav import Drone, ParrotDrone
from .space import Satellite, SatelliteWithWheels

__license__ = 'MIT'
__author__ = 'Kevin Walchko'
__version__ = version("thundercougarfalconbird")