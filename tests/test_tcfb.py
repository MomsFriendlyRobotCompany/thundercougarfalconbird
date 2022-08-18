import pytest
from thundercougarfalconbird import *


def test_wheeled():
    r = DifferentialDriveKinematics(1,2)

    with pytest.raises(TypeError):
        r = DifferentialDriveKinematics()

    r = RoombaCreate()
    assert True


def test_uav():
    ParrotDrone()

    with pytest.raises(Exception):
        r = Drone()


def test_simple():
    Vanderpol(5)
    with pytest.raises(TypeError):
        Vanderpol()

    MassSpringDamper(1,2,3)
    with pytest.raises(TypeError):
        MassSpringDamper()