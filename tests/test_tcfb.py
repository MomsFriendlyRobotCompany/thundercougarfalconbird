import pytest
from thundercougarfalconbird import *


def test_wheeled():
    r = DifferentialDriveKinematics(1,2)

    with pytest.raises(TypeError):
        r = DifferentialDriveKinematics()

    r = RoombaCreate()
    assert True


def test_uav():
    assert ParrotDrone()

    with pytest.raises(Exception):
        r = Drone()