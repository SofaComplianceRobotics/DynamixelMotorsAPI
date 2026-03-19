import time
import logging
import pytest
import os
import sys
from math import pi

sys.path.append(os.path.dirname(os.path.realpath(__file__))+'/..')

from dynamixelmotorsapi import DynamixelMotors
from dynamixelmotorsapi import listFTDIDevices, listUnusedFTDIDevices, listUsedFTDIDevices


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

@pytest.fixture
def motors():
    config = [
                {
                    "id": 0,
                    "model": "XM430-W210",
                    "pulley_radius": 20,
                    "pulse_center": 2048,
                    "max_vel": 1000
                },
                {
                    "id": [1, 2, 3],
                    "model": "L54-30-S400-R",
                    "pulley_radius": 20,
                    "pulse_center": 0,
                    "max_vel": 500
                },
            ]
    print("CONFIG ", config)
    return DynamixelMotors.from_dicts(config)

@pytest.fixture
def emio_motors():
    config = [
                {
                    "id": [0, 1, 2, 3],
                    "model": "XM430-W210",
                    "pulley_radius": 20,
                    "pulse_center": 2048,
                    "max_vel": 1000
                },
            ]
    print("CONFIG ", config)
    return DynamixelMotors.from_dicts(config)

def test_creation(motors):
    """Test the creation of a DynamixelMotors instance."""
    
    assert isinstance(motors, DynamixelMotors), "Failed to create a DynamixelMotors instance."
    assert len(motors._motor_configs) == 4, "Incorrect number of motor configurations created."
    assert motors._motor_configs[0].id == 0, "First motor ID mismatch."
    assert motors._motor_configs[1].id == 1, "Second motor ID mismatch."
    assert motors._motor_configs[0].model == "XM430-W210", "First motor model mismatch."
    assert motors._motor_configs[1].model == "L54-30-S400-R", "Second motor model mismatch."
    assert motors._motor_configs[0].pulley_radius == 20, "First motor pulley_radius mismatch."
    assert motors._motor_configs[1].pulley_radius == 20, "Second motor pulley_radius mismatch."
    assert motors._motor_configs[0].pulse_center == 2048, "First motor pulse_center mismatch."
    assert motors._motor_configs[1].pulse_center == 0, "Second motor pulse_center mismatch."
    assert motors._motor_configs[0].max_vel == 1000, "First motor max_vel mismatch."
    assert motors._motor_configs[1].max_vel == 500, "Second motor max_vel mismatch."


def test_connection(emio_motors):
    """Setup function to be called before each test."""

    emio_motors.open()

    assert len(listUnusedFTDIDevices()) == len(listFTDIDevices())-len(listUsedFTDIDevices()), "Unused devices found."

    emio_motors.close()


def test_main(emio_motors):
    emio_motors.open()
    assert emio_motors.is_connected, "Failed to connect to the motor group."

    initial_pos = [0] * 4
    emio_motors.angles = initial_pos
    time.sleep(1)
    logger.info(f"Initial position:")
    emio_motors.printStatus()

    emio_motors.max_velocity = [1000] * 4
    new_pos = [pi/8] * 4
    logger.info(new_pos)
    emio_motors.angles = new_pos
    time.sleep(1)
    emio_motors.printStatus()

    new_pos = [pi/2] * 4
    logger.info(new_pos)
    emio_motors.angles = new_pos
    time.sleep(1)
    emio_motors.printStatus()

    emio_motors.angles = initial_pos
    time.sleep(1)
    emio_motors.printStatus()

    assert all(abs(i-m) < 0.01 for i, m in zip(emio_motors.angles, initial_pos)) , "Motor did not return to initial position."

    emio_motors.close()


def test_multiturn(emio_motors):
    emio_motors.open(multi_turn=True)
    assert emio_motors.is_connected, "Failed to connect to the motor group."

    initial_pos = [0] * 4
    emio_motors.angles = initial_pos
    time.sleep(1)
    logger.info(f"Initial position: ")
    emio_motors.printStatus()

    emio_motors.max_velocity = [1000] * 4
    new_pos = [2*pi] * 4
    logger.info(new_pos)
    emio_motors.angles = new_pos
    time.sleep(1)
    emio_motors.printStatus()

    emio_motors.angles = initial_pos
    time.sleep(1)
    emio_motors.printStatus()

    assert all(abs(i-m) < 1 for i, m in zip(emio_motors.angles, initial_pos)) , "Motor did not return to initial position."

    emio_motors.close()