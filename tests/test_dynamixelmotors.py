import time
import logging
import pytest
import os
import sys

sys.path.append(os.path.dirname(os.path.realpath(__file__))+'/..')

from dynamixelmotorsapi import DynamixelMotors
from dynamixelmotorsapi import listFTDIDevices, listUnusedFTDIDevices, listUsedFTDIDevices


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


config = [
            {
                "id": 0,
                "model": "XM430-W210",
                "length_to_rad": 0.05,
                "pulse_center": 2048,
                "max_vel": 1000
            },
            {
                "id": [1, 2],
                "model": ["L54-30-S400-R", "P_SERIES"],
                "length_to_rad": [0.05, 0.03],
                "pulse_center": [0, 0],
                "max_vel": [800, 500]
            }
        ]
print("CONFIG ", config)
motors = DynamixelMotors.from_dicts(config)


def test_creation():
    """Test the creation of a DynamixelMotors instance."""
    
    assert isinstance(motors, DynamixelMotors), "Failed to create a DynamixelMotors instance."
    assert len(motors._motor_configs) == 3, "Incorrect number of motor configurations created."
    assert motors._motor_configs[0].id == 0, "First motor ID mismatch."
    assert motors._motor_configs[1].id == 1, "Second motor ID mismatch."
    assert motors._motor_configs[0].model == "XM430-W210", "First motor model mismatch."
    assert motors._motor_configs[1].model == "L54-30-S400-R", "Second motor model mismatch."
    assert motors._motor_configs[0].length_to_rad == 0.05, "First motor length_to_rad mismatch."
    assert motors._motor_configs[1].length_to_rad == 0.05, "Second motor length_to_rad mismatch."
    assert motors._motor_configs[0].pulse_center == 2048, "First motor pulse_center mismatch."
    assert motors._motor_configs[1].pulse_center == 0, "Second motor pulse_center mismatch."
    assert motors._motor_configs[0].max_vel == 1000, "First motor max_vel mismatch."
    assert motors._motor_configs[1].max_vel == 800, "Second motor max_vel mismatch."


def test_connection():
    """Setup function to be called before each test."""

    motors.open()

    assert listUnusedFTDIDevices() == listFTDIDevices()-listUsedFTDIDevices(), "Unused devices found."


def test_main(setupBefore):

    initial_pos_pulse = [0] * 4
    logger.info(f"Initial position in pulses: {initial_pos_pulse}")
    motors.printStatus()

    motors.max_velocity = [1000] * 4
    time.sleep(1)
    new_pos = [3.14/8] * 4
    logger.info(new_pos)
    motors.angles = new_pos

    motors.printStatus()
    time.sleep(1)
    motors.printStatus()
    new_pos = [3.14/2] * 4
    logger.info(new_pos)
    motors.angles = new_pos
    logging.info(motors.moving)
    time.sleep(1)
    motors.printStatus()
    motors.angles = initial_pos_pulse
    time.sleep(1)
    motors.printStatus()
    assert (motors.angles == motors.angles) , "Motor did not return to initial position."


if __name__ == "__main__":
    try:
        pytest.main()
    except Exception as e:
        logger.error(f"An error occurred: {e}")
    finally:
        motors.close()
