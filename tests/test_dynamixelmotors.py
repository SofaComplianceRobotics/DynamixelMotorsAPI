import time
import logging
import pytest
from dynamixelmotorsapi import *


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


motors = DynamixelMotors()

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
