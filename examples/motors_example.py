#!/usr/bin/env -S uv run --script

import time
import logging
import os
import sys
from math import pi

sys.path.append(os.path.dirname(os.path.realpath(__file__))+'/..')
from dynamixelmotorsapi import DynamixelMotors
from dynamixelmotorsapi._dynamixelmotorsparameters import DXL_IDs
from dynamixelmotorsapi._logging_config import logger


class MyDynamixelMotors(DynamixelMotors):
    _length_to_rad = 1.0 / 20.0  # 1/radius of the pulley
    _rad_to_pulse = 4096 / (2 * pi)  # the resolution of the Dynamixel xm430 w210
    _pulse_center= 2048
    _max_vel = 1000  # *0.01 rev/min

    def __init__(self):
        super().__init__() # Check if all parameters have been set


def main(robot_motors: DynamixelMotors, loops=1):

    initial_pos_pulse = [0] * len(DXL_IDs)
    robot_motors.max_velocity = [1000] * len(DXL_IDs)
    logger.info(f"Initial position in rad: {initial_pos_pulse}")
    robot_motors.angles = initial_pos_pulse
    time.sleep(1)
    robot_motors.printStatus()


    for i in range(loops):
        new_pos = [((2*3.14)*((i+1)%8)/8)] * len(DXL_IDs)
        print("-"*20)
        logger.info(f"new_pos {new_pos}")
        try:
            if robot_motors.is_connected:
                robot_motors.angles = new_pos
                time.sleep(1)
                robot_motors.printStatus()
            else:
                robot_motors.open()
        except Exception as e:
            logger.error(f"Error during communication: {e}")
            robot_motors.close()
            robot_motors.open()


if __name__ == "__main__":
    robot_motors=None
    try:
        logger.info("Starting DynamixelMotors API test...")
        logger.info("Opening and configuring Robot Motors API...")
        
        robot_motors = MyDynamixelMotors()
        
        if robot_motors.open(): 
            
            robot_motors.printStatus()

            logger.info("Robot motors opened and configured.")
            logger.info("Running main function...")
            main(robot_motors, 15)

            logger.info("Main function completed.")
            logger.info("Closing Robot motors connection...")

            robot_motors.close()
            logger.info("Robot Motors connection closed.")
    except Exception as e:
        logger.exception(f"An error occurred: {e}")
        if robot_motors:
            robot_motors.close()