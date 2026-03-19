#!/usr/bin/env -S uv run --script

import time
import logging
import os
import sys
from math import pi

sys.path.append(os.path.dirname(os.path.realpath(__file__))+'/..')
from dynamixelmotorsapi import DynamixelMotors
from dynamixelmotorsapi._logging_config import logger


def main(robot_motors: DynamixelMotors, loops=1):
    motors_count = robot_motors._motor_configs
    initial_pos_pulse = [0] * len(motors_count)
    robot_motors.max_velocity = [1000] * len(motors_count)
    logger.info(f"Initial position in rad: {initial_pos_pulse}")
    robot_motors.angles = initial_pos_pulse
    time.sleep(1)
    robot_motors.printStatus()


    for i in range(loops):
        new_pos = [((2*3.14)*((i+1)%8)/8)] * len(motors_count)
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

        motors_description = [
            {
                "id": 0,
                "model": "XM430-W210",
                "pulley_radius": 20,  # radius of the pulley in mm
                "pulse_center": 2048,
                "max_vel": 1000,
                "baud_rate": 1000000
            },
            {
                "id": [1, 2, 3],
                "model": ["XM430-W210"]*3,
                "pulley_radius": [20]*3, # radius of the pulley in mm
                "pulse_center": [2048]*3,
                "max_vel": [1000]*3,
                "baud_rate": [4000000]*3
            }
        ]
        
        robot_motors = DynamixelMotors.from_dicts(motors_description)
        
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