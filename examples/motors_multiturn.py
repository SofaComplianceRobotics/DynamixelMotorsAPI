#!/usr/bin/env -S uv run --script

import time
import logging
import os
import sys
import locale
from math import pi

sys.path.append(os.path.dirname(os.path.realpath(__file__))+'/..')
from dynamixelmotorsapi import DynamixelMotors
from dynamixelmotorsapi._logging_config import logger

class MyDynamixelMotors(DynamixelMotors):
    _length_to_rad = 1.0 / 20.0  # 1/radius of the pulley
    _rad_to_pulse = 4096 / (2 * pi)  # the resolution of the Dynamixel xm430 w210
    _pulse_center= 2048
    _max_vel = 1000  # *0.01 rev/min

    def __init__(self):
        super().__init__() # Check if all parameters have been set


def main(robot_motors: DynamixelMotors, loops=1):

    initial_pos_pulse = [0] * 4
    robot_motors.max_velocity = [1000] * 4
    logger.info(f"Initial position in rad: {initial_pos_pulse}")
    robot_motors.angles = initial_pos_pulse
    time.sleep(1)
    robot_motors.printStatus()

    angle_command = ""
    while True:
        angle_command = input("Enter an angle for the motor [-256*360, 256*360]: ")
        try:
            if angle_command == "quit":
                break
            new_angle = locale.atof(angle_command)
            new_angle = new_angle*pi/180
            new_pos = [new_angle]*4
            print("-"*20)
            logger.info(f"new_pos {new_pos}")
            if robot_motors.is_connected:
                robot_motors.angles = new_pos
                time.sleep(1)
                robot_motors.printStatus()
            else:
                print("Reconnecting")
                robot_motors.open(multi_turn=True)
        except Exception as e:
            logger.error(f"Error during communication: {e}")
    robot_motors.close()


if __name__ == "__main__":
    try:
        logger.info("Starting DynamixelMotors API test...")
        logger.info("Opening and configuring DynamixelMotors API...")
        
        robot_motors = MyDynamixelMotors()
        
        if robot_motors.open(multi_turn=True): 
            
            robot_motors.printStatus()

            logger.info("Robot Motors motors opened and configured.")
            logger.info("Running main function...")
            main(robot_motors, 15)

            logger.info("Main function completed.")
            logger.info("Closing Robot Motors motor connection...")

            robot_motors.close()
            logger.info("Robot Motors connection closed.")
    except Exception as e:
        logger.exception(f"An error occurred: {e}")
        robot_motors.close()