from threading import Lock
from math import pi
from typing import List
import json

import dynamixelmotorsapi._motorgroup as motorgroup
from dynamixelmotorsapi._dynamixelmotorsconfigs import MotorConfig
from dynamixelmotorsapi._logging_config import logger

from dynamixelmotorsapi._dynamixelmotorsconfigs import register_model_from_json, MODELS_CONFIGS
try:
    loaded_models = register_model_from_json("./dynamixelmotorsapi/dynamixel_configs.json", overwrite=True)
    if len(loaded_models):
        logger.info(f"Loaded  {len(MODELS_CONFIGS)}  motor configs")
except Exception as e:
    logger.error(f"Failed to load motor configs from JSON: {e}")


class DynamixelMotors:
    """
    Abstract class to control Dynamixel motors, supporting heterogeneous motor groups
    where each motor can be a different series with different conversion parameters.

    The motor group is configured via a list of MotorConfig objects, one per motor,
    which can be loaded from a dict or JSON file.

    Example:
        ```python
        from dynamixelmotorsapi import DynamixelMotors

        motors = DynamixelMotors.from_json("my_motors.json")

        if motors.open():
            print("Current angles (rad):", motors.angles)
            motors.angles = [0.5, 1.0, -0.5, 1.0]
            motors.printStatus()
            motors.close()
        else:
            print("Failed to connect to motors.")
        ```

    JSON format examples:
        ```json
        [
            {
                "id": 0,
                "model": "XM430-W210",
                "pulley_radius": 20,  # radius of the pulley in mm
                "pulse_center": 2048,
                "max_vel": 1000,
                "baud_rate": 57600
            },
            {
                "id": 1,
                "model": "P_SERIES",
                "pulley_radius": 30,  # radius of the pulley in mm
                "pulse_center": 0,
                "max_vel": 500,
                "baud_rate": 57600
            }
        ]
        ```

        ```json
        {
            "id": [0, 1],
            "model": ["XM430-W210", "P_SERIES"],
            "pulley_radius": [20, 30],
            "pulse_center": [2048, 0],
            "max_vel": [1000, 500],
            "baud_rate": 57600
        }
        ```
    """
    _FTDI_list = {}
    _initialized: bool = False

    _motor_configs: List[MotorConfig] = None
    _goal_velocity: list = None  # store the last commanded velocity in rev/min, per motor
    _goal_position: list = None  # store the last commanded position in radians
    _mg: motorgroup.MotorGroup = None
    _device_index: int = None


    #####################
    ###### METHODS ######
    #####################


    def __init__(self, motor_configs: List[MotorConfig]|List[dict]):
        """
        Args:
            motor_configs: list of MotorConfig, one per motor, ordered by motor index.
        """
        self._lock = Lock()
        
        # Normalize: if dicts were passed, unwrap and convert to MotorConfig
        if motor_configs and isinstance(motor_configs[0], dict):
            dicts = []
            for d in motor_configs:
                dicts.extend(self.__unwrap_dict(d))
            self._motor_configs = [MotorConfig.from_dict(m) for m in dicts]
        else:
            self._motor_configs = motor_configs
        
        n = len(motor_configs)
        self._goal_velocity = [0] * n
        self._goal_position = [0] * n

        if not self._initialized:
            self._mg = motorgroup.MotorGroup(motor_configs)
            self._initialized = True

    @staticmethod
    def listMotorsModels() -> list:
        """List the models of Dynamixel motors supported by this API."""
        return list(MODELS_CONFIGS.keys())


    @classmethod
    def __unwrap_dict(cls, data: dict) -> List[MotorConfig]:
        """Helper function to convert a dict of motor configs into a list of dicts, one per motor."""
        if isinstance(data["id"], list):
            motors_count = len(data["id"])
            for key, value in data.items():
                if isinstance(value, list) and len(value) != motors_count:
                    raise ValueError(f"All list values in the motor config dict must have the same length. Key '{key}' has length {len(value)}, expected {motors_count}.")
                elif not isinstance(value, list):
                    data[key] = [value] * motors_count
            return [dict(zip(data.keys(), values)) for values in zip(*data.values())]  # zip the dict of lists into a list of dicts, one per motor
        else :
            return [data]  # single motor case
        

    @classmethod
    def from_dicts(cls, data: list) -> "DynamixelMotors":
        """
        Instantiate from a list of per-motor config dicts.

        Args:
            data: list of dicts, each containing the fields for one MotorConfig.

        Returns:
            A configured DynamixelMotors instance (not yet connected).

        Example:
        ```json
        [
            {
                "id": 0,
                "model": "XM430-W210",
                "pulley_radius": 20,  # radius of the pulley in mm
                "pulse_center": 2048,
                "max_vel": 1000,
                "baud_rate": 57600
            },
            {
                "id": [1, 2],
                "model": ["XM430-W210", "P_SERIES"],
                "pulley_radius": [0.05, 0.03],
                "pulse_center": [0, 0],
                "max_vel": [1000, 500],
                "baud_rate": 57600
            }
        ]
        ```
        """
        return cls(data)
    
    
    @classmethod
    def from_dict(cls, data: dict) -> "DynamixelMotors":
        """
        Instantiate from a list of per-motor config dicts.

        Args:
            data: dict of lists, each list containing the fields for the MotorConfig.

        Returns:
            A configured DynamixelMotors instance (not yet connected).

        Example:
        ```json
         {
            "id": 0,
            "model": "XM430-W210",
            "pulley_radius": 20, # radius of the pulley in mm
            "pulse_center": 2048,
            "max_vel": 1000,
            "baud_rate": 57600
        }

        OR

        {
            "id": [0, 1],
            "model": ["XM430-W210", "P_SERIES"],
            "pulley_radius": [20, 30],
            "pulse_center": [2048, 0],
            "max_vel": [1000, 500],
            "baud_rate": 57600
        }
        ```
        """
        return cls([data])


    @classmethod
    def from_json(cls, path: str) -> "DynamixelMotors":
        """
        Instantiate from a JSON file containing a list of per-motor config dicts.

        Args:
            path: path to the JSON file.

        Returns:
            A configured DynamixelMotors instance (not yet connected).
        """
        with open(path) as f:
            json = json.load(f)
            if isinstance(json, list):
                return cls.from_dicts(json)
            elif isinstance(json, dict):
                return cls.from_dict(json)


    def _config(self, index: int) -> MotorConfig:
        """Convenience accessor for the MotorConfig of motor at position `index`."""
        return self._motor_configs[index]


    def lengthToPulse(self, displacement: list) -> list:
        """
        Convert length (mm) to pulse, per motor.

        Args:
            displacement: list of length values in mm, one per motor.

        Returns:
            A list of pulse values for each motor.
        """
        return [
            cfg.pulse_center - int(d * cfg.length_to_pulse)
            for d, cfg in zip(displacement, self._motor_configs)
        ]


    def pulseToLength(self, pulse: list) -> list:
        """
        Convert pulse to length (mm), per motor.

        Args:
            pulse: list of pulse integer values, one per motor.

        Returns:
            A list of length values in mm for each motor.
        """
        return [
            (cfg.pulse_center - float(p)) / cfg.length_to_pulse
            for p, cfg in zip(pulse, self._motor_configs)
        ]


    def pulseToRad(self, pulse: list) -> list:
        """
        Convert pulse to radians, per motor.

        Args:
            pulse: list of pulse integer values, one per motor.

        Returns:
            A list of angles in radians for each motor.
        """
        return [
            (cfg.pulse_center - float(p)) / cfg.rad_to_pulse
            for p, cfg in zip(pulse, self._motor_configs)
        ]


    def pulseToDeg(self, pulse: list) -> list:
        """
        Convert pulse to degrees, per motor.

        Args:
            pulse: list of pulse values, one per motor.

        Returns:
            A list of angles in degrees for each motor.
        """
        return [
            (cfg.pulse_center - float(p)) / cfg.rad_to_pulse * 180.0 / pi
            for p, cfg in zip(pulse, self._motor_configs)
        ]


    def _openAndConfig(self, device_name: str = None, multi_turn: bool = False) -> bool:
        """Open the connection to the motors, configure position mode and enable torque."""
        with self._lock:
            try:
                self._mg.updateDeviceName(device_name)

                if self._mg.deviceName is None:
                    logger.error("Device name is None. Please check the connection.")
                    return False

                self._mg.open()
                self._mg.disableTorque()
                self._mg.clearPort()
                if multi_turn:
                    self.enableExtendedPositionMode()
                else:
                    self.enablePositionMode()
                # set the max velocity profile to the configured max velocity for each motor
                self._mg.setVelocityProfile([cfg.max_vel for cfg in self._motor_configs])
                self._mg.enableTorque()

                DynamixelMotors._FTDI_list[self._mg.deviceName] = self  # add the connected device to the list of used devices

                logger.debug(
                    f"Motor group opened and configured. "
                    f"Device name: {self._mg.deviceName}, Multi turn: {multi_turn}"
                )
                return True
            except Exception as e:
                logger.error(f"Failed to open and configure the motor group: {e}")
                return False


    def open(self, device_name: str = None, multi_turn: bool = False) -> bool:
        """
        Open the connection to the motors.

        Args:
            device_name: if set, connect to this specific port; otherwise use the first available.
            multi_turn: enable multi-turn mode. Angle interval becomes [-256*2π, 256*2π].
        """
        if self._openAndConfig(device_name, multi_turn):
            self._device_index = motorgroup.listMotors().index(self._mg.deviceName)
            logger.info(f"Connected to device: {self._mg.deviceName}")
            return True
        return False


    def findAndOpen(self, device_name: str = None, multi_turn: bool = False) -> int:
        """
        Iterate over serial ports and connect to the first available FTDI device.

        Args:
            device_name: if set, try only this port.
            multi_turn: enable multi-turn mode.

        Returns:
            Index of the connected port, or -1 if no connection was possible.
        """
        if device_name is not None:
            try:
                index = motorgroup.listMotors().index(device_name)
                logger.info(f"Trying given device number {index} on port: {device_name}.")
                return index if len(motorgroup.listMotors()) > 0 and self.open(device_name, multi_turn) else -1
            except Exception:
                return -1

        index = 0
        connected = False
        try:
            while not connected and index < len(motorgroup.listMotors()):
                device_name = motorgroup.listMotors()[index]
                logger.info(f"Trying device number {index} on port: {device_name}.")
                connected = self.open(device_name, multi_turn)
                if connected:
                    self._device_index = index
                    return self.device_index
                index += 1
        except Exception:
            return -1
        return -1


    def close(self):
        """Close the connection to the motors."""
        with self._lock:
            try:
                self._mg.close()
                DynamixelMotors._FTDI_list.pop(self._mg.deviceName, None)
                logger.info("Motors connection closed.")
            except Exception as e:
                logger.error(e)


    def printStatus(self):
        """Print the current position of the motors in radians, pulses, and degrees."""
        with self._lock:
            current_pos = self._mg.getCurrentPosition()
            rads    = self.pulseToRad(current_pos)
            degrees = self.pulseToDeg(current_pos)
            logger.info(
                f"Current position of the motors\n"
                f"  radians: {[round(a, 4) for a in rads]}\n"
                f"  pulses:  {list(current_pos)}\n"
                f"  degrees: {[round(a, 2) for a in degrees]}"
            )

    
    def printConfig(self):
        """Print the current configuration of the motors."""
        for i, cfg in enumerate(self._motor_configs):
            logger.info(
                f"Motor {i} config:\n"
                f"  ID: {cfg.id}\n"
                f"  Model: {cfg.model}\n"
                f"  Length to rad conversion: {cfg.length_to_rad}\n"
                f"  Pulse center: {cfg.pulse_center}\n"
                f"  Max velocity: {cfg.max_vel}"
            )


    def enablePositionMode(self):
        self._mg.enablePositionMode()


    def enableExtendedPositionMode(self):
        self._mg.enableExtendedPositionMode()


    ####################
    #### PROPERTIES ####
    ####################


    #### Read and Write properties ####

    @property
    def torque(self) -> list:
        """Get the current torque status of the motors."""
        with self._lock:
            return self._mg.isTorqueEnable()
        
    @torque.setter
    def torque(self, enable: bool):
        """Enable or disable torque for the motors."""
        with self._lock:
            if enable:
                self._mg.enableTorque()
            else:
                self._mg.disableTorque()

                
    @property
    def angles(self) -> list:
        """Get the current angles of the motors in radians."""
        with self._lock:
            return self.pulseToRad(self._mg.getCurrentPosition())

    @angles.setter
    def angles(self, angles: list):
        """Set the goal angles of the motors in radians."""
        with self._lock:
            self._goal_position = angles
            self._mg.setGoalPosition([
                int(cfg.pulse_center - cfg.rad_to_pulse * a)
                for a, cfg in zip(angles, self._motor_configs)
            ])

    
    @property
    def positions(self) -> list:
        """Get the current positions of the motors in pulses."""
        with self._lock:
            return self._mg.getCurrentPosition()
        
    @positions.setter
    def positions(self, positions: list):
        """Set the goal positions of the motors in pulses."""
        with self._lock:
            self._goal_position = self.pulseToRad(positions)
            self._mg.setGoalPosition(positions)


    @property
    def goal_velocity(self) -> list:
        """Get the last commanded velocity (rev/min) for each motor."""
        return self._goal_velocity

    @goal_velocity.setter
    def goal_velocity(self, velocities: list):
        """Set the goal velocity (rev/min) for each motor."""
        self._goal_velocity = velocities
        with self._lock:
            self._mg.setGoalVelocity(velocities)


    @property
    def max_velocity(self) -> list:
        """Get the maximum velocity profile (rev/min) for each motor."""
        return [cfg.max_vel for cfg in self._motor_configs]

    @max_velocity.setter
    def max_velocity(self, max_vel: list):
        """
        Set the maximum velocity profile (rev/min) in position mode, per motor.

        Args:
            max_vel: list of maximum velocities for each motor in rev/min.
        """
        for cfg, v in zip(self._motor_configs, max_vel):
            cfg.max_vel = v
        with self._lock:
            self._mg.setVelocityProfile(max_vel)


    @property
    def position_p_gain(self) -> list:
        """Get the current position P gains of the motors."""
        with self._lock:
            return self._mg.getPositionPGain()

    @position_p_gain.setter
    def position_p_gain(self, p_gains: list):
        """Set the position P gains of the motors."""
        with self._lock:
            self._mg.setPositionPGain(p_gains)


    @property
    def position_i_gain(self) -> list:
        """Get the current position I gains of the motors."""
        with self._lock:
            return self._mg.getPositionIGain()

    @position_i_gain.setter
    def position_i_gain(self, i_gains: list):
        """Set the position I gains of the motors."""
        with self._lock:
            self._mg.setPositionIGain(i_gains)


    @property
    def position_d_gain(self) -> list:
        """Get the current position D gains of the motors."""
        with self._lock:
            return self._mg.getPositionDGain()

    @position_d_gain.setter
    def position_d_gain(self, d_gains: list):
        """Set the position D gains of the motors."""
        with self._lock:
            self._mg.setPositionDGain(d_gains)

    @property
    def velocity_profile(self) -> list:
        """Get the velocity profile (rev/min) of the motors."""
        with self._lock:
            return self._mg.getVelocityProfile()
        
    @velocity_profile.setter
    def velocity_profile(self, profile: list):
        """Set the velocity profile (rev/min) of the motors."""
        with self._lock:
            self._mg.setVelocityProfile(profile)

    @property
    def currents(self) -> list:
        """Get the current (mA) of the motors."""
        with self._lock:
            raw_currents = self._mg.getPresentCurrent()
            currents = []
            for raw, cfg in zip(raw_currents, self._motor_configs):
                if cfg.current_unit is not None:
                    currents.append(raw * cfg.current_unit)
                else:
                    currents.append(raw)
            return currents
        
    @currents.setter
    def currents(self, currents: list):
        """Set the current (mA) of the motors."""
        with self._lock:
            self._mg.setPresentCurrent(currents)


    #### Read-only properties ####

    @property
    def motor_configs(self) -> List[MotorConfig]:
        """Get the list of per-motor configurations."""
        return self._motor_configs

    @property
    def is_connected(self) -> bool:
        """Check if the motors are connected."""
        with self._lock:
            return self._mg.isConnected

    @property
    def device_name(self) -> str:
        """Get the name of the connected device port."""
        with self._lock:
            return self._mg.deviceName

    @property
    def device_index(self) -> int:
        """Get the index of the device in the list of available motor devices."""
        return self._device_index

    @property
    def moving(self) -> list:
        """Check if the motors are moving."""
        with self._lock:
            return self._mg.isMoving()

    @property
    def moving_status(self) -> list:
        """Get the moving status byte of the motors.

        See https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#moving-status for details.
        """
        with self._lock:
            return self._mg.getMovingStatus()

    @property
    def velocity(self) -> list:
        """Get the current velocity (rev/min) of the motors."""
        with self._lock:
            return self._mg.getCurrentVelocity()

    @property
    def velocity_trajectory(self) -> list:
        """Get the velocity (rev/min) trajectory of the motors."""
        with self._lock:
            return self._mg.getVelocityTrajectory()

    @property
    def position_trajectory(self) -> list:
        """Get the position (pulse) trajectory of the motors."""
        with self._lock:
            return self._mg.getPositionTrajectory()
    