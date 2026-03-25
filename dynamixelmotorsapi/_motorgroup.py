import ctypes
from threading import Lock
from math import pi
from typing import List, Dict
from itertools import groupby

import serial.tools.list_ports as list_ports

from dynamixel_sdk import *

from dynamixelmotorsapi._dynamixelmotorsconfigs import (
    MotorConfig, ModelConfig, PROTOCOL_VERSION, TORQUE_ENABLE, TORQUE_DISABLE, 
    VELOCITY_MODE, POSITION_MODE, EXT_POSITION_MODE, PWM_MODE, CURRENT_MODE
)
from dynamixelmotorsapi._logging_config import logger


def listMotors():
    """
    List all the FTDI devices connected to the computer.

    Returns:
        A list containing the devices name (string)
    """
    ports = []
    comports = serial.tools.list_ports.comports()

    for p in comports:
        if p.manufacturer is not None and "FTDI" in p.manufacturer:
            ports.append(p.device)
        elif p.description is not None and "FTDI" in p.description:
            ports.append(p.device)
        elif p.serial_number is not None and "FTDI" in p.serial_number:
            ports.append(p.device)

    if not ports:
        logger.warning("No motor found. Please check the connection.")

    return ports


def getDevicePort(entry, method="manufacturer"):
    """
    Get the device port based on the device name and method.

    Args:
        entry (str): The name of the device to search for.
        method (str): The method to use for searching (default is "manufacturer").

    Returns:
        The first port of the device if found, otherwise None.
    """
    comports = serial.tools.list_ports.comports()

    if not comports:
        logger.error("Serial ports check failed, list of ports is empty.")
        return None

    if method == "manufacturer":
        ports = [p for p in comports if p.manufacturer is not None and entry in p.manufacturer]
    elif method == "description":
        ports = [p for p in comports if p.description is not None and entry in p.description]
    elif method == "serial_number":
        ports = [p for p in comports if p.serial_number is not None and entry in p.serial_number]
    else:
        ports = []

    if not ports:
        logger.error(f"No serial port found with {method} = {entry}")
        return None

    if len(ports) > 1:
        logger.warning(f"Multiple ports found with {method} = {entry}. Using the first.")

    p = ports[0]
    logger.debug(
        f"Found port with {method} = {entry}:\n"
        f"  device:        {p.device}\n"
        f"  manufacturer:  {p.manufacturer}\n"
        f"  description:   {p.description}\n"
        f"  serial number: {p.serial_number}"
    )
    return p.device


def _valToArray(val):
    """Convert a 32-bit integer to a list of 4 bytes."""
    return [DXL_LOBYTE(DXL_LOWORD(val)), DXL_HIBYTE(DXL_LOWORD(val)),
            DXL_LOBYTE(DXL_HIWORD(val)), DXL_HIBYTE(DXL_HIWORD(val))]


def _valTo2Bytes(val):
    """Convert a 16-bit integer to a list of 2 bytes."""
    return [DXL_LOBYTE(val), DXL_HIBYTE(val)]


class DisconnectedException(Exception):
    """Raised when an operation is attempted on a disconnected motor group."""
    def __init__(self):
        super().__init__(
            "MotorGroup is not connected. It is either disconnected or permission denied."
        )


class MotorGroup:
    """
    Controls a group of Dynamixel motors, supporting heterogeneous models.

    Motors are grouped internally by models so that GroupSyncRead/Write —
    which require a uniform address and data length — operate correctly even
    when motors from different models (with different register maps) are mixed.

    All public read methods return values ordered by the original motor_configs
    list, regardless of internal grouping.
    """

    def __init__(self, motor_configs: List[MotorConfig]) -> None:
        self.motorsConfig = motor_configs
        self.deviceName = None

        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.portHandler = PortHandler(self.deviceName)

        # Group motors by models so each GroupSyncRead/Write uses a consistent
        # address and length. Dict key is model name, value is list of MotorConfigs.
        self._models_groups: Dict[str, List[MotorConfig]] = {}
        for cfg in motor_configs:
            self._models_groups.setdefault(cfg.model, []).append(cfg)

        # Per-models sync read/write groups: {model_name: {reader_name: GroupSyncRead}}
        self.groupReaders: Dict[str, Dict[str, GroupSyncRead]] = {}
        self.groupWriters: Dict[str, Dict[str, GroupSyncWrite]] = {}

        self._initGroups()

    # ------------------------------------------------------------------ #
    #  Initialisation helpers                                              #
    # ------------------------------------------------------------------ #

    def _initGroups(self):
        """Create GroupSyncRead/Write objects for each models."""
        for model_name, configs in self._models_groups.items():
            sc: ModelConfig = configs[0].model_config  # all share the same ModelConfig

            readers = {
                "position": GroupSyncRead(
                                self.portHandler, self.packetHandler,
                                sc.addr_present_position, sc.len_present_position),
                "velocity": GroupSyncRead(
                                self.portHandler, self.packetHandler,
                                sc.addr_present_velocity, sc.len_present_velocity),
                "goal_position": GroupSyncRead(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_goal_position, sc.len_goal_position),
                "goal_velocity": GroupSyncRead(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_goal_velocity, sc.len_goal_velocity),
                "velocity_profile": GroupSyncRead(
                                        self.portHandler, self.packetHandler,
                                        sc.addr_velocity_profile, sc.len_goal_velocity),
                "moving": GroupSyncRead(
                            self.portHandler, self.packetHandler,
                            sc.addr_moving, 1),
                "moving_status": GroupSyncRead(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_moving_status, 1),
                "velocity_trajectory": GroupSyncRead(
                                        self.portHandler, self.packetHandler,
                                        sc.addr_velocity_trajectory, sc.len_velocity_trajectory),
                "position_trajectory": GroupSyncRead(
                                        self.portHandler, self.packetHandler,
                                        sc.addr_position_trajectory, sc.len_position_trajectory),
                "position_p_gain": GroupSyncRead(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_position_p_gain, sc.len_position_p_gain),
                "position_i_gain": GroupSyncRead(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_position_i_gain, sc.len_position_i_gain),
                "position_d_gain": GroupSyncRead(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_position_d_gain, sc.len_position_d_gain),
                "present_current": GroupSyncRead(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_present_current, sc.len_present_current),
                "pwm": GroupSyncRead(self.portHandler, self.packetHandler,
                        sc.addr_present_pwm,
                        sc.len_present_pwm),
                "pwm_limit": GroupSyncRead(self.portHandler, self.packetHandler,
                                sc.addr_pwm_limit,
                                sc.len_pwm_limit),
                "goal_pwm": GroupSyncRead(self.portHandler, self.packetHandler,
                                sc.addr_goal_pwm,
                                sc.len_goal_pwm)
            }

            writers = {
                "goal_position": GroupSyncWrite(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_goal_position, sc.len_goal_position),
                "goal_velocity": GroupSyncWrite(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_goal_velocity, sc.len_goal_velocity),
                "velocity_profile": GroupSyncWrite(
                                        self.portHandler, self.packetHandler,
                                        sc.addr_velocity_profile, sc.len_goal_velocity),
                "position_p_gain": GroupSyncWrite(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_position_p_gain, sc.len_position_p_gain),
                "position_i_gain": GroupSyncWrite(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_position_i_gain, sc.len_position_i_gain),
                "position_d_gain": GroupSyncWrite(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_position_d_gain, sc.len_position_d_gain),
                "present_current": GroupSyncWrite(
                                    self.portHandler, self.packetHandler,
                                    sc.addr_present_current, sc.len_present_current),
                "pwm": GroupSyncWrite(self.portHandler, self.packetHandler,
                        sc.addr_present_pwm,
                        sc.len_present_pwm),
                "pwm_limit": GroupSyncWrite(self.portHandler, self.packetHandler,
                                sc.addr_pwm_limit,
                                sc.len_pwm_limit),
                "goal_pwm": GroupSyncWrite(self.portHandler, self.packetHandler,
                                sc.addr_goal_pwm,
                                sc.len_goal_pwm)
            }

            # Register all motor IDs in this model with every reader
            for cfg in configs:
                for reader in readers.values():
                    reader.addParam(cfg.id)

            self.groupReaders[model_name] = readers
            self.groupWriters[model_name] = writers
            

    def _updateGroups(self):
        """Propagate the new portHandler to all sync groups after reconnection."""
        for readers in self.groupReaders.values():
            for group in readers.values():
                group.port = self.portHandler
                group.ph = self.packetHandler
        for writers in self.groupWriters.values():
            for group in writers.values():
                group.port = self.portHandler
                group.ph = self.packetHandler

    # ------------------------------------------------------------------ #
    #  Connection management                                               #
    # ------------------------------------------------------------------ #

    @property
    def isConnected(self):
        """Check if the motor group is connected."""
        try:
            return bool(self.portHandler and self.portHandler.is_open and self._isDeviceDetected())
        except Exception as e:
            logger.exception(f"Failed to check connection: {e}")
            return False
        

    def _isDeviceDetected(self):
        for port in serial.tools.list_ports.comports():
            if port.device == self.deviceName:
                return True
        return False
    

    def updateDeviceName(self, device_name: str = None):
        """
        Update the device name. If not provided, uses the first FTDI device found.
        """
        self.deviceName = device_name if device_name is not None else getDevicePort("FTDI")
        self.portHandler = PortHandler(self.deviceName)
        self._updateGroups()
        logger.debug(f"Device name updated to: {self.deviceName}")


    def openPort(self) -> None:
        """Open the port"""
        try:
            self.portHandler.openPort()
            logger.debug(f"Port opened")
        except Exception as e:
            raise Exception(f"Failed to open port: {e}")
        

    def close(self) -> None:
        """Disable torque on all motors and close the port."""
        try:
            for cfg in self.motorsConfig:
                self.packetHandler.write1ByteTxRx(
                    self.portHandler, cfg.id,
                    cfg.model_config.addr_torque_enable, TORQUE_DISABLE
                )
            self.portHandler.closePort()
            self.deviceName = None
        
            logger.debug(f"Closed port and disabled torque")
        except Exception as e:
            raise Exception(f"Failed to close port: {e}")
        

    def clearPort(self) -> None:
        """Clear the port buffer."""
        if not self.isConnected:
            raise DisconnectedException()
        if self.portHandler:
            self.portHandler.clearPort()
            logger.debug(f"Port cleared")

    # ------------------------------------------------------------------ #
    #  Torque and operating mode                                           #
    # ------------------------------------------------------------------ #

    def enableTorque(self):
        """Enable torque on all motors."""
        self._write1ByteAll(lambda cfg: cfg.model_config.addr_torque_enable, TORQUE_ENABLE)
        logger.debug(f"Torque enabled")


    def disableTorque(self):
        """Disable torque on all motors."""
        self._write1ByteAll(lambda cfg: cfg.model_config.addr_torque_enable, TORQUE_DISABLE)
        logger.debug(f"Torque disabled")


    def isTorqueEnable(self) -> list:
        """Return the torque enable state for each motor."""
        result = []
        for cfg in self.motorsConfig:
            self.portHandler.setBaudRate(cfg.baud_rate)
            torque, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(
                self.portHandler, cfg.id, cfg.model_config.addr_torque_enable
            )
            if dxl_comm_result != COMM_SUCCESS:
                raise Exception(f"Failed to read torque (motor {cfg.id}): "
                                f"{self.packetHandler.getTxRxResult(dxl_comm_result)}")
            if dxl_error != 0:
                raise Exception(f"Failed to read torque (motor {cfg.id}): "
                                f"{self.packetHandler.getRxPacketError(dxl_error)}")
            
            result.append(torque)
        return result
    

    def __setOperatingMode(self, mode: int):
        """
        Set the operating mode on all motors.

        Args:
            mode: 1=Velocity, 3=Position, 4=Extended Position.
                  See https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#operating-mode
        """
        for cfg in self.motorsConfig:
            self.portHandler.setBaudRate(cfg.baud_rate)
            torque, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(
                self.portHandler, cfg.id, cfg.model_config.addr_torque_enable
            )
        self._write1ByteAll(lambda cfg: cfg.model_config.addr_operating_mode, mode)


    def enableVelocityMode(self):
        torques = self.isTorqueEnable()
        if any(t == 1 for t in torques):
            self.disableTorque()
        self.__setOperatingMode(VELOCITY_MODE)
        if any(t == 1 for t in torques):
            self.enableTorque()
        logger.debug(f"Enabled Velocity Mode")


    def enablePositionMode(self):
        torques = self.isTorqueEnable()
        if any(t == 1 for t in torques):
            self.disableTorque()
        self.__setOperatingMode(POSITION_MODE)
        if any(t == 1 for t in torques):
            self.enableTorque()
        logger.debug(f"Enabled Position Mode")


    def enableExtendedPositionMode(self):
        torques = self.isTorqueEnable()
        if any(t == 1 for t in torques):
            self.disableTorque()
        self.__setOperatingMode(EXT_POSITION_MODE)
        if any(t == 1 for t in torques):
            self.enableTorque()
        logger.debug(f"Enabled Extended Position Mode")

    
    def enablePWMMode(self):
        torques = self.isTorqueEnable()

        if any(t==1 for t in torques):
            self.disableTorque()
        self.__setOperatingMode(PWM_MODE)
        if any(t==1 for t in torques):
            self.enableTorque()


    def enableCurrentMode(self):
        torques = self.isTorqueEnable()

        if any(t==1 for t in torques):
            self.disableTorque()
        self.__setOperatingMode(CURRENT_MODE)
        if any(t==1 for t in torques):
            self.enableTorque()


    def getBaudRate(self) -> list:
        """Get the baud rate for each motor."""
        result = []
        for cfg in self.motorsConfig:
            baud_rate, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(
                self.portHandler, cfg.id, cfg.model_config.addr_baud_rate
            )
            if dxl_comm_result != COMM_SUCCESS:
                raise Exception(f"Failed to read baud rate (motor {cfg.id}): "
                                f"{self.packetHandler.getTxRxResult(dxl_comm_result)}")
            if dxl_error != 0:
                raise Exception(f"Failed to read baud rate (motor {cfg.id}): "
                                f"{self.packetHandler.getRxPacketError(dxl_error)}")
            
            result.append(baud_rate)
        return result

    # ------------------------------------------------------------------ #
    #  Low-level read / write helpers                                      #
    # ------------------------------------------------------------------ #

    def _write1ByteAll(self, addr_fn, value: int):
        """
        Write a single byte to all motors, using each motor's own address.

        Args:
            addr_fn: callable(MotorConfig) -> int, returns the register address for that motor.
            value:   the byte value to write.
        """
        for cfg in self.motorsConfig:
            addr = addr_fn(cfg)
            
            self.portHandler.setBaudRate(cfg.baud_rate)

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, cfg.id, addr, value
            )
            if dxl_comm_result != COMM_SUCCESS:
                raise Exception(f"Failed to write byte {value} to motor ID {cfg.id} at addr {addr}: "
                                f"{self.packetHandler.getTxRxResult(dxl_comm_result)}")
            if dxl_error != 0:
                raise Exception(f"Error on motor {cfg.id} at addr {addr}: "
                                f"{self.packetHandler.getRxPacketError(dxl_error)}")
            
            logger.debug(f"Motor {cfg.id}: addr {addr} set to {value}")

    def _readSyncMotorsData(self, reader_name: str) -> list:
        """
        Read data from all motors using GroupSyncRead, per model, and return
        results in the original motor_configs order.

        Args:
            reader_name: key into self.groupReaders[model].

        Returns:
            List of values in motor_configs order.
        """
        # Collect results keyed by motor ID across all models
        results_by_id = {}
        for model_name, readers in self.groupReaders.items():
            
            self.portHandler.setBaudRate(self._models_groups[model_name][0].baud_rate)

            group = readers[reader_name]
            dxl_comm_result = group.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                raise Exception(
                    f"Failed to read '{reader_name}' for model '{model_name}': "
                    f"{self.packetHandler.getTxRxResult(dxl_comm_result)}"
                )
            
            for cfg in self._models_groups[model_name]:
                if not group.isAvailable(cfg.id, group.start_address, group.data_length):
                    raise Exception(
                        f"Data not available for motor {cfg.id} (model '{model_name}', "
                        f"reader '{reader_name}')"
                    )
                raw = group.getData(cfg.id, group.start_address, group.data_length)
                results_by_id[cfg.id] = ctypes.c_int(raw).value

        # Return in original motor_configs order
        return [results_by_id[cfg.id] for cfg in self.motorsConfig]
    

    def __writeSyncMotorsData(self, writer_name: str, values: list):
        """
        Write data to all motors using GroupSyncWrite, per models.

        Args:
            writer_name: key into self.groupWriters[model].
            values:      list of values in motor_configs order.
        """
        # Map motor ID -> value from the ordered values list
        values_by_id = {cfg.id: values[i] for i, cfg in enumerate(self.motorsConfig)}

        for model_name, writers in self.groupWriters.items(): # iterate through motor models in groupwriters
            self.portHandler.setBaudRate(self._models_groups[model_name][0].baud_rate)
            group = writers[writer_name]
            group.clearParam()
            for cfg in self._models_groups[model_name]: # get the MotorConfigs for model model_name
                val = values_by_id[cfg.id]
                if group.data_length == 2:
                    data = _valTo2Bytes(val)
                elif group.data_length == 4:
                    data = _valToArray(val)
                else:
                    raise Exception(f"Unsupported data length: {group.data_length}")
                group.addParam(cfg.id, data)
                

            dxl_comm_result = group.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                raise Exception(
                    f"Failed to read '{writer_name}' for model '{model_name}': "
                    f"{self.packetHandler.getTxRxResult(dxl_comm_result)}"
                )


    # ------------------------------------------------------------------ #
    #  Public read / write API                                             #
    # ------------------------------------------------------------------ #

    def setGoalPosition(self, positions: list):
        """Set the goal position (pulses) for each motor."""
        self.__writeSyncMotorsData("goal_position", positions)
        logger.debug(f"Goal Position set to {positions}")

    def setGoalVelocity(self, speeds: list):
        """Set the goal velocity for each motor."""
        self.__writeSyncMotorsData("goal_velocity", speeds)
        logger.debug(f"Goal Velocity set to {speeds}")

    def setGoalPWM(self, pwms: list):
        """Set the goal PWM for each motor.

        Args:
            pwms (list of numbers): unit depends on motor type
        """
        self.__writeSyncMotorsData(self.groupWriters["goal_pwm"] , pwms)

    def setPWMLimit(self, limits: list):
        """Set the PWM limit for each motor.

        Args:
            limits (list of numbers): unit depends on motor type
        """
        self.__writeSyncMotorsData(self.groupWriters["pwm_limit"] , limits)

    def setVelocityProfile(self, max_vel: list):
        """Set the maximum velocity profile (position mode) for each motor."""
        self.__writeSyncMotorsData("velocity_profile", max_vel)
        logger.debug(f"Velocity profile set to {max_vel}")

    def setPositionPGain(self, p_gains: list):
        """Set the position P gain for each motor."""
        self.__writeSyncMotorsData("position_p_gain", p_gains)
        logger.debug(f"Position P gains set to {p_gains}")

    def setPositionIGain(self, i_gains: list):
        """Set the position I gain for each motor."""
        self.__writeSyncMotorsData("position_i_gain", i_gains)
        logger.debug(f"Position I gains set to {i_gains}")

    def setPositionDGain(self, d_gains: list):
        """Set the position D gain for each motor."""
        self.__writeSyncMotorsData("position_d_gain", d_gains)
        logger.debug(f"Position D gains set to {d_gains}")

    def setPresentCurrent(self, currents: list):
        """Set the present current for each motor."""
        self.__writeSyncMotorsData("present_current", currents)
        logger.debug(f"Present Current set to {currents}")

    def getPresentPosition(self) -> list:
        """Get the current position (pulses) for each motor."""
        return self._readSyncMotorsData("position")

    def getGoalPosition(self) -> list:
        """Get the goal position (pulses) for each motor."""
        return self._readSyncMotorsData("goal_position")

    def getGoalVelocity(self) -> list:
        """Get the goal velocity for each motor."""
        return self._readSyncMotorsData("goal_velocity")
    
    def getGoalPWM(self) -> list:
        """Get the goal PWM for each motor."""
        return self._readSyncMotorsData("goal_pwm")
    
    def getPWMLimit(self) -> list:
        """Get the PWM limit for each motor."""
        return self._readSyncMotorsData("pwm_limit")

    def getPresentVelocity(self) -> list:
        """Get the current velocity for each motor."""
        return self._readSyncMotorsData("velocity")
    
    def getVelocityProfile(self) -> list:
        """Get the velocity profile (position mode) for each motor."""
        return self._readSyncMotorsData("velocity_profile")

    def isMoving(self) -> list:
        """Return True for each motor that is currently moving."""
        return self._readSyncMotorsData("moving")

    def getMovingStatus(self) -> list:
        """Get the moving status byte for each motor."""
        return self._readSyncMotorsData("moving_status")

    def getVelocityTrajectory(self) -> list:
        """Get the velocity trajectory for each motor."""
        return self._readSyncMotorsData("velocity_trajectory")

    def getPositionTrajectory(self) -> list:
        """Get the position trajectory for each motor."""
        return self._readSyncMotorsData("position_trajectory")

    def getPositionPGain(self) -> list:
        """Get the position P gain for each motor."""
        return self._readSyncMotorsData("position_p_gain")

    def getPositionIGain(self) -> list:
        """Get the position I gain for each motor."""
        return self._readSyncMotorsData("position_i_gain")

    def getPositionDGain(self) -> list:
        """Get the position D gain for each motor."""
        return self._readSyncMotorsData("position_d_gain")
    
    def getPresentCurrent(self) -> list:
        """Get the present current for each motor."""
        return self._readSyncMotorsData("present_current")
    
    def getPresentPWM(self) -> list:
        """Get the present PWM for each motor."""
        return self._readSyncMotorsData("pwm")