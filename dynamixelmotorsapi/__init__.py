from .dynamixelmotors import DynamixelMotors, motorgroup

@staticmethod
def listFTDIDevices() -> list:
    """
    List all the FTDI devices connected to the computer.
    
    Returns:
        A list of device names (the ports).
    """
    return motorgroup.listMotors()


@staticmethod
def listUnusedFTDIDevices() -> list:
    """
    List all the FTDI devices that are not currently used by any instance of DynamixelMotors in this process.
    
    Returns:
        A list of device names (the ports).
    """
    return [device for device in listFTDIDevices() if device not in DynamixelMotors._FTDI_list]


@staticmethod
def listUsedFTDIDevices() -> list:
    """
    List all the FTDI devices that are currently used by an instance of DynamixelMotors in this process.
    
    Returns:
        A list of device names (the ports).
    """
    return [device for device in DynamixelMotors._FTDI_list.keys()]