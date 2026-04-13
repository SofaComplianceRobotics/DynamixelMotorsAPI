from importlib.resources import files

from .dynamixelmotors import DynamixelMotors, motorgroup
from dynamixelmotorsapi._dynamixelmotorsconfigs import register_model_from_json, MODELS_CONFIGS
from dynamixelmotorsapi._logging_config import logger

try:
    _config_path = files("dynamixelmotorsapi").joinpath("dynamixel_configs.json")
    loaded_models = register_model_from_json(_config_path, overwrite=True)
    if len(loaded_models):
        logger.info(f"Loaded  {len(MODELS_CONFIGS)}  motor configs")
except Exception as e:
    logger.error(f"Failed to load motor configs from JSON: {e}")


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