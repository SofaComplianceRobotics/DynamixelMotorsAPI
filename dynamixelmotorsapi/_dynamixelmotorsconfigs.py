from dataclasses import dataclass
from math import pi
from typing import Dict, Optional


# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION = 2.0

BAUDRATE = 1000000

# Generic motor parameters
TORQUE_ENABLE     = 1
TORQUE_DISABLE    = 0
VELOCITY_MODE     = 1
POSITION_MODE     = 3
EXT_POSITION_MODE = 4


@dataclass(frozen=True)
class ModelConfig:
    """
    Hardware-level register map and resolution for a Dynamixel motor model.
    Frozen because these are fixed hardware constants — never mutated at runtime.

    To add a custom model at runtime, use register_model() rather than
    editing this file.
    """
    model: str
    series: str
    url: str

    resolution:               Optional[int] = None
    addr_torque_enable:       Optional[int] = None
    addr_operating_mode:      Optional[int] = None
    addr_goal_position:       Optional[int] = None
    len_goal_position:        Optional[int] = None
    addr_goal_velocity:       Optional[int] = None
    len_goal_velocity:        Optional[int] = None
    addr_present_position:    Optional[int] = None
    len_present_position:     Optional[int] = None
    addr_present_velocity:    Optional[int] = None
    len_present_velocity:     Optional[int] = None
    addr_velocity_profile:    Optional[int] = None
    addr_moving:              Optional[int] = None
    addr_moving_status:       Optional[int] = None
    addr_velocity_trajectory: Optional[int] = None
    len_velocity_trajectory:  Optional[int] = None
    addr_position_trajectory: Optional[int] = None
    len_position_trajectory:  Optional[int] = None
    addr_position_p_gain:     Optional[int] = None
    len_position_p_gain:      Optional[int] = None
    addr_position_i_gain:     Optional[int] = None
    len_position_i_gain:      Optional[int] = None
    addr_position_d_gain:     Optional[int] = None
    len_position_d_gain:      Optional[int] = None
    min_position_value:       Optional[int] = None
    max_position_value:       Optional[int] = None

    @property
    def rad_to_pulse(self) -> float:
        """Conversion factor from radians to pulses, derived from resolution."""
        return self.resolution / (2 * pi)


# Registry of all supported model configs.
# Add new model here, or at runtime via register_model().
MODELS_CONFIGS: Dict[str, ModelConfig] = {

    "X_SERIES": ModelConfig(
        model                    = "X_SERIES",
        series                   = "X_SERIES",
        url                      = "https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/",
        resolution               = 4096,
        addr_torque_enable       = 64,
        addr_operating_mode      = 11,
        addr_goal_position       = 116,
        len_goal_position        = 4,
        addr_goal_velocity       = 104,
        len_goal_velocity        = 4,
        addr_present_position    = 132,
        len_present_position     = 4,
        addr_present_velocity    = 128,
        len_present_velocity     = 4,
        addr_velocity_profile    = 112,
        addr_moving              = 122,
        addr_moving_status       = 123,
        addr_velocity_trajectory = 136,
        len_velocity_trajectory  = 4,
        addr_position_trajectory = 140,
        len_position_trajectory  = 4,
        addr_position_p_gain     = 84,
        len_position_p_gain      = 2,
        addr_position_i_gain     = 82,
        len_position_i_gain      = 2,
        addr_position_d_gain     = 80,
        len_position_d_gain      = 2,
        min_position_value       = 0,
        max_position_value       = 4095,
    ),

    "MX_SERIES": ModelConfig(
        # MX series with 2.0 firmware — same register map as X_SERIES
        model                    = "MX_SERIES",
        series                   = "MX_SERIES",
        url                      = "https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/",
        resolution               = 4096,
        addr_torque_enable       = 64,
        addr_operating_mode      = 11,
        addr_goal_position       = 116,
        len_goal_position        = 4,
        addr_goal_velocity       = 104,
        len_goal_velocity        = 4,
        addr_present_position    = 132,
        len_present_position     = 4,
        addr_present_velocity    = 128,
        len_present_velocity     = 4,
        addr_velocity_profile    = 112,
        addr_moving              = 122,
        addr_moving_status       = 123,
        addr_velocity_trajectory = 136,
        len_velocity_trajectory  = 4,
        addr_position_trajectory = 140,
        len_position_trajectory  = 4,
        addr_position_p_gain     = 84,
        len_position_p_gain      = 2,
        addr_position_i_gain     = 82,
        len_position_i_gain      = 2,
        addr_position_d_gain     = 80,
        len_position_d_gain      = 2,
        min_position_value       = 0,
        max_position_value       = 4095,
    ),

    "P_SERIES": ModelConfig(
        model                    = "P_SERIES",
        series                   = "P_SERIES",
        url                      = "https://emanual.robotis.com/docs/en/dxl/p/pm42-010-s260-r/",
        resolution               = 501923,
        addr_torque_enable       = 512,
        addr_operating_mode      = 11,
        addr_goal_position       = 564,
        len_goal_position        = 4,
        addr_goal_velocity       = 552,
        len_goal_velocity        = 4,
        addr_present_position    = 580,
        len_present_position     = 4,
        addr_present_velocity    = 576,
        len_present_velocity     = 4,
        addr_velocity_profile    = 560,
        addr_moving              = 570,
        addr_moving_status       = 571,
        addr_velocity_trajectory = 584,
        len_velocity_trajectory  = 4,
        addr_position_trajectory = 588,
        len_position_trajectory  = 4,
        addr_position_p_gain     = 532,
        len_position_p_gain      = 2,
        addr_position_i_gain     = 530,
        len_position_i_gain      = 2,
        addr_position_d_gain     = 528,
        len_position_d_gain      = 2,
        min_position_value       = -150000,
        max_position_value       = 150000,
    ),
}


def register_model(config: ModelConfig, overwrite: bool = False) -> None:
    """
    Register a custom modelConfig in the global registry at runtime.

    Use this for motor model not listed in this file, without needing to
    modify the library source.

    Args:
        config:    A fully constructed modelConfig instance.
        overwrite: If False (default), raises ValueError if the model name
                   is already registered. Set to True to replace an existing entry.

    Example:
        ```python
        from dynamixelmotorsapi._dynamixelmotorsparameters import modelConfig, register_model

        my_model = modelConfig(model="MY_CUSTOM_model", resolution=8192, ...)
        register_model(my_model)
        ```
    """
    if config.model in MODELS_CONFIGS and not overwrite:
        raise ValueError(
            f"model '{config.model}' is already registered. "
            f"Pass overwrite=True to replace it."
        )
    MODELS_CONFIGS[config.model] = config


def register_model_from_dict(data: dict, overwrite: bool = False) -> ModelConfig:
    """
    Build a modelConfig from a dict and register it.

    Validates that all required fields are present and reports all missing
    fields at once rather than failing on the first one.

    Args:
        data:      dict containing all ModelConfig fields.
        overwrite: passed through to register_model.

    Returns:
        The registered ModelConfig instance.

    Raises:
        ValueError: if any required fields are missing, listing all of them.
    """
    import dataclasses
    required_fields = {f.name for f in dataclasses.fields(ModelConfig)}
    missing = required_fields - data.keys()
    if missing:
        raise ValueError(
            f"Missing required fields for ModelConfig: {sorted(missing)}"
        )
    config = ModelConfig(**{k: data[k] for k in required_fields})
    register_model(config, overwrite=overwrite)
    return config


def register_model_from_json(path: str, overwrite: bool = False) -> list:
    """
    Load a JSON file containing a list of model config dicts and register
    each one into model_CONFIGS.

    Args:
        path:      path to a JSON file containing a list of ModelConfig dicts.
        overwrite: passed through to register_model for each entry.

    Returns:
        List of registered ModelConfig instances.

    Example JSON:
        ```json
        [
            {
                "model": "MY_CUSTOM_MODEL",
                "resolution": 8192,
                "addr_torque_enable": 64,
                "addr_operating_mode": 11,
                ...
            }
        ]
        ```
    """
    import json
    with open(path) as f:
        entries = json.load(f)
    if not isinstance(entries, list):
        raise ValueError(
            f"Expected a JSON array of model configs, got {type(entries).__name__}."
        )
    [register_model_from_dict(entry, overwrite=overwrite) for entry in entries]
    return MODELS_CONFIGS

@dataclass
class MotorConfig:
    """
    Per-motor configuration. Holds the varying parameters for a single motor
    alongside a reference to its model hardware config.

    The model can be specified in two ways:
    - By name (str): looked up in the global model_CONFIGS registry.
    - Directly (ModelConfig): for one-off or custom model without registering globally.

    If both are provided, they must refer to the same model name.

    Example (from name):
        ```python
        MotorConfig(id=0, model="XM430-W210", length_to_rad=0.05, pulse_center=2048, max_vel=1000)
        ```

    Example (ModelConfig directly):
        ```python
        MotorConfig(id=0, model_config=my_config, length_to_rad=0.05, pulse_center=2048, max_vel=1000)
        ```

    Example JSON (name-based, for from_dict / from_json):
        ```json
        {
            "id": 0,
            "model": "XM430-W210",
            "length_to_rad": 0.05,
            "pulse_center": 2048,
            "max_vel": 1000
        }
        ```
    """
    id:            int
    length_to_rad: float
    pulse_center:  int
    max_vel:       float

    # Provide one of these — not both (unless consistent), not neither
    model:        Optional[str]          = None
    model_config: Optional[ModelConfig] = None

    def __post_init__(self):
        if self.model_config is not None and self.model is not None:
            if self.model_config.model != self.model:
                raise ValueError(
                    f"Conflicting model: model='{self.model}' but "
                    f"model_config.model='{self.model_config.model}'. "
                    f"Provide one or the other, or ensure they match."
                )
            # Both given and consistent — model_config takes precedence
            return

        if self.model_config is not None:
            # ModelConfig provided directly — set model name for reference
            self.model = self.model_config.model
            return

        if self.model is not None:
            # Model name provided — look up in registry
            if self.model not in MODELS_CONFIGS:
                raise ValueError(
                    f"Unknown motor model '{self.model}'. "
                    f"Available: {list(MODELS_CONFIGS.keys())}. "
                    f"Use register_model() to add a custom model, "
                    f"or pass a ModelConfig directly."
                )
            self.model_config = MODELS_CONFIGS[self.model]
            return

        raise ValueError(
            "Either 'model' (str) or 'model_config' (ModelConfig) must be provided."
        )

    @property
    def rad_to_pulse(self) -> float:
        """Derived from the model resolution."""
        return self.model_config.rad_to_pulse

    @property
    def length_to_pulse(self) -> float:
        """Derived from length_to_rad and model resolution."""
        return self.length_to_rad * self.rad_to_pulse

    @classmethod
    def from_dict(cls, data: dict) -> "MotorConfig":
        """
        Instantiate from a dict. The model must be specified as a string name
        referring to a registered entry in MODEL_CONFIGS.
        """
        return cls(
            id            = data["id"],
            model         = data["model"],
            length_to_rad = data["length_to_rad"],
            pulse_center  = data["pulse_center"],
            max_vel       = data["max_vel"],
        )