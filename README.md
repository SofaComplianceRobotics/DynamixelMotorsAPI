# Dynamixel Motors API

Dynamixel Motors API is a simple and easy-to-use API for controling the Dynamixel motors from [Compliance Robotics](https://compliance-robotics.com/).

## Installation
To install the Dynamixel Motors API, you can use pip:

```bash
python -m pip install git+https://github.com/SofaComplianceRobotics/DynamixelMotorsAPI.git@release-main
```

## Usage
The Dynamixel Motors API provides the `DynamixelMotors` class, which can be used to control the Motors. The API provides methods for controlling the robot's motors.
You can look at the [motors_example.py](motors_example.py) file for a simple example of how to use the API to control the motors of the motors.


Simple example thaht sets the angles of 4 *XM430-W210* motors to 0 radians, waits for 1 second, and then prints the status of the robot:
```python
import time
from dynamixelmotorsapi import DynamixelMotors

motors_description = [
                        {
                            "id": [0, 1, 2, 3],
                            "model": "XM430-W210",
                            "length_to_rad": 0.05,
                            "pulse_center": 2048,
                            "max_vel": 1000
                        }
                    ]

robot_motors = DynamixelMotors.from_dicts(motors_description)
        
if robot_motors.open(): 
    
    robot_motors.printStatus()
    initial_pos_pulse = [0] * len(DXL_IDs)
    robot_motors.angles = initial_pos_pulse
    time.sleep(1)
    robot_motors.printStatus()
    robot_motors.close()

```


The catalog of Dynamixel motors has been compiled into the file [dynamixel_configs.json](dynamixelmotorsapi\dynamixel_configs.json).
They've been extracted from the Dynamixel website.

You have several ways to create a `DynamixelMotors` object, either by using the `from_dicts` method, which takes a list of dictionaries or one dictionnary describing the motors like above or you can also use a json file containing the same list of dictionaries, and use the `from_json` method. 

Except for the `id` parameter, which must be a list of unique IDs for each motor, the other parameters can be shared among motors of the same model or can also be lists of values for each motor, as long as they are consistent with the number of motors described in the `id` parameter.

Several syntaxes are possible for the motor description dictionaries, as long as they contain the required information (id, model, length_to_rad, pulse_center, max_vel).

```python
# Example of a motor description dictionary for 4 XM430-W210 motors, note that the IDs must be unique for each motor, but the other parameters can be shared among motors of the same model.
{
    "id": [0, 1, 2, 3],
    "model": "XM430-W210",
    "length_to_rad": 0.05,
    "pulse_center": 2048,
    "max_vel": 1000
}

# Equivalent to the above, but with the parameters as lists of values for each motor, which can be useful if you have motors of different models or with different configurations.
[
    {
        "id": [0, 1, 2, 3],
        "model": ["XM430-W210"]*4,
        "length_to_rad": [0.05]*4,
        "pulse_center": [2048, 2048, 2048, 2048],
        "max_vel": [1000]*4
    }
]

# Equivalent to the above, but with each motor described as a separate dictionary in the list, which can be useful if you want to have more flexibility in the configuration of each motor.
[
    {
        "id": 0,
        "model": "XM430-W210",
        "length_to_rad": 0.05,
        "pulse_center": 2048,
        "max_vel": 1000
    },
    {
        "id": 1,
        "model": "XM430-W210",
        "length_to_rad": 0.05,
        "pulse_center": 2048,
        "max_vel": 1000
    },
    {
        "id": 2,
        "model": "XM430-W210",
        "length_to_rad": 0.05,
        "pulse_center": 2048,
        "max_vel": 1000
    },
    {
        "id": 3,
        "model": "XM430-W210",
        "length_to_rad": 0.05,
        "pulse_center": 2048,
        "max_vel": 1000
    }
]
```


You can also create a `DynamixelMotors` object by directly passing a list of `MotorConfig` objects to the constructor.

## For Developers
The documentation is generated using [pydoc-markdown](https://pypi.org/project/pydoc-markdown/). To generate the documentation, you need to install `pydoc-markdown`:

```bash
pipx install pydoc-markdown
```

Then, you can generate the documentation in a `dynamixelmotors-api.md` file, by using the following command at the root of the project:

```bash
pydoc-markdown
```
