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


Simple example thaht sets the angles of the motors to 0 radians, waits for 1 second, and then prints the status of the robot:
```python
import time
from dynamixelmotorsapi import DynamixelMotors

# Define your own class of motors that inherits DynamixelMotors class
class MyDynamixelMotors(DynamixelMotors):
    _length_to_rad = 1.0 / 20.0  # 1/radius of the pulley
    _rad_to_pulse = 4096 / (2 * pi)  # the resolution of the Dynamixel xm430 w210
    _pulse_center= 2048
    _max_vel = 1000  # *0.01 rev/min

    def __init__(self):
        super().__init__() # Check if all parameters have been set

robot_motors = MyDynamixelMotors()
        
if robot_motors.open(): 
    
    robot_motors.printStatus()
    initial_pos_pulse = [0] * len(DXL_IDs)
    robot_motors.angles = initial_pos_pulse
    time.sleep(1)
    robot_motors.printStatus()
    robot_motors.close()

```

By default, there are four motors with IDS 0, 1, 2, and 3. You can change this in the [_dynamixelmotorsparameters.py](dynamixelmotorsapi/_dynamixelmotorsparameters.py) at line 21:

``` python
21 |    DXL_IDs = (0, 1, 2, 3)
```

To change the motors parameters based on your motors, like the addresses and values, you can also change them into the file [_dynamixelmotorsparameters.py](dynamixelmotorsapi/_dynamixelmotorsparameters.py) if needed.

The parameters already include some Dynamixel motor series so you can easily switch by commenting and uncommenting the right line:
```python
 4 |    #***** (Use only one definition at a time) *****
 5 |    MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
 6 |    # MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
 7 |    # MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
 8 |    # MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
 9 |    # MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
10 |    # MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V
```

⚠️ For now, the API assumes that all the motors are the same

## For Developers
The documentation is generated using [pydoc-markdown](https://pypi.org/project/pydoc-markdown/). To generate the documentation, you need to install `pydoc-markdown`:

```bash
pipx install pydoc-markdown
```

Then, you can generate the documentation in a `dynamixelmotors-api.md` file, by using the following command at the root of the project:

```bash
pydoc-markdown
```
