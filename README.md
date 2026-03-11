# Robot Motors API

Robot Motors API is a simple and easy-to-use API for controling the Robot Motors robot from [Compliance Robotics](https://compliance-robotics.com/).

## Installation
To install the Robot Motors API, you can use pip:

```bash
python -m pip install git+https://github.com/SofaComplianceRobotics/Robot Motors.API.git@release-main
```

## Usage
TheDynamixel Motors API provides the `DynamixelMotors` class, which can be used to control the Motors. The API provides methods for controlling the robot's motors.
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

## For Developers
The documentation is generated using [pydoc-markdown](https://pypi.org/project/pydoc-markdown/). To generate the documentation, you need to install `pydoc-markdown`:

```bash
pipx install pydoc-markdown
```

Then, you can generate the documentation in a `dynamixelmotors-api.md` file, by using the following command at the root of the project:

```bash
pydoc-markdown
```
