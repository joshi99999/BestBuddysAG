# RO45 portal robot interface package

This package contains the interface definition for communicating with the portal robot used in the RO45 project.

The robot subscribes to the defined `RobotCmd` messages and publishes the defined `RobotPos` messages.
The `RobotCmd` message contains the commanded velocity for the x-, y-, and z-axis as well as a bool indicating whether the gripper shall be activated or not.
The `RobotPos` message contains the measured position of the robot.

## Build
To build the interface package clone it into the `src` folder inside your ros2 workspace directory and run
```bash
colcon build --packages-select ro45_portalrobot_interfaces
```
from the workspace directory.
