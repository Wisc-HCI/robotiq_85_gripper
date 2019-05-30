# robotiq_85_gripper

## Original README
Common packages for the Robotiq 85 Gripper provided by Stanley Innovation

Defaults to 'ttyUSB0' and 115200 baud rate

Single gripper and left gripper (in dual gripper configuration) need to be configured as device 9 using the Robotiq User Interface
Right gripper (in dual gripper configuration) need to be configured as device 9 using the Robotiq User Interface


start with:
```
roslaunch robotiq_85_bringup robotiq_85.launch run_test:=true
```

## Fork Notes
Repository forked from [waypointrobotics/robotiq_85_gripper](https://github.com/waypointrobotics/robotiq_85_gripper).

Why use this version?
- removed Kinova arm coupler from URDF (applicable to more robots off the shelf).
- provides a gripper_action_server to convert GripperCommandActions used by
planning pipelines like MoveIt into driver specific messages.
- Referenced by [Wisc-HCI/robot_configurations](https://github.com/Wisc-HCI/robot_configurations) repository.
