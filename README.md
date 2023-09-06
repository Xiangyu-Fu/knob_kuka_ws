# knob_kuka_ws

## First Easy Demo
Open the simulation and the rosserial node:
```bash
$ roslaunch knob_robot_control lwr_sim.launch
$ roslaunch knob_robot_control rosserial.launch
```

Then Start the `lwr_controller` node:
```bash
$ rosrun knob_robot_control lwr_controller.py
```


## Real Robot Demo

```bash
ssh fortiss@10.200.2.61
roslaunch iiwa_driver start.launch
```

```bash
cobot_mode
roslaunch knob_robot_control rosserial.launch
```
    
```bash
roslaunch knob_robot_control knob_controller.launch
```

## GUI Demo

TODO: 

GUI interface for the robot control