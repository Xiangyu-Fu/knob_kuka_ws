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