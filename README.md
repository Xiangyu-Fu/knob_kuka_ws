# knob_kuka_ws
## Msg generation

rosrun rosserial_arduino make_libraries.py /home/fu/repos/fortiss_knob/lib

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

## Trouble Shooting


###  ModuleNotFoundError: No module named 'PyQt5.QtChart'

```
pip install -U pip
pip install pyqt5 --config-settings --confirm-license= --verbose
```


## UPDATE in Dec.20.2023

Frist, you need to change the robot IP in the `src/robot_movement_interface/iiwa_driver/ros/config/config.yaml`

Then open the terminals and run the following commands:

```bash
roslaunch iiwa_driver start.launch
rosrun knob_robot_control robot_controller.py
```

## UPDATE in Jan.04.2024

```bash
roslaunch iiwa_driver start.launch
roslaunch knob_robot_control knob_controller.launch
```


[WARN] [1705503647.144380]: linstiff move : -83.7400034070015 454.6999931335449 254.67917323112488 0.0 -0.0 3.140000104904175 0.10000000149011612 700 5.0 5.0 5.0 0.1 0.1 0.1

[WARN] [1705503742.630102]: linstiff move-83.7400034070015 : 454.6999931335449 : 259.09721851348877 : 0.0 : -0.0 : 3.140000104904175 0.10000000149011612 700 5.0 5.0 5.0 0.1 0.1 0.1
