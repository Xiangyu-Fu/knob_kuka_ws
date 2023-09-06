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


```
0 -0 180
[ INFO] [1694018281.682000770]: Sending move position to robot: -0.0827, 0.7209, 0.1761, 0.0000, -0.0000, 3.1416  with speed: 0.20 %
[WARN] [1694018281.695096]: Sent: lin move : -82.6618596911 720.944523811 176.11348629 0.0 -0.0 3.14159274101 0.20000000298 0.0

[ INFO] [1694018281.912623177]: Set speed to 25.00 %
[ INFO] [1694018281.914085598]: Moving home
[ INFO] [1694018281.914139837]: Set speed to 25.00 %
[ INFO] [1694018282.364963888]: Waiting for robot to be in stationary mode. current status 2
[ INFO] [1694018282.688847296]: Command result for ID 7 = 0
[ INFO] [1694018282.688915332]: All commands executed. Setting robot to stationary
[ INFO] [1694018282.731815341]: Control gripper: isOpen: 1, range: 0.105000, speed: 0.150000, force: 40.000000, acceleration: 0.100000
[ INFO] [1694018282.757396034]: Open gripper called: 0.1050, 40.0000
[ INFO] [1694018282.957558874]: Sending move joint to robot: 1.5710, 0.2620, 0.0000, -1.2530, 0.0000, 1.6280, -1.5710  with speed: 0.25 %
[WARN] [1694018282.984969]: Sent: joint move : 1.57099997997 0.261999994516 0.0 -1.25300002098 0.0 1.62800002098 -1.57099997997 0.25 0.25 0.25 0.25 0.25 0.25 0.25 0.0

```