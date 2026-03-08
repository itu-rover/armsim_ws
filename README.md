# Arm26 Simulation Workspace

## Simulation Configurations

You can start the simulation in different configurations depending on your needs.

### Standard Simulation

To launch the standard simulation without any additional peripherals:

```
roslaunch arm26_config demo_gazebo.launch
```

### Keyboard Simulation

To start the simulation with the keyboard model included:

```
roslaunch arm26_config demo_gazebo.launch keyboard:=true
```

### Panel Simulation

To start the simulation with the interaction panel included:

```
roslaunch arm26_config demo_gazebo.launch panel:=true
```

## ArUco Detection

To start ArUco detect:

```
roslaunch aruco_detect aruco_detect.launch
```

## Behavior Tree

Behavior tree can be started using:

```
rosrun panel_bhv main.py
```
