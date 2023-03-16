# crazyflie_online_tracker
## Structure
This ROS package builds connection between the hardware(crazyflie/simulator) and the online control algorithm. It consists of three main components: state estimator, controller and actuator.
- State estimator gets current state of the hardware or the target and publish their states to rostopic `\targetState`(for target state) and `\crazyflieState`(for crazyflie).
- Controller subscribes the state estimator topics `\targetState` and `\crazyflieState`, computes the control conmmand based on current and historical states and publishes the control command to rostopic `\setpointHL`.
- Actuator substribes the controller topic `\setpointHL`, translates it to the hardware-specific type and sends the command.

This repo also contains a matlab simulator that receives `\setpointHL` messages from a python controller via ROS, executes the commands with a non-linear Simulink model and publish next states of the quadrotor via ROS.

## How to simulate the controller in MATLAB
0. build the package
1. run `roslaunch crazyflie_online_tracker <launch_file>.launch` where `<launch_file>` can be chosen from `hover_matlab`, `square_matlab` and `FTL_matlab`.
2. Open the script `/src/crazyflie_online_tracker/matlab_simulator/matlab_simulator.m` in MATLAB, put a breakpoint before the line `rosshutdown` and then run the script.
3. click buttom `continue` in MATLAB to terminate the ros node created by MATLAB properly.

## Known issue
Sending commands to crazyflie via Commander is not as stable comparing with MotionCommander or PositionHlCommander. Two issues are observed during experiments with the hover controller:
1. Although the commands are designed to be sent in constant frequency, some are received with delay, resulting in sudden jumps of the quadrotor hovering height. MotionCommander deals with is by creating a separate thread for command sneding. Maybe we should do that too. 
2. The quadrotor reacts agreesively to large errors between the current state and the setpoint. For example, when the quadrotor is on the ground, setting a hovering height to be 0.5m causes the quadrotor to take off too fast and flip over. Changing the setpoint gradually mitigate the issue(e.g. setting the hovering height to be 0.1m for 0.1s, then 0.2m for another 0.1s, etc.).

## Todo
1. Investigate the cause of the issue1 above.
2. Add function for smooth taking off. Find the most stable hover height.
3. Support Gazebo simulation.
