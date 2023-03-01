# crazyflie_online_tracker
## Structure
This ROS package builds connection between the hardware(crazyflie/simulator) and the online control algorithm. It consists of three main components: state estimator, controller and actuator.
- State estimator gets current state of the hardware or the target and publish their states to rostopic `\targetState`(for target state) and `\crazyflieState`(for crazyflie).
- Controller subscribes the state estimator topics `\targetState` and `\crazyflieState`, computes the control conmmand based on current and historical states and publishes the control command to rostopic `\setpointHL`.
- Actuator substribes the controller topic `\setpointHL`, translates it to the hardware-specific type and sends the command.

## Known issue
Sending commands to crazyflie via Commander is not as stable comparing with MotionCommander or PositionHlCommander. Two issues are observed during experiments with the hover controller:
1. Although the commands are designed to be sent in constant frequency, some are received with delay, resulting in sudden jumps of the quadrotor hovering height. MotionCommander deals with is by creating a separate thread for command sneding. Maybe we should do that too. 
2. The quadrotor reacts agreesively to large errors between the current state and the setpoint. For example, when the quadrotor is on the ground, setting a hovering height to be 0.5m causes the quadrotor to take off too fast and flip over. Changing the setpoint gradually mitigate the issue(e.g. setting the hovering height to be 0.1m for 0.1s, then 0.2m for another 0.1s, etc.).

## Todo
1.run the hovering controller with crazyflie.
2.support Matlab simulation.
3.support Gazebo simulation.
