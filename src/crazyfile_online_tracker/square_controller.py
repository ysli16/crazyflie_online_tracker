#!/usr/bin/env python3
import rospy
import numpy as np
import math
import time
from crazyflie_online_tracker.msg import SetpointHL, CrazyflieState, TargetState
from controller import Controller, StateIndex, MotionIndex


class SquareController(Controller):
    def __init__(self):
        super().__init__()
        self.droneState = np.zeros(9)
        self.motion = MotionIndex.stop
        self.forwardMax = 0.5
        self.sideMax = 0.5
        rospy.init_node('square_controller')
        self.drone_state_sub = rospy.Subscriber('crazyflieState', CrazyflieState, self.callback_state_drone)
        self.target_state_sub = rospy.Subscriber('targetState', TargetState, self.callback_state_target)        
        # TODO: function for smooth takeoff

    def callback_state_drone(self, data):
        self.droneState[StateIndex.x] = data.pose.position.x
        self.droneState[StateIndex.y] = data.pose.position.y
        self.droneState[StateIndex.z] = data.pose.position.z
        self.droneState[StateIndex.vx] = data.velocity.linear.x
        self.droneState[StateIndex.vy] = data.velocity.linear.y
        self.droneState[StateIndex.vz] = data.velocity.linear.z
        q1 = data.pose.orientation.x
        q2 = data.pose.orientation.y
        q3 = data.pose.orientation.z
        q0 = data.pose.orientation.w
        # github michaelwro/quat2eulers.py
        self.droneState[StateIndex.roll] = math.atan2(2 * ((q2 * q3) + (q0 * q1)), q0**2 - q1**2 - q2**2 + q3**2)
        self.droneState[StateIndex.pitch] = math.asin(2 * ((q1 * q3) - (q0 * q2)))
        self.droneState[StateIndex.yaw] = math.atan2(2 * ((q1 * q2) + (q0 * q3)),q0**2 + q1**2 - q2**2 - q3**2)
        self.compute_setpoint()
        self.publish_setpoint()

    def callback_state_target(self, data):
        self.targetState = data

    def compute_setpoint(self):
        setpointHL = SetpointHL()
        if self.motion == MotionIndex.stop:
            setpointHL.velocity.x = 0
            setpointHL.velocity.y = 0
        if self.motion == MotionIndex.forward:
            setpointHL.velocity.x = 0
            setpointHL.velocity.y = 0.2
        if self.motion == MotionIndex.right:
            setpointHL.velocity.x = 0.2
            setpointHL.velocity.y = 0
        if self.motion == MotionIndex.backward:
            setpointHL.velocity.x = 0
            setpointHL.velocity.y = -0.2
        if self.motion == MotionIndex.left:
            setpointHL.velocity.x = -0.2
            setpointHL.velocity.y = 0
        if self.motion == MotionIndex.forward and self.droneState[StateIndex.y]>self.forwardMax:
            self.motion = MotionIndex.right
        if self.motion == MotionIndex.right and self.droneState[StateIndex.x]>self.sideMax:
            self.motion = MotionIndex.backward
        if self.motion == MotionIndex.backward and self.droneState[StateIndex.y]<0:
            self.motion = MotionIndex.left
        if self.motion == MotionIndex.left and self.droneState[StateIndex.x]<0:
            self.motion = MotionIndex.forward
        setpointHL.posMode.x = False
        setpointHL.posMode.y = False
        setpointHL.posMode.z = True
        setpointHL.position.z = self.hover_height
        setpointHL.yaw = self.hover_yaw
        self.setpointHL = setpointHL

    def publish_setpoint(self):
        self.compute_setpoint()
        self.setpointHL_pub.publish(self.setpointHL)


if __name__ == '__main__':
    square_controller = SquareController()
    # time.sleep(3)
    square_controller.motion = MotionIndex.forward
    rospy.spin()
