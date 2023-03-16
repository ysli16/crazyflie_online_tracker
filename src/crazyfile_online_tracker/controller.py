#!/usr/bin/env python3
from abc import ABC, abstractmethod

import rospy
from crazyflie_online_tracker.msg import SetpointHL, CrazyflieState, TargetState


class Controller(ABC):
    def __init__(self):
        self.droneState = None
        self.targetState = None
        self.setpointHL = None
        self.hover_height = 0.3
        self.hover_yaw = 0
        self.setpointHL_pub = rospy.Publisher('setpointHL', SetpointHL, queue_size=10)

    @abstractmethod
    def callback_state_drone(self, data):
        pass

    @abstractmethod
    def callback_state_target(self, data):
        pass

    @abstractmethod
    def compute_setpoint(self):
        pass

    @abstractmethod
    def publish_setpoint(self):
        pass

class StateIndex():
    x = 0
    y = 1
    z = 2
    vx = 3
    vy = 4
    vz = 5
    roll = 6
    pitch = 7
    yaw = 8


class MotionIndex():
    stop = 0
    forward = 1
    right = 2
    backward = 3
    left = 4
