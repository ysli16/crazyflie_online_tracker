#!/usr/bin/env python3
from abc import ABC, abstractmethod

import rospy
from crazyflie_online_tracker.msg import SetpointHL


class Controller(ABC):
    def __init__(self):
        self.droneState = None
        self.targetState = None
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

