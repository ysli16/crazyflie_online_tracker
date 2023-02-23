#!/usr/bin/env python3
from abc import ABC, abstractmethod

import rospy
from crazyflie_online_tracker.msg import SetpointHL


class Actuator(ABC):
    def __init__(self):
        self.actuator_pub = None
        self.setpoint_sub = rospy.Subscriber("setpointHL", SetpointHL, self.callback_setpoint)

    @abstractmethod
    def callback_setpoint(self, data):
        pass

    @abstractmethod
    def actuator_pub_init(self):
        pass
