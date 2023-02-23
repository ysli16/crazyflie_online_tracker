#!/usr/bin/env python3
import rospy
from crazyflie_online_tracker.msg import SetpointHL
from dynamic_reconfigure.server import Server
from crazyflie_online_tracker.cfg import HoverControllerConfig
from controller import Controller


class HoverController(Controller):
    def __init__(self):
        super().__init__()
        rospy.init_node('hover_controller')
        self.dynamic_reconfig_server = Server(HoverControllerConfig, self.set_hover_param)
        self.hover_height = rospy.get_param('/hover_controller/hover_height')
        self.hover_yaw = rospy.get_param('/hover_controller/hover_yaw')
        self.setpointHL = None

    def callback_state_drone(self, data):
        self.droneState = data

    def callback_state_target(self, data):
        self.targetState = data

    def compute_setpoint(self):
        setpointHL = SetpointHL()
        setpointHL.posMode.x = False
        setpointHL.velocity.x = 0
        setpointHL.posMode.y = False
        setpointHL.velocity.y = 0
        setpointHL.posMode.z = True
        setpointHL.position.z = self.hover_height
        setpointHL.yaw = self.hover_yaw
        self.setpointHL = setpointHL

    def publish_setpoint(self):
        self.compute_setpoint()
        self.setpointHL_pub.publish(self.setpointHL)

    def set_hover_param(self, config, level):
        rospy.loginfo("Config set to {hover_height}, {hover_yaw}".format(**config))
        self.hover_height = config.hover_height
        self.hover_yaw = config.hover_yaw
        return config


if __name__ == '__main__':
    hover_controller = HoverController()
    rate = rospy.Rate(1)  # 1hz
    while not rospy.is_shutdown():
        hover_controller.publish_setpoint()
        rate.sleep()
