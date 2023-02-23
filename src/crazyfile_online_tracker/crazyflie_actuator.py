#!/usr/bin/env python3
import rospy
from crazyflie_online_tracker.msg import CommandCF
from cflib.utils import uri_helper
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from actuator import Actuator
import crazyflie_connection_helper


class SetpointType:
    VELOCITY_SETPOINT = 0
    POSITON_SETPOINT = 1
    UNSUPPORTED_SETPOINT = 9


class CrazyflieActuator(Actuator):
    def __init__(self):
        super().__init__()
        self.scf = crazyflie_connection_helper.helper.get_scf()
        self.cf = crazyflie_connection_helper.helper.get_cf()
        rospy.init_node('crazyflie_actuator')
        self.actuator_pub_init()

    def actuator_pub_init(self):
        self.actuator_pub = rospy.Publisher('cfCommand', CommandCF, queue_size=10)

    def callback_setpoint(self, data):
        setpoint_type = self.check_setpoint(data.posMode)
        command = CommandCF()
        command.setpointType = setpoint_type
        if setpoint_type == SetpointType.VELOCITY_SETPOINT:
            command.x = data.velocity.x
            command.y = data.velocity.y
            command.z = data.position.z
            command.yaw = 0
            self.cf.commander.send_hover_setpoint(command.x, command.y, command.yaw, command.z)
        elif setpoint_type == SetpointType.POSITON_SETPOINT:
            command.x = data.position.x
            command.y = data.position.y
            command.z = data.position.z
            command.yaw = data.yaw
            self.cf.commander.send_position_setpoint(command.x, command.y, command.z, command.yaw)
        else:
            rospy.logdebug("setpoint with mode" + data.posMode + "is currently not supported. Send stop setpoint instead.")
            self.cf.commander.send_stop_setpoint()
        self.actuator_pub.publish(command)

    def check_setpoint(self, posMode):
        """
        This function checks the validility of the received setpoint.
        """
        if ~posMode.x and ~posMode.y and posMode.z:
            return SetpointType.VELOCITY_SETPOINT
        elif posMode.x and posMode.y and posMode.z:
            return SetpointType.POSITON_SETPOINT
        else:
            return SetpointType.UNSUPPORTED_SETPOINT


if __name__ == '__main__':
    rospy.loginfo("test.")
    cf_actuator = CrazyflieActuator()
    rospy.spin()
