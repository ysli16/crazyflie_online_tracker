#!/usr/bin/env python3
import rospy
from crazyflie_online_tracker.msg import CrazyflieState
from state_estimator import StateEstimator

from cflib.crazyflie.syncLogger import SyncLogger
import crazyflie_connection_helper # import xx gets the original object while from xx import yy gets a copy


class CrazyflieStateEstimator(StateEstimator):
    def __init__(self):
        super().__init__()
        rospy.init_node('crazyflie_state_estimator')
        self.state_pub_init()
        self.scf = crazyflie_connection_helper.helper.get_scf()
        self.cf = crazyflie_connection_helper.helper.get_cf()
        self.logconfig_state = crazyflie_connection_helper.helper.get_logconfig_state()

    def state_pub_init(self):
        self.state_pub = rospy.Publisher('crazyflieState', CrazyflieState, queue_size=10)

    def publish_state(self):
        self.get_state()
        self.state_pub.publish(self.state)

    def get_state(self):
        self.state = CrazyflieState()
        with SyncLogger(self.scf, self.logconfig_state) as logger:
            for log_entry in logger:
                data = log_entry[1]
                self.state.pose.position.x = data['stateEstimate.x']
                self.state.pose.position.y = data['stateEstimate.y']
                self.state.pose.position.z = data['stateEstimate.z']
                self.state.pose.orientation.x = data['stateEstimate.qx']
                self.state.pose.orientation.y = data['stateEstimate.qy']
                self.state.pose.orientation.z = data['stateEstimate.qz']
                self.state.pose.orientation.w = data['stateEstimate.qw']
                self.state.velocity.x = data['stateEstimate.vx']
                self.state.velocity.y = data['stateEstimate.vy']
                self.state.velocity.z = data['stateEstimate.vz']
                self.publish_state()
                break


if __name__ == '__main__':
    crazyflie_state_estimator = CrazyflieStateEstimator()
    rate = rospy.Rate(1)  # 1hz
    while not rospy.is_shutdown():
        crazyflie_state_estimator.publish_state()
        rate.sleep()
