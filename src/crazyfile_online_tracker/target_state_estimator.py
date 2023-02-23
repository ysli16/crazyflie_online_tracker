#!/usr/bin/env python3
import rospy
from crazyflie_online_tracker.msg import TargetState
from state_estimator import StateEstimator


class TargetStateEstimator(StateEstimator):
    def __init__(self):
        super().__init__()
        rospy.init_node('target_state_estimator')
        self.state_pub_init()
        self.state_pub = rospy.Publisher('targetState', TargetState, queue_size=10)
        self.get_stationary_target()

    def state_pub_init(self):
        self.state_pub = rospy.Publisher('targetState', TargetState, queue_size=10)

    def publish_state(self):
        if self.state is not None:
            self.state_pub.publish(self.state)
        else:
            rospy.logdebug("The target state has not been assigned with any value.")

    def get_stationary_target(self):
        self.state = TargetState()


if __name__ == '__main__':
    target_state_estimator = TargetStateEstimator()
    rate = rospy.Rate(1)  # 1hz
    while not rospy.is_shutdown():
        target_state_estimator.publish_state()
        rate.sleep()
