#!/usr/bin/env python3
import rospy
from cflib.utils import uri_helper
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
import time


class CrazyflieConnectionHelper:
    def __init__(self):
        self.URI = uri_helper.uri_from_env(default='radio://0/0/2M/E7E7E7E7E7')
        cflib.crtp.init_drivers()

        logconfig_state = LogConfig(name='state', period_in_ms=10)
        logconfig_state.add_variable('stateEstimate.x', 'float')
        logconfig_state.add_variable('stateEstimate.y', 'float')
        logconfig_state.add_variable('stateEstimate.z', 'float')
        logconfig_state.add_variable('stateEstimate.qw', 'float')
        logconfig_state.add_variable('stateEstimate.qx', 'float')
        logconfig_state.add_variable('stateEstimate.qy', 'float')
        logconfig_state.add_variable('stateEstimate.qz', 'float')
        logconfig_state.add_variable('stateEstimate.vx', 'float')
        logconfig_state.add_variable('stateEstimate.vy', 'float')
        logconfig_state.add_variable('stateEstimate.vz', 'float')
        self.logconfig_state = logconfig_state

        # with SyncCrazyflie(self.URI, cf=Crazyflie(rw_cache='./cache')) as self.scf:
        #     self.take_off_simple()

        self.scf = SyncCrazyflie(self.URI, cf=Crazyflie(rw_cache='./cache'))
        self.cf = self.scf.cf
        self.scf.open_link()
        self.take_off_simple()
        # self.cf.param.set_value('kalman.resetEstimation', '1')
        # time.sleep(0.1)
        # self.cf.param.set_value('kalman.resetEstimation', '0')
        # time.sleep(2)
        # for y in range(10):
        #     self.cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
        #     time.sleep(0.1)

        # for _ in range(20):
        #     self.cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
        #     time.sleep(0.1)

        # for _ in range(50):
        #     self.cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, 0.4)
        #     time.sleep(0.1)

        # for _ in range(50):
        #     self.cf.commander.send_hover_setpoint(0.5, 0, -36 * 2, 0.4)
        #     time.sleep(0.1)

        # for _ in range(20):
        #     self.cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
        #     time.sleep(0.1)

        # for y in range(10):
        #     self.cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
        #     time.sleep(0.1)
        # self.cf.commander.send_stop_setpoint()

    def get_scf(self):
        if self.scf is None:
            rospy.loginfo("SyncCrazyflie is not initialized.")
        return self.scf

    def get_cf(self):
        if self.cf is None:
            rospy.loginfo("Crazyflie is not initialized.")
        return self.cf

    def get_logconfig_state(self):
        return self.logconfig_state

    def take_off_simple(self):
        with MotionCommander(self.scf, default_height=0.5) as mc:
            time.sleep(3)

helper = CrazyflieConnectionHelper()
