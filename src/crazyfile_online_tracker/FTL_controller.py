#!/usr/bin/env python3
import rospy
import numpy as np
from scipy import linalg
import math
import matplotlib.pyplot as plt
from crazyflie_online_tracker.msg import SetpointHL, CrazyflieState, TargetState
from controller import Controller, StateIndex

class FTLController(Controller):
    def __init__(self):
        super().__init__()
        # linear dynamic model: x_{t+1} = Ax_t + Bu_t
        # x = [x,y,z,vx,vy,vz,roll,pitch,yaw]
        # u = [vx, vy, z, yaw]
        self.A = np.array([[1, 0,       0,   0.0963,      0,      0,       0, 0.0396,      0], 
                           [0, 1,       0,        0, 0.0963,      0, -0.0396,      0,      0],
                           [0, 0,  0.9911,        0,      0, 0.0953,       0,      0,      0],

                           [0, 0,       0,   0.8943,      0,      0,       0, 0.7027,      0],
                           [0, 0,       0,        0, 0.8943,      0, -0.7027,      0,      0],
                           [0, 0, -0.1776,        0,      0, 0.9034,       0,      0,      0],

                           [0, 0,       0,        0, 0.1932,      0,  0.4524,      0,      0],
                           [0, 0,       0,  -0.1932,      0,      0,       0, 0.4524,      0],
                           [0, 0,       0,        0,      0,      0,       0,      0, 0.5454]])
        
        self.B = np.array([[0.0037,       0,      0,      0],
                           [     0,  0.0037,      0,      0],
                           [     0,       0, 0.0089,      0],

                           [0.1057,       0,      0,      0],
                           [     0,  0.1057,      0,      0],
                           [     0,       0, 0.1776,      0],

                           [     0, -0.1932,      0,      0],
                           [0.1932,       0,      0,      0],
                           [     0,       0,      0, 0.4546]])
        # loss function: l_t = (x_t - g_t)^TQ(x_t - g_t) + u_t^Tu_t
        self.Q = np.zeros((9,9))
        self.Q[0, 0] = 1
        self.Q[1, 1] = 1
        self.R = np.eye(4)
        # policy: pi_t(x) = -Kx + c_t
        self.solve_K_star()
        self.c = np.zeros((4, 1))
        self.c_log = [self.c]
        # value function: V(x, u)=(x^T u^T)P(x u)^T + L^T(x u)^T
        self.solve_P()
        self.P_sum = self.P
        self.P_log = [self.P]
        self.L = None
        self.L_sum = 0
        self.L_log = []
        
        self.action_log = []

        rospy.init_node('square_controller')
        self.drone_state_sub = rospy.Subscriber('crazyflieState', CrazyflieState, self.callback_state_drone)
        self.target_state_sub = rospy.Subscriber('targetState', TargetState, self.callback_state_target)        
        
        # TODO: smooth takeoff

    def callback_state_drone(self, data):
        self.droneState = np.zeros((9, 1))
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
        # following conversion is from github michaelwro/quat2eulers.py
        self.droneState[StateIndex.roll] = math.atan2(2 * ((q2 * q3) + (q0 * q1)), q0**2 - q1**2 - q2**2 + q3**2)
        self.droneState[StateIndex.pitch] = math.asin(2 * ((q1 * q3) - (q0 * q2)))
        self.droneState[StateIndex.yaw] = math.atan2(2 * ((q1 * q2) + (q0 * q3)),q0**2 + q1**2 - q2**2 - q3**2)

    def callback_state_target(self, data):
        self.targetState = np.zeros((9, 1))
        self.targetState[StateIndex.x] = data.pose2D.x
        self.targetState[StateIndex.y] = data.pose2D.y
        self.targetState[StateIndex.yaw] = data.pose2D.theta
        self.targetState[StateIndex.vx] = data.velocity.linear.x
        self.targetState[StateIndex.vy] = data.velocity.linear.y
    
    def solve_K_star(self):
        # compute the gain matrix of LQR(assuming all target vectors are zero)
        S = linalg.solve_discrete_are(self.A, self.B, self.Q, self.R)
        B_transpose_S = self.B.T @ S
        self.K_star = linalg.inv(self.R + B_transpose_S @ self.B) @ B_transpose_S @ self.A

    def solve_P(self):
        self.A_aug = np.vstack((np.hstack((self.A,self.B)),
                                np.hstack((-self.K_star@self.A, -self.K_star@self.B)))).T
        Q_aug = np.vstack((np.hstack((self.Q, np.zeros((9, 4)))),
                           np.hstack((np.zeros((4, 9)), np.eye(4)))))
        self.P = linalg.solve_discrete_lyapunov(self.A_aug, Q_aug)
    
    def solve_L(self):
        # rearrange eq(4) and solve the linear system AL = b
        b = 2 * self.A_aug @ self.P.T @ np.vstack((np.zeros((9,1)), self.c)) \
            - np.vstack((2* self.Q @ self.targetState, np.zeros((4, 1))))
        A = np.eye(13) - self.A_aug
        self.L = linalg.solve(A, b)
        self.L_sum = self.L_sum + self.L
        self.L_log += [self.L]

    def update_policy(self):
        if self.targetState is None:
            rospy.loginfo("target state has not been initialized.")
            return
        self.P_sum = self.P_sum + self.P # P_t is unchanged according to lemma3
        self.solve_L()
        self.c = -0.5 * linalg.inv(self.P_sum[-4:, -4:]) @ self.L_sum[-4:]
        self.c_log += [self.c]
    
    def compute_setpoint(self):
        if self.droneState is None:
            rospy.loginfo("drone state has not been initialized.")
            return
        action = -self.K_star @ self.droneState + self.c
        setpointHL = SetpointHL()
        setpointHL.velocity.x = action[0]
        setpointHL.velocity.y = action[1]
        setpointHL.posMode.x = False
        setpointHL.posMode.y = False
        setpointHL.posMode.z = True
        setpointHL.position.z = self.hover_height
        setpointHL.yaw = self.hover_yaw
        self.setpointHL = setpointHL
        self.action_log += [action]

    def publish_setpoint(self):
        self.compute_setpoint()
        if self.setpointHL is not None:
            self.setpointHL_pub.publish(self.setpointHL)

def plot(actions):
    time = np.arange(0, len(actions))
    fig, (ax1, ax2) = plt.subplots(2)
    ax1.plot(time, actions[:, 0])
    ax1.set_xlabel("time[s]")
    ax1.set_ylabel("vx_ref[m/s]")
    ax2.plot(time, actions[:, 1])
    ax2.set_xlabel("time[s]")
    ax2.set_ylabel("vy_ref[m/s]")

if __name__ == '__main__':
    FTL_controller = FTLController()
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        FTL_controller.publish_setpoint()
        FTL_controller.update_policy()
        rate.sleep()
