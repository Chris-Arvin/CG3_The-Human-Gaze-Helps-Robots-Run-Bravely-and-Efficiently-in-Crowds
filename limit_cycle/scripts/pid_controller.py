#! /usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from geometry_msgs.msg import Twist
import numpy as np
from math import hypot, atan2

ARRIVED_DIS = 0.5

MAX_V = 0.5 # maximum speed [m/s]
MIN_V = -0.5  # minimum speed [m/s]
MAX_W = np.deg2rad(60.0) # maximum speed [m/s]
MIN_W = -np.deg2rad(60.0)  # minimum speed [m/s]
MAX_V_ACCEL = 0.5  # maximum accel [m/ss]
MAX_W_ACCEL = np.deg2rad(30.0)  # maximum accel [m/ss]        


def check_goal(state, goal):
    # check goal
    dx = state[0] - goal[0]
    dy = state[1] - goal[1]
    d = hypot(dx, dy)
    is_arrived = (d <= ARRIVED_DIS)
    if is_arrived:
        return "arrived"
    else:
        return "controlling"


def pi_2_pi(angle):
    while(angle > np.pi):
        angle = angle - 2.0 * np.pi

    while(angle < -np.pi):
        angle = angle + 2.0 * np.pi
    return angle



def pid_tracer(refer_x, refer_y, initial_state, goal_state):
    mode = check_goal(initial_state, goal_state)
    if mode == "arrived":
        cmd_vel = Twist()
        return cmd_vel
    else:
        dx = 0
        dy = 0
        for x,y in zip(refer_x,refer_y):
            dx = x-initial_state[0]
            dy = y-initial_state[1]
            if hypot(dx,dy) > ARRIVED_DIS:
                break
        v_desired = MAX_V
        dtheta = pi_2_pi(atan2(dy,dx)-initial_state[2])
        #todo 倒车会出现问题。。
        if abs(dtheta) > np.pi*0.5:
            v_desired = MAX_V* (np.pi-abs(dtheta)/np.pi)*0.2 #掉头时别太快
        w_desired = dtheta * 3.0
        v_desired = min(v_desired,MAX_V)
        v_desired = max(v_desired,MIN_V)
        v_desired = min(v_desired, initial_state[3]+MAX_V_ACCEL)
        v_desired = max(v_desired, initial_state[3]-MAX_V_ACCEL)
        w_desired = min(w_desired, MAX_W)
        w_desired = max(w_desired, MIN_W)
        w_desired = min(w_desired, initial_state[4]+MAX_W_ACCEL)
        w_desired = max(w_desired, initial_state[4]-MAX_W_ACCEL)
        cmd_vel = Twist()
        cmd_vel.linear.x = v_desired
        cmd_vel.angular.z = w_desired
        return cmd_vel
