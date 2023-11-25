#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import atan2, cos, sin, hypot, acos
from rt_gene.msg import MSG_Headpose, MSG_HeadposeList
from limit_cycle.msg import MSG_Headpose_Imp, MSG_HeadposeList_Imp
from pedsim_msgs.msg import TrackedPerson, TrackedPersons
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import rospy
from transformations import *
import numpy as np
import time
from tools import thetaConstrain
from param import BETA_GAZE

class gaze_feature:
    def __init__(self, head_pose):
        """
        key param:
        self.timestamped
        self.id
        self.delta_theta
        """
        self.value = 1.0
        self.id = int(head_pose.track_id)
        self.vel_on_map = [head_pose.twist.twist.linear.x, head_pose.twist.twist.linear.y]
        self.position_on_map = [head_pose.pose.pose.position.x, head_pose.pose.pose.position.y, euler_from_quaternion([head_pose.pose.pose.orientation.x,head_pose.pose.pose.orientation.y,head_pose.pose.pose.orientation.z,head_pose.pose.pose.orientation.w])[2]]
        self.pub_person_visuals_ = rospy.Publisher('my_people', TrackedPersons, queue_size=1)
        # print(self.id)
        # print(self.vel_on_map)
        # print(self.position_on_map)

    def update(self, head_pose):
        # self.value = 0.1
        self.value = 1.0
        # self.value = 3.0    
        self.vel_on_map = [head_pose.twist.twist.linear.x, head_pose.twist.twist.linear.y]
        self.position_on_map = [head_pose.pose.pose.position.x, head_pose.pose.pose.position.y, euler_from_quaternion([head_pose.pose.pose.orientation.x,head_pose.pose.pose.orientation.y,head_pose.pose.pose.orientation.z,head_pose.pose.pose.orientation.w])[2]]
        self.pubPerson()

    
    def getVel(self):
        return self.vel_on_map
        # if self.vel_on_map[0] == None:
        #     return [None, None]
        # speed = hypot(self.vel_on_map[0],self.vel_on_map[1])
        # return [speed*cos(self.position_on_map[2]), speed*sin(self.position_on_map[2])]
    
    def getPose(self):
        return self.position_on_map


        
    def print_(self):
        print('timestamped: ',self.timestamped)
        print('id: ', self.id)
        print('delta_theta: ', self.delta_theta/np.pi*180.0)
        print('gaze_value: ',self.value)
    
    def pubPerson(self):
        tracked_people = TrackedPersons()
        tracked_people.header.frame_id = 'map'

        person = TrackedPerson()

        person.track_id = self.id
        person.is_occluded = False
        person.detection_id = self.id

        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose.position.x = self.position_on_map[0]
        pose_with_cov.pose.position.y = self.position_on_map[1]
        pose_with_cov.pose.position.z = 0.0
        qua = quaternion_from_euler(0,0,self.position_on_map[2])
        pose_with_cov.pose.orientation.x = qua[0]
        pose_with_cov.pose.orientation.y = qua[1]
        pose_with_cov.pose.orientation.z = qua[2]
        pose_with_cov.pose.orientation.w = qua[3]
        person.pose = pose_with_cov

        twist_with_cov = TwistWithCovariance()
        twist_with_cov.twist.linear.x = self.vel_on_map[0]
        twist_with_cov.twist.linear.y = self.vel_on_map[1]
        person.twist = twist_with_cov

        tracked_people.tracks.append(person)
        # print(tracked_people)
        self.pub_person_visuals_.publish(tracked_people)

class gaze_bridge:
    def __init__(self):
        self.gaze_sub = rospy.Subscriber("/persons", TrackedPersons, self.processHeadPose, queue_size=3)
        self.headpose_dir = dict()

    def processHeadPose(self,msg):
        for head_pose in msg.tracks:
            if int(head_pose.track_id) not in self.headpose_dir.keys():
                self.headpose_dir[int(head_pose.track_id)] = gaze_feature(head_pose)
            else:
                self.headpose_dir[int(head_pose.track_id)].update(head_pose)


if __name__ == '__main__':
    rospy.init_node('headpose_listener', anonymous=True)
    gaze_box = gaze_bridge()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()





