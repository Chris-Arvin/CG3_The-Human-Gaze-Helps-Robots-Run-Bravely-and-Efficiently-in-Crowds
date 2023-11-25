#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
print('---',sys.version)

from math import hypot, sin, cos
import rospy
from api2python.srv import api_info
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from srv_preprocess import preprocess
# from gaze_bridge import gaze_bridge
from gaze_bridge_for_sim import gaze_bridge
import numpy as np
from transformations import *
from limit_cycle_game import people, robot
from open_loop import open_loop
from close_loop import close_loop
from pid_controller import pid_tracer
import time
from param import U_MAX, R_PEOPLE,R_ROBOT


class Duel_loop:
    def __init__(self):
        self.res= rospy.Service('local_plan', api_info, self.startCb)
        self.gaze_box = gaze_bridge()
        self.open_loop_obj = open_loop()
        self.close_loop_obj = close_loop()
        self.pre_obj = preprocess()
        self.last_decision_time = None
        self.last_mode = None
        self.last_u = None
        self.last_dir = None
        self.last_dis_p_r = None
        self.last_opt_x_list = None
        self.last_opt_y_list = None
    
    def startCb(self, req):
        # # test for input
        # pub_map = rospy.Publisher('my_map',OccupancyGrid,queue_size=10)
        # pub_path = rospy.Publisher('my_path',Path,queue_size=10)
        # pub_map.publish(req.map)
        # pub_path.publish(req.reference_path)

        # receive request and process map
        self.pre_obj.process(req.map, req.robot_pose, req.robot_vel, req.reference_path)
        initial_state = [self.pre_obj.current_pose[0], self.pre_obj.current_pose[1], self.pre_obj.current_pose[2], self.pre_obj.vel[0], self.pre_obj.vel[1]]
        goal_state = [self.pre_obj.local_goal[0], self.pre_obj.local_goal[1]]
        # gaze_bridge to update headposes
        """
        HGPG starting
        """
        # find the nearest person
        self.nearest_id, rob_list, nearest_person_list, nearest_dis, state = self.findRelatedPerson()
        self.rob_obj = robot()
        self.rob_obj.pubPath([pos[0] for pos in rob_list], [pos[1] for pos in rob_list], 'robot_list')
        self.rob_obj.pubPath([pos[0] for pos in nearest_person_list], [pos[1] for pos in nearest_person_list], 'nearest_person_list')
        
        if self.nearest_id == -1:
            cmd_vel = pid_tracer(self.pre_obj.refer_x, self.pre_obj.refer_y, initial_state, goal_state)
            self.rob_obj.pubPath(self.pre_obj.refer_x, self.pre_obj.refer_y, 'robot_path')
            self.rob_obj.pubMarker([initial_state[0], initial_state[1]],0)
            self.last_decision_time = None
            self.last_mode = None 
            # print('-----------------------')
            # print('follow global path')
            # print('can not find the nearest people')
            return cmd_vel
            # return self.zeroVel()

        self.nearest_peole_feature_obj = self.gaze_box.headpose_dir[self.nearest_id]

        # judge the loop mode
        # print(self.gaze_box.headpose_dir[self.nearest_id])
        people_pose = self.nearest_peole_feature_obj.getPose()
        people_vel = self.nearest_peole_feature_obj.getVel()
        # if people_vel == [0.0,0.0]:
        #     print('initializing....')
        #     return self.zeroVel()
        robot_pose = self.pre_obj.current_pose
        robot_vel = self.pre_obj.vel
        self.peo_obj = people(people_pose[0], people_pose[1], people_vel[0], people_vel[1], R_PEOPLE)
        # self.peo_obj = people(people_pose[0], people_pose[1], hypot(people_vel[0],people_vel[1])*math.cos(people_pose[2]), hypot(people_vel[0],people_vel[1])*math.sin(people_pose[2]), R_PEOPLE)
        self.rob_obj = robot(robot_pose[0], robot_pose[1], robot_vel[0]*cos(robot_pose[2]), robot_vel[1]*sin(robot_pose[2]),R_ROBOT,self.last_dis_p_r )
        self.rob_obj.calA(self.peo_obj)
        Rel_p = self.rob_obj.calRelPos(self.peo_obj)
        # print('people: ',people_pose, people_vel)
        # print('info: ', Rel_p.dir_, Rel_p.u_star_)

        if Rel_p.u_star_ >= U_MAX-1 or state=="stop" or state=="following":
            self.mode = "open_loop"
        else:
            self.mode = "close_loop"

        # time_duration = 1.0
        time_duration = 0.2
        if self.last_decision_time == None and self.last_mode == None:
            self.last_decision_time = time.time()
            self.last_mode = self.mode
        elif time.time()-self.last_decision_time < time_duration:
            self.mode = self.last_mode
        elif time.time()-self.last_decision_time > time_duration:
            self.last_mode = self.mode
            self.last_decision_time = time.time()

        start_time = time.time()
        opt_rob_x_list = []
        opt_rob_y_list = []
        if self.mode == "open_loop":
            # print ('-----------------------')
            # print('info: ', Rel_p.dir_, Rel_p.u_star_)
            print(self.mode)
            self.open_loop_obj.setVelocity(people_pose[2], hypot(people_vel[0],people_vel[1]))
            self.open_loop_obj.updateSigma(self.nearest_peole_feature_obj.value)
            self.setCostmapForOpenLoop()
            # self.pre_obj.pubMap()
            self.pre_obj.pubPureMap(self.map_for_show)
            # 如果能找到动作，则正常执行
            # try:
            opt_u, opt_dir, opt_cost, opt_rob_x_list, opt_rob_y_list, self.last_dis_p_r = self.rob_obj.calWithMap(self.pre_obj, self.peo_obj, self.close_loop_obj.dir, self.map_for_show)
            self.setLastState(opt_u, opt_dir)
            # except:
            #     cmd_vel = Twist()
            #     return cmd_vel
            self.rob_obj.pubPath(opt_rob_x_list, opt_rob_y_list, 'robot_path')
            if nearest_dis>3.0:
                self.rob_obj.pubMarker([initial_state[0], initial_state[1]],1)
            else:
                self.rob_obj.pubMarker([initial_state[0], initial_state[1]],2)
            # print('output:', opt_u)
            # print('time: ',time.time()-start_time)

        # if self.mode == "close_loop" or self.mode== "open_loop":
        if self.mode == "close_loop":
            # print ('-----------------------')
            # print('info: ', Rel_p.dir_, Rel_p.u_star_)
            print(self.mode)
            print(Rel_p.u_star_)
            self.close_loop_obj.setUStar(Rel_p.u_star_)
            self.close_loop_obj.updateSigmaForPerson(Rel_p.dir_, self.nearest_peole_feature_obj.value)
            person_path_without_collision, robot_path_cost = self.setCostmapForClosedLoop(Rel_p.dir_)
            # self.pre_obj.pubMap()
            self.pre_obj.pubPureMap(self.map_for_show)
            # try:
            opt_u, opt_dir, opt_cost, opt_rob_x_list, opt_rob_y_list, opt_peo_x_list, opt_peo_y_list, self.last_dis_p_r = self.rob_obj.calWithCooperation(person_path_without_collision, self.pre_obj, self.peo_obj, Rel_p.dir_, self.last_u, self.last_dir, self.last_dis_p_r, robot_path_cost, self.close_loop_obj.alpha_d, self.map_for_show)
            self.setLastState(opt_u, opt_dir)
            x_list, y_list = self.rob_obj.getPersonListFromU(self.peo_obj, Rel_p.u_star_, Rel_p.dir_)
            # except:
            #     cmd_vel = Twist()
            #     return cmd_vel            
            self.rob_obj.pubPath(opt_rob_x_list, opt_rob_y_list, 'robot_path')
            self.rob_obj.pubPath(opt_peo_x_list, opt_peo_y_list, 'person_path')
            self.rob_obj.pubMarker([initial_state[0], initial_state[1]],3)
            self.rob_obj.pubPath(x_list, y_list, 'recognized_path')
            # print('time: ',time.time()-start_time)
        
        # if self.mode == "close_loop" and time.time()-self.last_decision_time >0.3:
        #     cmd_vel = mpc_tracer(self.last_opt_x_list, self.last_opt_y_list, initial_state, goal_state)
        #     return cmd_vel
        
        cmd_vel = pid_tracer(opt_rob_x_list, opt_rob_y_list, initial_state, goal_state)
        self.last_opt_x_list = opt_rob_x_list
        self.last_opt_y_list = opt_rob_y_list
        # cmd_vel = mpc_tracer(self.pre_obj.refer_x, self.pre_obj.refer_y, initial_state, goal_state)
        return cmd_vel
        # return self.zeroVel()
        """
        HGPG ended
        """

    def setLastState(self, last_u, last_dir):
        self.last_u = last_u
        self.last_dir = last_dir

    def zeroVel(self):
        cmd_vel = Twist()
        cmd_vel.linear.x=0
        cmd_vel.angular.z=0
        return cmd_vel
    

    def setCostmapForClosedLoop(self, dir):
        person_path_without_collision = []
        self.map_for_show = np.zeros([len(self.pre_obj.col_map),len(self.pre_obj.col_map[0])])
        for u in range(U_MAX):
            x_list, y_list = self.rob_obj.getPersonListFromU(self.peo_obj, u, dir)
            cost = self.close_loop_obj.getCost(u)
            # cost = 100
            # print(u, cost)
            is_collision = False
            for i in range(len(x_list)):
                x_real = x_list[i]
                y_real = y_list[i]
                x_map, y_map =self.pre_obj.world2map(x_real, y_real)
                # print(x_map,y_map,self.open_loop_obj.getCost(theta,dis))
                if x_map<0 or x_map>self.pre_obj.width or y_map<0 or y_map>self.pre_obj.height:
                    continue
                self.map_for_show[int(x_map)][int(y_map)] = min(max(0,cost),255)
                if self.pre_obj.col_map[int(x_map)][int(y_map)] == 200.0:
                    is_collision = True
            if not is_collision or u==U_MAX-1 and len(person_path_without_collision)==0:
                person_path_without_collision.append([x_list, y_list, cost, u])

        robot_path_cost = []
        for u in range(U_MAX):
            cost = self.close_loop_obj.getRobotCost(u)
            robot_path_cost.append(cost)

        # normalized to [0,100]
        # max_value = -1
        # for y in range(self.pre_obj.height):
        #     for x in range(self.pre_obj.width):
        #         max_value = max(self.map_for_show[x][y], max_value)
        for y in range(self.pre_obj.height):
            for x in range(self.pre_obj.width):
                if self.pre_obj.col_map[x][y] == 200.0:
                    self.map_for_show[x][y] = 200.0
                # else:
                #     self.map_for_show[x][y] = self.map_for_show[x][y]/max_value*100.0
        return person_path_without_collision, robot_path_cost

    def setCostmapForOpenLoop(self):
        self.map_for_show = np.zeros([len(self.pre_obj.col_map),len(self.pre_obj.col_map[0])])

        people_pose = self.nearest_peole_feature_obj.getPose()
        robot_pose = self.pre_obj.current_pose
        # relation_dis = np.ceil(hypot(people_pose[0]-robot_pose[0],people_pose[1]-robot_pose[1]))
        relation_dis = int(4)
        for theta in np.linspace(people_pose[2]-np.pi/2.0, people_pose[2]+np.pi/2.0, int(relation_dis*np.pi*2/self.pre_obj.resolution), endpoint=True):
            for dis in range(int(relation_dis/self.pre_obj.resolution)):
                dis = dis*self.pre_obj.resolution
                if abs(dis*sin(theta-people_pose[2]))>0.5:
                    break
                x_real = people_pose[0] + cos(theta)*dis
                y_real = people_pose[1] + sin(theta)*dis
                x_map, y_map =self.pre_obj.world2map(x_real, y_real)
                # print(x_map,y_map,self.open_loop_obj.getCost(theta,dis))
                if x_map<0 or x_map>self.pre_obj.width or y_map<0 or y_map>self.pre_obj.height:
                    continue
                # self.pre_obj.col_map[int(x_map)][int(y_map)] = min(max(0,self.open_loop_obj.getCost(theta,dis)),255)
                self.map_for_show[int(x_map)][int(y_map)] = min(max(0,self.open_loop_obj.getCost(theta,dis)),255)
        # normalized to [0,100]
        # max_value = -1
        # for y in range(self.pre_obj.height):
        #     for x in range(self.pre_obj.width):
        #         max_value = max(self.map_for_show[x][y], max_value)
        for y in range(self.pre_obj.height):
            for x in range(self.pre_obj.width):
                if self.pre_obj.col_map[x][y] == 200.0:
                    self.map_for_show[x][y] = 200.0
                # else:
                #     self.map_for_show[x][y] = self.map_for_show[x][y]/max_value*100.0
    

    # def findNearestPerson(self):
    #     pose_rob = self.pre_obj.current_pose[0:2]
    #     pose_local_goal = self.pre_obj.local_goal
    #     nearest_id = -1
    #     nearest_dis = 99999.0
    #     for subject_id in self.gaze_box.headpose_dir.keys():
    #         A = pose_local_goal[1]-pose_rob[1]
    #         B = pose_rob[0]-pose_local_goal[0]
    #         C = pose_local_goal[0]*pose_rob[1] - pose_local_goal[1]*pose_rob[0]
    #         person_pose = self.gaze_box.headpose_dir[subject_id].getPose()
    #         # print(person_pose)
    #         if person_pose[0] == None:
    #             continue

    #         dis = abs(A*person_pose[0] + B*person_pose[1] + C)/hypot(A,B)
    #         if dis < nearest_dis:
    #             nearest_dis = dis
    #             nearest_id = subject_id
    #     if nearest_id ==-1 or nearest_dis >= 4.0:
    #         return -1
    #     return nearest_id
    

    def findRelatedPerson(self):
        DT = 1.0
        STEP = 5
        rob_list = [self.pre_obj.current_pose[:]]
        for i in range(STEP):
            pose_rob = rob_list[-1]
            rob_list.append([pose_rob[0]+cos(pose_rob[2])*self.pre_obj.vel[0]*DT, pose_rob[1]+sin(pose_rob[2])*self.pre_obj.vel[0]*DT, pose_rob[2]+self.pre_obj.vel[1]*DT])
            # rob_list.append([pose_rob[0]+cos(pose_rob[2])*1.0*DT, pose_rob[1]+sin(pose_rob[2])*1.0*DT, pose_rob[2]+0.0*DT])
        nearest_id = -1
        nearest_dis = 99999.0
        nearest_person_list = []
        for subject_id in self.gaze_box.headpose_dir.keys():
            pose_person = self.gaze_box.headpose_dir[subject_id].getPose()[:2]
            vel_person = self.gaze_box.headpose_dir[subject_id].getVel()[:]
            if pose_person[0] == None or vel_person[0] == None:
                    continue
            person_list = [pose_person]
            for i in range(STEP):
                pose_person = person_list[-1]
                person_list.append([pose_person[0]+vel_person[0]*DT, pose_person[1]+vel_person[1]*DT])
            nearest_dis_temp = min([hypot(rob_list[i][0]-person_list[i][0], rob_list[i][1]-person_list[i][1]) for i in range(1,STEP)])

            if nearest_dis_temp < nearest_dis:
                nearest_dis = nearest_dis_temp
                nearest_id = subject_id
                nearest_person_list = person_list

        nearest_pose_person = self.gaze_box.headpose_dir[nearest_id].getPose()[:2]
        nearest_vel_person = self.gaze_box.headpose_dir[nearest_id].getVel()[:]
        robot_pose = self.pre_obj.current_pose[:]
        # # todo temp, just for abandon unreasonable global path followings
        # if nearest_pose_person[0] - robot_pose[0] >= R_PEOPLE+R_ROBOT+0.2 and nearest_pose_person[0] - robot_pose[0] <=1.2:
        #     return nearest_id, rob_list, nearest_person_list

        # 如果最近的人离得远，舍弃
        if nearest_id ==-1 or (nearest_dis >= 1.2 and self.last_mode==None):
            return -1, rob_list, nearest_person_list, 0, ""
        
        # 仅passing环境
        # 对面来人 且距离较近，就不绕行了
        # if nearest_vel_person[0] * self.pre_obj.vel[0] <= 0:
        if nearest_vel_person[0]<=0 and nearest_pose_person[0] - robot_pose[0] <= R_PEOPLE+R_ROBOT+0.2: 
            return -1, rob_list, nearest_person_list, 0, "oncoming"
        
        # 同方向的人 且距离较远，则也不绕行了
        # if nearest_vel_person[0] * self.pre_obj.vel[0] >= 0:
        #     if hypot(nearest_pose_person[0] - robot_pose[0],nearest_pose_person[1] - robot_pose[1]) >= R_PEOPLE+R_ROBOT+0.5: 
        #         return -1, rob_list, nearest_person_list, 0

        # 最后一维是人的状态
        if hypot(nearest_vel_person[0],nearest_vel_person[1])<=0.05:
            return nearest_id, rob_list, nearest_person_list, nearest_dis, "stop"   # 战立的人，默认为open loop

        if nearest_vel_person[0] > 0:
            if nearest_dis <= R_PEOPLE+R_ROBOT+0.4:
                return nearest_id, rob_list, nearest_person_list, nearest_dis, "following"  # 机器人同方向的人，默认为open loop
            else:
                return -1, rob_list, nearest_person_list, nearest_dis, "following"  # 机器人同方向的人，默认为open loop

        return nearest_id, rob_list, nearest_person_list, nearest_dis, "oncoming"   # 对面来的人，根据pose确定博弈状态


if __name__ == '__main__':
    rospy.init_node('Duel_loop_listener',anonymous=True)
    rospy.Rate(20)  # 1秒20次
    Duel_obj = Duel_loop()
    while not rospy.is_shutdown():
        rospy.spin()
