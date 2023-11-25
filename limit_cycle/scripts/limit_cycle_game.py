#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import hypot
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from limit_cycle.msg import Point_Imp
import matplotlib.pyplot as plt
import numpy as np
import math
from tools import cycle_closewise, Gamma, thetaConstrain
import rospy

from param import U_MAX, R_PEOPLE, R_ROBOT

def plot_circle(center=(3, 3),R=2):        
    x = np.linspace(center[0] - R, center[0] + R, 500)
    y1 = np.sqrt(R**2 - (x-center[0])**2) + center[1]
    y2 = -np.sqrt(R**2 - (x-center[0])**2) + center[1]
    plt.plot(x, y1, c='k')
    plt.plot(x, y2, c='k')

# def transform(origin, state):
#     # x,y,z
#     T = np.zeros([3,3])
#     T[0][3] = origin.x
#     T[1][3] = origin.y
#     T[2][3] = 0
#     T[3][3] = 1
#     T[2][2] = 1
#     T[0][0] = origin.vx/np.sqrt(origin.vx*2+origin.vy**2)
#     T[1][1] = origin.vx/np.sqrt(origin.vx*2+origin.vy**2)
#     T[0][1] = origin.vy/np.sqrt(origin.vx*2+origin.vy**2)
#     T[1][0] = -origin.vy/np.sqrt(origin.vx*2+origin.vy**2)
#     return np.dot(T,state)

class people:
    def __init__(self,x,y,vx,vy,r):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.r = r
        self.u_star_ = None
        self.diff_ = None
        self.dir_ = None

class robot:
    def __init__(self,x=None,y=None,vx=None,vy=None,r=None,last_dis_p_r=None):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.r = r
        self.last_dis_p_r = last_dis_p_r


    def calRelPos(self,p):
        RelP=people(p.x-self.x,p.y-self.y,p.vx-self.vx,p.vy-self.vy,p.r+self.r)
        Rel_position_theta=thetaConstrain(math.atan2(-RelP.y,-RelP.x))
        Rel_orientation_theta=thetaConstrain(math.atan2(RelP.vy,RelP.vx))
        if Rel_orientation_theta < Rel_position_theta:
            dir = -1 #逆时针
        elif Rel_orientation_theta > Rel_position_theta:
            dir = 1 #顺时针
        else:
            dir= 0
        
        # print(RelP.y, RelP.x)
        # print(RelP.vy, RelP.vx)
        
        if dir==0:
            # print('error')
            # exit()
            RelP.u_star_ = 9999.9
            RelP.dir_ = 0
            return RelP
        
        u_star = None
        diff = None
        test_temp = []
        for u in range(1,U_MAX):
            x=RelP.x
            y=RelP.y
            dx,dy=cycle_closewise(x,y,RelP.r,dir,Gamma(u*self.a))
            theta = thetaConstrain(math.atan2(dy,dx))
            test_temp.append([u,theta*180/np.pi])
            temp_diff = abs(theta - Rel_orientation_theta)
            # if diff!= None:
            #     print(diff, temp_diff)
            if diff==None or temp_diff<diff:
                diff = temp_diff
                u_star = u

        RelP.u_star_ = u_star
        RelP.diff_ = diff
        RelP.dir_ = dir

        # print(Rel_position_theta, Rel_orientation_theta, dir)
        # print(test_temp)
        # print("u*: ",u_star,diff*180/np.pi)

        return RelP
    
    def calTheta(self, u, p, dir):
        dx,dy=cycle_closewise(p.x-self.x,p.y-self.y,p.r+self.r,dir,Gamma(u*self.a))
        theta = thetaConstrain(math.atan2(dy,dx)*180/np.pi)
        return theta

    def getRobotListFromU(self, p, u, dir, resolution=0.05):
        RelP=people(self.x-p.x, self.y-p.y, self.vx, self.vy,p.r+self.r)
        x = RelP.x
        y = RelP.y
        x_list = [x+p.x]
        y_list = [y+p.y]
        param = Gamma(u*self.a)
        for i in range(int(8/resolution)):
            dx,dy=cycle_closewise(x, y, RelP.r, dir, param)
            x+=dx/(np.sqrt(dx**2+dy**2))*resolution
            y+=dy/(np.sqrt(dx**2+dy**2))*resolution
            x_list.append(x+p.x)
            y_list.append(y+p.y)
            if RelP.x*x +RelP.y*y<0:
                break
        Rel_orientation_theta = thetaConstrain(math.atan2(RelP.vy,RelP.vx))
        Real_orientation_theta = thetaConstrain(math.atan2(self.vy,self.vx))
        diff_theta = Rel_orientation_theta-Real_orientation_theta
        # diff_theta=0
        for i in range(len(x_list)):
            x_list[i],y_list[i] = (x_list[i]-self.x)*math.cos(diff_theta)+(y_list[i]-self.y)*math.sin(diff_theta)+self.x, -(x_list[i]-self.x)*math.sin(diff_theta)+(y_list[i]-self.y)*math.cos(diff_theta)+self.y
        return x_list, y_list

    def calWithMap(self, map_obj, p, last_dir, map_for_show):
        RelP=people(self.x-p.x, self.y-p.y, self.vx, self.vy,p.r+self.r)      
        goal = map_obj.arb_local_goal

        W_goal = 1.0
        W_map = 1.0

        map_cost = []
        goal_cost = []
        final_cost = []
        recorded_dir_list = []
        recorded_u_list = []

        if last_dir == None:
            dir_list = [1,-1]
        else:
            dir_list = [last_dir]
        for dir in dir_list:
            for u in range(U_MAX):
                x = RelP.x
                y = RelP.y
                ave_cost = 0.0
                x_list = [x+p.x]
                y_list = [y+p.y]
                param = Gamma(u*self.a)
                for i in range(80):
                    dx,dy=cycle_closewise(x, y, RelP.r, dir, param)
                    x+=dx/(np.sqrt(dx**2+dy**2))*0.05
                    y+=dy/(np.sqrt(dx**2+dy**2))*0.05
                    x_list.append(x+p.x)
                    y_list.append(y+p.y)
                    if RelP.x*x +RelP.y*y<0:
                        break
                    temp = map_obj.getCostwithArbMap(x+p.x,y+p.y,map_for_show)
                    if temp == 200.0:
                        ave_cost = 9999.9
                        break
                    ave_cost = (ave_cost*i +temp)*1.0/((i+1)*1.0)
                    # ave_cost = max(ave_cost, temp)

                recorded_dir_list.append(dir)
                recorded_u_list.append(u)
                map_cost.append(ave_cost)
                goal_cost.append(abs(goal[1]-y_list[-1]))        
        # reform goal_cost
        min_goal_cost = min(goal_cost)
        for i in range(len(goal_cost)):
            goal_cost[i] = goal_cost[i]-min_goal_cost
        max_goal_cost = max(goal_cost)
        for i in range(len(goal_cost)):
            goal_cost[i] /= max_goal_cost
        for i in range(len(goal_cost)):
            goal_cost[i] = 4.2**goal_cost[i]
            # goal_cost[i] = goal_cost[i]

        for i in range(len(goal_cost)):
            final_cost.append(W_map*map_cost[i] + W_goal*goal_cost[i])

        opt_cost = 999999.9
        min_index = -1
        for i,cost in enumerate(final_cost):
            if cost<opt_cost:
                min_index = i
                opt_cost = cost
        opt_u = recorded_u_list[min_index]
        opt_dir = recorded_dir_list[min_index]

        x = RelP.x
        y = RelP.y
        opt_x_list = [x+p.x]
        opt_y_list = [y+p.y]
        param = Gamma(opt_u*self.a)
        for i in range(80):
            dx,dy=cycle_closewise(x, y, RelP.r, opt_dir, param)
            x+=dx/(np.sqrt(dx**2+dy**2))*0.2
            y+=dy/(np.sqrt(dx**2+dy**2))*0.2
            opt_x_list.append(x+p.x)
            opt_y_list.append(y+p.y)
            if RelP.x*x +RelP.y*y<0:
                break    

        x = RelP.x
        y = RelP.y
        dis_x_list = [x+p.x]
        dis_y_list = [y+p.y]
        param = Gamma(opt_u*self.a)
        for i in range(400):
            dx,dy=cycle_closewise(x, y, RelP.r, opt_dir, param)
            x+=dx/(np.sqrt(dx**2+dy**2))*0.02
            y+=dy/(np.sqrt(dx**2+dy**2))*0.02
            dis_x_list.append(x+p.x)
            dis_y_list.append(y+p.y)
            if RelP.x*x +RelP.y*y<0:
                break   

        last_dis_p_r = self.calDis([p.x,p.y], [dis_x_list, dis_y_list])
        
        # print('*'*3)
        # print(map_cost)
        # print(goal_cost)
        # print('*'*3)
        print(opt_u)
        return opt_u, opt_dir, opt_cost, opt_x_list, opt_y_list, last_dis_p_r


    # def calWithCooperation(self, person_list_all, map_obj, p, robot_dir, last_u, last_dir, last_dis_p_r, robot_path_cost, alpha_d, map_for_show):
    #     RelP=people(self.x-p.x, self.y-p.y, self.vx, self.vy,p.r+self.r)
    #     goal = map_obj.arb_local_goal
    #     robot_list_all = []
    #     for u in range(U_MAX):
    #         x = RelP.x
    #         y = RelP.y
    #         x_list = [x+p.x]
    #         y_list = [y+p.y]
    #         param = Gamma(u*self.a)
    #         is_collision = False
    #         for i in range(80):
    #             dx,dy=cycle_closewise(x, y, RelP.r, robot_dir, param)
    #             x+=dx/(np.sqrt(dx**2+dy**2))*0.05
    #             y+=dy/(np.sqrt(dx**2+dy**2))*0.05
    #             x_list.append(x+p.x)
    #             y_list.append(y+p.y)
    #             if x>=0:
    #                 break
    #             if map_obj.getCostwithArbMap(x+p.x, y+p.y, map_for_show) == 200.0:
    #                 is_collision = True
    #                 break
    #         if not is_collision:
    #             robot_list_all.append([x_list, y_list, u, None, None])

    #     middle_x = (self.x+p.x)/2.0
    #     for i in range(len(robot_list_all)):
    #         x_list = robot_list_all[i][0]
    #         y_list = robot_list_all[i][1]
    #         for j in range(len(x_list)-1):
    #             if x_list[j]<middle_x and x_list[j+1]>middle_x:
    #                 x1 = x_list[j]
    #                 y1 = y_list[j]
    #                 x2 = x_list[j+1]
    #                 y2 = y_list[j+1]
    #                 robot_list_all[i][3] = (y1*x2-x1*y2+middle_x*(y2-y1))/(x2-x1)
    #                 break
    #     person_list_all_y = []
    #     for i in range(len(person_list_all)):
    #         x_list = person_list_all[i][0]
    #         y_list = person_list_all[i][1]
    #         for j in range(len(x_list)-1):
    #             if x_list[j]>middle_x and x_list[j+1]<middle_x:
    #                 x1 = x_list[j]
    #                 y1 = y_list[j]
    #                 x2 = x_list[j+1]
    #                 y2 = y_list[j+1]
    #                 person_list_all_y.append((y1*x2-x1*y2+middle_x*(y2-y1))/(x2-x1))
    #                 break

    #     opt_u = -1
    #     opt_dir = robot_dir
    #     opt_cost = 9999999999999.9
    #     # cost_parts = []
    #     opt_rob_x_list = []
    #     opt_rob_y_list = []
    #     opt_peo_x_list = []
    #     opt_peo_y_list = []

    #     recorded_up = []
    #     recorded_ur = []
    #     recorded_per = []
    #     recorded_go = []
    #     recorded_dis = []


    #     W_desire = 1.0
    #     W_goal = 15.0
    #     W_dis = 10.0

    #     for rob_list in robot_list_all:
    #         x_list = rob_list[0]
    #         y_list = rob_list[1]
    #         current_u = rob_list[2]

    #         for j, person_list in enumerate(person_list_all):
    #             c1 = -person_list[2]
    #             c2 = abs(rob_list[3]-goal[1])
    #             c3 = -abs(rob_list[3]-person_list_all_y[j])

    #             combined_cost = W_desire*c1 + W_goal*c2 + W_dis*alpha_d*c3
    #             recorded_up.append(person_list[3])
    #             recorded_ur.append(current_u)
    #             recorded_per.append(c1)
    #             recorded_go.append(c2)
    #             recorded_dis.append(c3*alpha_d)

    #             if combined_cost < opt_cost:
    #                 opt_cost = combined_cost
    #                 opt_u = current_u
    #                 opt_rob_x_list = x_list
    #                 opt_rob_y_list = y_list
    #                 opt_peo_x_list = person_list[0]
    #                 opt_peo_y_list = person_list[1]
    #     last_dis_p_r = self.calDis([p.x, p.y], [opt_rob_x_list, opt_rob_y_list])


    #     print('*'*3)
    #     print(recorded_up)
    #     print(recorded_ur)
    #     print(recorded_per)
    #     print(recorded_go)
    #     print(recorded_dis)
    #     print(W_desire,W_goal, W_dis)
    #     print('*'*3)
    #     print(opt_u)
        
    #     # if opt_u -last_u > 3:
    #     #     for i in range(len(robot_list_all)):
    #     #         index = len(robot_list_all)-1-i
    #     #         if robot_list_all[index][2] - last_u <=3:
    #     #             opt_u = robot_list_all[index][2]
    #     #             opt_rob_x_list = robot_list_all[index][0]
    #     #             opt_rob_y_list = robot_list_all[index][1]
    #     #             break
    #     #         if robot_list_all[index][2]-last_u<=0:
    #     #             break

    #     # if opt_u -last_u < -3:
    #     #     for index in range(len(robot_list_all)):
    #     #         if robot_list_all[index][2] - last_u >= -3:
    #     #             opt_u = robot_list_all[index][2]
    #     #             opt_rob_x_list = robot_list_all[index][0]
    #     #             opt_rob_y_list = robot_list_all[index][1]
    #     #             break
    #     #         if robot_list_all[index][2] - last_u >= 0:
    #     #             break
    #     # print(opt_u)


    #     return opt_u, opt_dir, opt_cost, opt_rob_x_list, opt_rob_y_list, opt_peo_x_list, opt_peo_y_list, last_dis_p_r

    def calWithCooperation(self, person_list_all, map_obj, p, robot_dir, last_u, last_dir, last_dis_p_r, robot_path_cost, alpha_d, map_for_show):
        RelP=people(self.x-p.x, self.y-p.y, self.vx, self.vy,p.r+self.r)
        goal = map_obj.arb_local_goal
        robot_list_all = []
        for u in range(U_MAX):
            x = RelP.x
            y = RelP.y
            x_list = [x+p.x]
            y_list = [y+p.y]
            param = Gamma(u*self.a)
            is_collision = False
            for i in range(80):
                dx,dy=cycle_closewise(x, y, RelP.r, robot_dir, param)
                x+=dx/(np.sqrt(dx**2+dy**2))*0.05
                y+=dy/(np.sqrt(dx**2+dy**2))*0.05
                x_list.append(x+p.x)
                y_list.append(y+p.y)
                # 防止机器人无限制地远离
                if abs(y+p.y-10)>=0.8:
                    is_collision = True
                if x>=0:
                    break
                if map_obj.getCostwithArbMap(x+p.x, y+p.y, map_for_show) == 200.0:
                    is_collision = True
                    break
            if not is_collision:
                robot_list_all.append([x_list, y_list, u, None, None])
            

        middle_x = (self.x+p.x)/2.0
        for i in range(len(robot_list_all)):
            x_list = robot_list_all[i][0]
            y_list = robot_list_all[i][1]
            for j in range(len(x_list)-1):
                if x_list[j]<middle_x and x_list[j+1]>middle_x:
                    x1 = x_list[j]
                    y1 = y_list[j]
                    x2 = x_list[j+1]
                    y2 = y_list[j+1]
                    robot_list_all[i][3] = (y1*x2-x1*y2+middle_x*(y2-y1))/(x2-x1)
                    break
        
        person_list_all_y = []
        for i in range(len(person_list_all)):
            x_list = person_list_all[i][0]
            y_list = person_list_all[i][1]
            for j in range(len(x_list)-1):
                if x_list[j]>middle_x and x_list[j+1]<middle_x:
                    x1 = x_list[j]
                    y1 = y_list[j]
                    x2 = x_list[j+1]
                    y2 = y_list[j+1]
                    person_list_all_y.append((y1*x2-x1*y2+middle_x*(y2-y1))/(x2-x1))
                    break

        opt_u = -1
        opt_dir = robot_dir
        opt_cost = 999999.9
        # cost_parts = []
        opt_rob_x_list = []
        opt_rob_y_list = []
        opt_peo_x_list = []
        opt_peo_y_list = []

        recorded_up = []
        recorded_ur = []
        recorded_per = []
        recorded_go = []
        recorded_dis = []


        W_desire = 8.0
        W_goal = 13.0
        W_dis = 5.0

        for rob_list in robot_list_all:
            x_list = rob_list[0]
            y_list = rob_list[1]
            current_u = rob_list[2]

            for j, person_list in enumerate(person_list_all):
                c1 = -person_list[2]
                # print(111,rob_list)
                # print(222,goal)
                c2 = abs(rob_list[3]-goal[1])
                c3 = -abs(rob_list[3]-person_list_all_y[j])

                recorded_up.append(person_list[3])
                recorded_ur.append(current_u)
                recorded_per.append(c1)
                recorded_go.append(c2)
                recorded_dis.append(c3*alpha_d)

        min_go = min(recorded_go)
        for i in range(len(recorded_go)):
            recorded_go[i] = math.e**(recorded_go[i]-min_go)

        for i in range(len(robot_list_all)):
            for j in range(len(person_list_all)):
                index = j+i*len(person_list_all)
                combined_cost =  W_desire*recorded_per[index] + W_goal*recorded_go[index] + W_dis*alpha_d*recorded_dis[index]
                if combined_cost < opt_cost:
                    opt_cost = combined_cost
                    opt_u = robot_list_all[i][2]
                    opt_rob_x_list = robot_list_all[i][0]
                    opt_rob_y_list = robot_list_all[i][1]
                    opt_peo_x_list = person_list_all[j][0]
                    opt_peo_y_list = person_list_all[j][1]
        last_dis_p_r = self.calDis([p.x, p.y], [opt_rob_x_list, opt_rob_y_list])


        # print('*'*3)
        # print(recorded_up)
        # print(recorded_ur)
        # print(recorded_per)
        # print(recorded_go)
        # print(recorded_dis)
        # print(W_desire,W_goal, W_dis)
        # print('*'*3)
        print(opt_u)
        
        # if opt_u -last_u > 3:
        #     for i in range(len(robot_list_all)):
        #         index = len(robot_list_all)-1-i
        #         if robot_list_all[index][2] - last_u <=3:
        #             opt_u = robot_list_all[index][2]
        #             opt_rob_x_list = robot_list_all[index][0]
        #             opt_rob_y_list = robot_list_all[index][1]
        #             break
        #         if robot_list_all[index][2]-last_u<=0:
        #             break

        # if opt_u -last_u < -3:
        #     for index in range(len(robot_list_all)):
        #         if robot_list_all[index][2] - last_u >= -3:
        #             opt_u = robot_list_all[index][2]
        #             opt_rob_x_list = robot_list_all[index][0]
        #             opt_rob_y_list = robot_list_all[index][1]
        #             break
        #         if robot_list_all[index][2] - last_u >= 0:
        #             break
        # print(opt_u)


        return opt_u, opt_dir, opt_cost, opt_rob_x_list, opt_rob_y_list, opt_peo_x_list, opt_peo_y_list, last_dis_p_r


    def calDis(self,person_list, robot_list):
        min_dis = 9999.9
        min_lateral = 9999.9
        for i in range(len(robot_list[0])):
            temp_dis = abs(person_list[0]-robot_list[0][i])
            if temp_dis < min_dis:
                min_dis = temp_dis
                min_lateral = min(min_lateral, abs(person_list[1]-robot_list[1][i]))
        return min_lateral

    # def calDis(self,person_list, robot_list):
    #     y_p = person_list[1][0]
    #     y_r = robot_list[1][-1]
    #     return abs(y_p-y_r)

    def calA(self,p):
        RelP=people(p.x-self.x,p.y-self.y,p.vx-self.vx,p.vy-self.vy,p.r+self.r)
        Rel_theta= thetaConstrain(math.atan2(-RelP.y,-RelP.x))
        
        theta2tangency = math.asin(RelP.r*1.0/math.hypot(RelP.x,RelP.y))
        # print(Rel_theta, theta2tangency)
        # print('theta real:',(Rel_theta-theta2tangency)*180.0/np.pi, (Rel_theta+theta2tangency)*180.0/np.pi)
        Rel_theta += 0.6* theta2tangency    # 1.0是因子，表征往相切的角度，偏离多少
        Rel_theta = thetaConstrain(Rel_theta) # for 顺时针，dir=1

        # y'/x'=tan(Rel_theta)
        dir = 1
        fenzi = math.tan(Rel_theta)*RelP.y*dir +dir*RelP.x
        fenmu = (RelP.r**2-RelP.x**2-RelP.y**2)*(RelP.y-math.tan(Rel_theta)*RelP.x)
        Gamma_desired = fenzi/fenmu
        # print('desired: ',Gamma_desired)

        a_fitted = -1
        diff_min = 1000000.0
        for a in range(1,11):
            a = a/100.0
            # print(a, abs(Gamma(U_MAX*a)))
            diff = abs(Gamma(U_MAX*a)-Gamma_desired)
            if a_fitted==-1 or diff<diff_min:
                a_fitted = a
                diff_min = diff
        # print(a_fitted)
        self.a = a_fitted
        return a_fitted

    def pubPath(self, x_list, y_list, topic_name='robot_path'):
        gui_path = Path()
        gui_path.header.frame_id = 'map'
        gui_path.header.stamp = rospy.Time.now()
        for i in range(len(x_list)):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = rospy.Time.now()
            p.pose.position.x = x_list[i]
            p.pose.position.y = y_list[i]
            p.pose.position.z = 0.0
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0
            gui_path.poses.append(p)
        pub_path = rospy.Publisher(topic_name, Path, queue_size=5)
        pub_path.publish(gui_path)
        # return gui_path
    
    def pubMarker(self, point, flag):
        imp_obj = Point_Imp()
        pose_obj = Pose()
        pose_obj.position.x = point[0]
        pose_obj.position.y = point[1]
        imp_obj.pose = pose_obj
        imp_obj.flag = flag
        pub_path = rospy.Publisher('flag', Point_Imp, queue_size=5)
        pub_path.publish(imp_obj)



    def drawRel(self, RelP):
        plot_circle((0, 0), RelP.r)
        # plt.scatter(1,1)
        x = RelP.x
        y = RelP.y
        x_list = [x]
        y_list = [y]
        for i in range(80):
            dx,dy=cycle_closewise(x,y,RelP.r,RelP.dir_,Gamma(RelP.u_star_*self.a))
            x+=dx/(np.sqrt(dx**2+dy**2))*0.1
            y+=dy/(np.sqrt(dx**2+dy**2))*0.1
            if RelP.x*x +RelP.y*y<0:
                break
            x_list.append(x)
            y_list.append(y)

        plt.plot(x_list,y_list,'blue')
        plt.plot([RelP.x, RelP.x+RelP.vx/np.sqrt(RelP.vx**2+RelP.vy**2)*2],[RelP.y, RelP.y+RelP.vy/np.sqrt(RelP.vx**2+RelP.vy**2)*2],'red')

        # optional: show all candidate trajectories 
        for u in range(1,U_MAX):
            if u==RelP.u_star_:
                continue
            x = RelP.x
            y = RelP.y
            x_list = [x]
            y_list = [y]
            for i in range(80):
                dx,dy=cycle_closewise(x,y,RelP.r,RelP.dir_,Gamma(u*self.a))
                x+=dx/(np.sqrt(dx**2+dy**2))*0.1
                y+=dy/(np.sqrt(dx**2+dy**2))*0.1
                if RelP.x*x +RelP.y*y<0:
                    break
                x_list.append(x)
                y_list.append(y)
            plt.plot(x_list,y_list,'green')
        # show opposite bound
        u = U_MAX-1
        x = RelP.x
        y = RelP.y
        x_list = [x]
        y_list = [y]
        for i in range(80):
            dx,dy=cycle_closewise(x,y,RelP.r,-RelP.dir_,Gamma(u*self.a))
            x+=dx/(np.sqrt(dx**2+dy**2))*0.1
            y+=dy/(np.sqrt(dx**2+dy**2))*0.1
            if RelP.x*x +RelP.y*y<0:
                break
            x_list.append(x)
            y_list.append(y)
        plt.plot(x_list,y_list,'green')
        # optional end

    def getPersonListFromU(self, p, u, dir):
        RelP=people(p.x-self.x,p.y-self.y,p.vx-self.vx,p.vy-self.vy,p.r+self.r)
        x = RelP.x
        y = RelP.y
        x_list = [x+self.x]
        y_list = [y+self.y]
        param = Gamma(u*self.a)
        for i in range(80):
            dx,dy=cycle_closewise(x, y, RelP.r, dir, param)
            x+=dx/(np.sqrt(dx**2+dy**2))*0.05
            y+=dy/(np.sqrt(dx**2+dy**2))*0.05
            x_list.append(x+self.x)
            y_list.append(y+self.y)
            if RelP.x*x +RelP.y*y<0:
                break
        Rel_orientation_theta = thetaConstrain(math.atan2(RelP.vy,RelP.vx))
        Real_orientation_theta = thetaConstrain(math.atan2(p.vy,p.vx))
        diff_theta = Rel_orientation_theta-Real_orientation_theta
        # diff_theta=0
        for i in range(len(x_list)):
            x_list[i],y_list[i] = (x_list[i]-p.x)*math.cos(diff_theta)+(y_list[i]-p.y)*math.sin(diff_theta)+p.x, -(x_list[i]-p.x)*math.sin(diff_theta)+(y_list[i]-p.y)*math.cos(diff_theta)+p.y
        return x_list, y_list

    def getRobotListFromU(self, p, u, dir, resolution=0.05):
        RelP=people(self.x-p.x, self.y-p.y, self.vx, self.vy,p.r+self.r)
        x = RelP.x
        y = RelP.y
        x_list = [x+p.x]
        y_list = [y+p.y]
        param = Gamma(u*self.a)
        for i in range(int(8/resolution)):
            dx,dy=cycle_closewise(x, y, RelP.r, dir, param)
            x+=dx/(np.sqrt(dx**2+dy**2))*resolution
            y+=dy/(np.sqrt(dx**2+dy**2))*resolution
            x_list.append(x+p.x)
            y_list.append(y+p.y)
            if RelP.x*x +RelP.y*y<0:
                break
        Rel_orientation_theta = thetaConstrain(math.atan2(RelP.vy,RelP.vx))
        Real_orientation_theta = thetaConstrain(math.atan2(self.vy,self.vx))
        diff_theta = Rel_orientation_theta-Real_orientation_theta
        # diff_theta=0
        for i in range(len(x_list)):
            x_list[i],y_list[i] = (x_list[i]-self.x)*math.cos(diff_theta)+(y_list[i]-self.y)*math.sin(diff_theta)+self.x, -(x_list[i]-self.x)*math.sin(diff_theta)+(y_list[i]-self.y)*math.cos(diff_theta)+self.y
        return x_list, y_list

    
    def drawReal(self, P, RelP):
        plot_circle((self.x, self.y), self.r)
        plt.plot([self.x,self.x+self.vx/np.sqrt(self.vx**2+self.vy**2)],[self.y,self.y+self.vy/np.sqrt(self.vx**2+self.vy**2)],'red')
        plot_circle((P.x, P.y), P.r)

        # plt.scatter(1,1)
        x = RelP.x
        y = RelP.y
        x_list = [x+self.x]
        y_list = [y+self.y]
        for i in range(80):
            dx,dy=cycle_closewise(x,y,RelP.r,RelP.dir_,Gamma(RelP.u_star_*self.a))
            x+=dx/(np.sqrt(dx**2+dy**2))*0.1
            y+=dy/(np.sqrt(dx**2+dy**2))*0.1
            if RelP.x*x +RelP.y*y<0:
                break
            x_list.append(x+self.x)
            y_list.append(y+self.y)

        Rel_orientation_theta = thetaConstrain(math.atan2(RelP.vy,RelP.vx))
        Real_orientation_theta = thetaConstrain(math.atan2(P.vy,P.vx))
        diff_theta = Rel_orientation_theta-Real_orientation_theta
        # diff_theta=0
        for i in range(len(x_list)):
            x_list[i],y_list[i] = (x_list[i]-P.x)*math.cos(diff_theta)+(y_list[i]-P.y)*math.sin(diff_theta)+P.x, -(x_list[i]-P.x)*math.sin(diff_theta)+(y_list[i]-P.y)*math.cos(diff_theta)+P.y


        plt.plot(x_list,y_list,'blue')
        plt.plot([P.x, P.x+ P.vx/np.sqrt((P.vx**2+P.vy**2))],[P.y, P.y+P.vy/np.sqrt((P.vx**2+P.vy**2))],'red')



if __name__ == '__main__':
    # x_p = [4.0,4.0]
    # x_r = [1,1]
    # v_p = [-1.0,-1.3]
    # v_r = [0.0,0.0]

    x_p = [0.0,0.0]
    x_r = [4.0,0.0]
    v_p = [2.0,0.01]
    v_r = [-1.0,0.0]

    p1 = people(x_p[0],x_p[1],v_p[0],v_p[1],0.3)
    rob = robot(x_r[0],x_r[1],v_r[0],v_r[1],0.3)
    Rel_p1 = rob.calRelPos(p1)
    print('theta range:', rob.calTheta(U_MAX, p1, 1), rob.calTheta(U_MAX, p1, -1))
    rob.drawRel(Rel_p1)
    plt.axis('equal')
    plt.show()

    rob.drawReal(p1, Rel_p1)
    plt.axis('equal')
    plt.show()

    for t in range(40):
        time = t*0.1
        p1 = people(x_p[0]+time*v_p[0],x_p[1]+time*v_p[1],v_p[0],v_p[1],0.5)
        rob = robot(x_r[0]+time*v_r[0],x_r[1]+time*v_r[1],v_r[0],v_r[1],0.5)
        Rel_p1 = rob.calRelPos(p1)
        rob.drawReal(p1, Rel_p1)
        plt.axis('equal')
        plt.show()  
