#!/usr/bin/env python
import pygame
import rospy
import math
from geometry_msgs.msg import Twist
import robot_teleop

class Person:
    def __init__(self, win_obj, id):
        self.id = id
        self.win_obj = win_obj
        self.key_pressed = [0,0,0,0,0,0]  #[move,move,gaze,gaze,gaze,gazemode]
        
    def set_robot_teleop(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_tel = robot_teleop.Robot()

    def draw_obj(self, location):
        (row, col) = (location[0]*WIN_OBJ_H, location[1]*WIN_OBJ_W)
        pygame.draw.lines(self.win_obj,(128,138,135),True,((col+2,row+2),(col+198,row+2),(col+198,row+198),(col+2,row+198)),1)
        position_rec = [(col+75,row+45),(col+75,row+145),(col+25,row+95),(col+125,row+95),((col+75,row+95))]
        font_key = pygame.font.SysFont('calibri',40)    ##(.ttf)
        font_id = pygame.font.SysFont('calibri',20)
        for i in range(5):
            key = font_key.render(self.keyboard[i],True,(41,36,33))
            pygame.draw.rect(self.win_obj,(220,220,220),(position_rec[i][0],position_rec[i][1],50,50))
            self.win_obj.blit(key,(position_rec[i][0]+15,position_rec[i][1]+8))
        if self.id == 'robot':
            id = font_id.render('ID: robot',True,(41,36,33))
            self.win_obj.blit(id,(position_rec[0][0]-13,position_rec[0][1]-30))
        else:
            id = font_id.render('ID: {}'.format(self.id),True,(41,36,33))
            self.win_obj.blit(id,(position_rec[0][0],position_rec[0][1]-30))
        pygame.display.update()

    
    def set_keyboard(self,keyboard):
        self.keyboard = keyboard
        self.ww = keyboard[0]
        self.xx = keyboard[1]
        self.aa = keyboard[2]
        self.dd = keyboard[3]
        self.ss = keyboard[4]

    def set_gazekeyboard(self,keyboard):
        self.gazekeyboard = keyboard
        self.guu=keyboard[0]
        self.gdd=keyboard[1]

        self.gll=keyboard[2]
        self.grr=keyboard[3]

        self.gzrise=keyboard[4]
        self.gzfall=keyboard[5]

        self.gii=keyboard[6]

    def set_publisher(self,topic):
        self.pub = rospy.Publisher(topic, Twist, queue_size=10)
    
    def set_gazepublisher(self,topic):
        self.gaze_pub=rospy.Publisher(topic,Twist,queue_size=10)


    def handle_key(self):
        key_msg = Twist()
        if self.key_pressed[0] == 0:
            key_msg.linear.x = 0
        if self.key_pressed[1] == 0:
            key_msg.angular.z = 0
        if self.key_pressed[0] == self.ww:
            key_msg.linear.x = 1
        if self.key_pressed[0] == self.xx:
            key_msg.linear.x = -1
        if self.key_pressed[0] == self.ss:
            key_msg.linear.x = -2
            key_msg.angular.z = -2
        if self.key_pressed[1] == self.aa:
            key_msg.angular.z = 1
        if self.key_pressed[1] == self.dd:
            key_msg.angular.z = -1
        return key_msg

    def handle_gaze_key(self):
        gazekey_msg=Twist()
        if self.key_pressed[2]==0:
            gazekey_msg.angular.y=0

        if self.key_pressed[3]==0:
            gazekey_msg.angular.z=0
        
        if self.key_pressed[2]==self.guu:
            gazekey_msg.angular.y=1

        if self.key_pressed[2]==self.gdd:
            gazekey_msg.angular.y=-1

        if self.key_pressed[3]==self.gll:
            gazekey_msg.angular.z=-1
        
        if self.key_pressed[3]==self.grr:
            gazekey_msg.angular.z=1

        if self.key_pressed[4]==self.gzrise:
            gazekey_msg.angular.x=1
        
        if self.key_pressed[4]==self.gzfall:
            gazekey_msg.angular.x=-1
        
        # use to control the mode of the keyboard
        if self.key_pressed[5]==0:
            gazekey_msg.linear.x=0
        elif self.key_pressed[5]==1:
            gazekey_msg.linear.x=1
        elif self.key_pressed[5]==2:
            gazekey_msg.linear.x=2

        return gazekey_msg


    def key_down(self, key):
        if key == self.ww or key == self.ss or key == self.xx:
            self.key_pressed[0] = key
        if key == self.aa or key == self.dd:
            self.key_pressed[1] = key
        
        # deal with gaze
        if key == self.guu or key==self.gdd:
            self.key_pressed[2]=key
        if key == self.gll or key==self.grr:
            self.key_pressed[3]=key

        if key==self.gzrise or key==self.gzfall:
            self.key_pressed[4]=key
        
        if key == self.gii and self.key_pressed[5]==0:
            self.key_pressed[5]=1
        elif key== self.gii and self.key_pressed[5]==1:
            self.key_pressed[5]=2
        elif key==self.gii and self.key_pressed[5]==2:
            self.key_pressed[5]=0

    def key_up(self, key):
        if key == self.key_pressed[0]:
            self.key_pressed[0] = 0
        if key == self.key_pressed[1]:
            self.key_pressed[1] = 0 

        if key == self.key_pressed[2]:
            self.key_pressed[2] = 0
        if key == self.key_pressed[3]:
            self.key_pressed[3] = 0

        if key==self.key_pressed[4]:
            self.key_pressed[4]=0


class InterfaceSystem:
    def __init__(self,n):    
        pygame.init()
        row_num = math.ceil(n/MAX_LINE_NUM)*WIN_OBJ_H +2
        col_num = min(n,MAX_LINE_NUM)*WIN_OBJ_W +2
        self.win_obj = pygame.display.set_mode((col_num,row_num))      
        pygame.display.set_caption('Interface to Manually Control the Objects')   
        self.win_obj.fill((255,255,255))   
        pygame.display.flip()
        
    def getWin(self):
        return self.win_obj


WIN_OBJ_H = 199
WIN_OBJ_W = 199
MAX_LINE_NUM = 6.0
keyboard_list = [['w','x','a','d','s'],['t','b','f','h','g'],['i','m','j','l','k']]
topic_list = ["/people{}/keyboard".format(i) for i in range(len(keyboard_list))]

gazekeyboard_list=[['8','2','4','6','9','3','5'],['8','2','4','6','9','3','5'],['8','2','4','6','9','3','5']]

#[pygame.K_UP,pygame.K_DOWN,pygame.K_LEFT,pygame.K_RIGHT,pygame.K_RSHIFT]
gazetopic_list=["/people{}/keyboard/gaze".format(i) for i in range(len(gazekeyboard_list))]



n = len(keyboard_list)

if __name__=="__main__":
    rospy.init_node('interface_keyboard_publisher', anonymous=True)
    person_mode = rospy.get_param('/pedsim_simulator/person_mode')
    robot_mode = rospy.get_param('/pedsim_simulator/robot_mode')
    unit_list = []
    
    if person_mode != 2 and robot_mode!=2:
        exit(0)
    elif person_mode!=2 and robot_mode==2:
        n=1
        i=0
        env = InterfaceSystem(n)
        location = (math.floor(i/MAX_LINE_NUM),math.ceil(i%MAX_LINE_NUM))
        obj = Person(env.getWin(),'robot')
        obj.set_keyboard(keyboard_list[i])
        obj.set_gazekeyboard(gazekeyboard_list[i])
        obj.set_robot_teleop()
        obj.draw_obj(location)
        unit_list.append(obj)
    elif person_mode==2 and robot_mode!=2:
        # initialize interface system    
        env = InterfaceSystem(n)
        # initialize the objects for all persons
        i = 0
        while i<n:  
            location = (math.floor(i/MAX_LINE_NUM),math.ceil(i%MAX_LINE_NUM))
            obj = Person(env.getWin(),i)
            obj.set_keyboard(keyboard_list[i])
            obj.set_gazekeyboard(gazekeyboard_list[i])
            obj.set_publisher(topic_list[i])
            obj.set_gazepublisher(gazetopic_list[i])
            obj.draw_obj(location)
            unit_list.append(obj)
            i+=1
    elif person_mode==2 and robot_mode==2:
        env = InterfaceSystem(n)
        i = 0
        while i<n:
            # control robot with keyboard_teleop
            if i==0:
                location = (math.floor(i/MAX_LINE_NUM),math.ceil(i%MAX_LINE_NUM))
                obj = Person(env.getWin(),'robot')
                obj.set_keyboard(keyboard_list[i])
                obj.set_robot_teleop()
                obj.draw_obj(location)
                unit_list.append(obj)
                i+=1
            # control persons
            else:    
                location = (math.floor(i/MAX_LINE_NUM),math.ceil(i%MAX_LINE_NUM))
                obj = Person(env.getWin(),i-1)
                obj.set_keyboard(keyboard_list[i])
                obj.set_gazekeyboard(gazekeyboard_list[i])
                obj.set_publisher(topic_list[i-1])
                obj.set_gazepublisher(gazetopic_list[i])
                obj.draw_obj(location)
                unit_list.append(obj)
                i+=1
    
    # loop to read the keyboard events
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit()
            if event.type == pygame.KEYDOWN:
                try:
                    key = chr(event.key)
                except:
                    if event.key==pygame.K_KP8:
                        key=chr(pygame.K_8)
                    elif event.key==pygame.K_KP2:
                        key=chr(pygame.K_2)
                    elif event.key==pygame.K_KP4:
                        key=chr(pygame.K_4)
                    elif event.key==pygame.K_KP6:
                        key=chr(pygame.K_6)
                    elif event.key==pygame.K_KP5:
                        key=chr(pygame.K_5)
                    elif event.key==pygame.K_KP3:
                        key=chr(pygame.K_3)
                    elif event.key==pygame.K_KP9:
                        key=chr(pygame.K_9)
                        

                for unit in unit_list:
                    unit.key_down(key)
                # main_obj.key_down(key)

            if event.type == pygame.KEYUP:
                try:
                    key = chr(event.key)
                except:
                    if event.key==pygame.K_KP8:
                        key=chr(pygame.K_8)
                    elif event.key==pygame.K_KP2:
                        key=chr(pygame.K_2)
                    elif event.key==pygame.K_KP4:
                        key=chr(pygame.K_4)
                    elif event.key==pygame.K_KP6:
                        key=chr(pygame.K_6)
                    elif event.key==pygame.K_KP3:
                        key=chr(pygame.K_3)
                    elif event.key==pygame.K_KP9:
                        key=chr(pygame.K_9)

                for unit in unit_list:
                    unit.key_up(key)

        for unit in unit_list:
            if unit.id == 'robot':
                unit.pub.publish(unit.robot_tel.loopHandleKeys(unit.key_pressed))
            else:
                key_msg = unit.handle_key()
                unit.pub.publish(key_msg)
                
                gaze_key_msg=unit.handle_gaze_key()
                unit.gaze_pub.publish(gaze_key_msg)
        
            # main_obj.handle_key()

        rate.sleep()
