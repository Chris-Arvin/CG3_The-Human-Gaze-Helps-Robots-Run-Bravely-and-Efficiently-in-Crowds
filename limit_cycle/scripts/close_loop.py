from param import U_MAX
from tools import AsymmetricGauss1DForClosedLoop
import numpy as np
import math

class close_loop:
    def __init__(self, init_sig_l = 1.0, init_sig_r = 1.0, alpha_d = 1.0, dir = None, u = None):
        self.dir = dir
        self.sig_l = init_sig_l
        self.sig_r = init_sig_r
        self.alpha_d = alpha_d
        self.u_star = u
        self.last_gaze = None


    def setDir(self, dir):
        self.dir = dir
    
    def setUStar(self, u_star):
        self.u_star = u_star

    def updateSigmaForPerson(self, new_dir, gaze):
        if new_dir != self.dir:
            self.changeDir(new_dir)
        else:
            if self.last_gaze == None:
                self.last_gaze = gaze
                return
            TEMP_VALUE = 0.2
            # TEMP_VALUE = 1.0
            # self.sig_l *= math.e**(self.dir*(self.last_gaze-gaze))
            # self.sig_r *= math.e**(-self.dir*(self.last_gaze-gaze))
            # self.alpha_d *= math.e**(gaze-self.last_gaze)
            self.sig_l = math.e**(0.5+self.dir*TEMP_VALUE*0.5)
            self.sig_r = math.e**(0.5-self.dir*TEMP_VALUE*0.5)
            self.alpha_d = 4.0-TEMP_VALUE*3.0

            # self.sig_l = min(max(0.2, self.sig_l), 1.0)
            # self.sig_r = min(max(0.2, self.sig_r), 1.0)
            # self.alpha_d = min(max(0.2, self.alpha_d), 1.0)

            # print(self.sig_l, self.sig_r)

    def changeDir(self, new_dir):
        self.dir = new_dir
        self.sig_l = 1.0
        self.sig_r = 1.0

    def getCost(self, u):
        # print('value: ',AsymmetricGauss1DForClosedLoop(self.u_star, self.sig_l, self.sig_r, u))
        return 10.0*AsymmetricGauss1DForClosedLoop(self.u_star, self.dir, self.sig_l, self.sig_r, u)

    def getRobotCost(self,u):
        return 10.0*AsymmetricGauss1DForClosedLoop(self.u_star, self.dir, self.sig_l, self.sig_r, self.u_star + U_MAX-u)