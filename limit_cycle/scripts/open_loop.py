#!/usr/bin/env python
# -*- coding: utf-8 -*-
from tools import AsymmetricGauss2DForOpenLoop
import math
import numpy as np

class open_loop:
    def __init__(self, theta = None, speed = None):
        self.theta = theta
        self.speed = speed
        self.mu = self.theta
        self.sigma = 0.3
        self.last_gaze = None
    
    def setVelocity(self, theta, speed):
        self.theta = theta
        self.speed = speed
        self.mu = self.theta
    
    def getVelocity(self):
        return [self.theta, self.speed]
    
    def updateSigma(self, gaze):
        if self.last_gaze == None:
            self.last_gaze = gaze
        else:  
            # self.sigma *= math.e**(self.last_gaze - gaze)
            # self.sigma = 0.5
            self.sigma = 0.2
            # self.sigma = 1.0
            self.last_gaze = gaze
        self.sigma = min(max(0.2, self.sigma),1.0)
    
    def getCost(self, theta, dis):
        return 20.0* AsymmetricGauss2DForOpenLoop(self.mu, self.sigma, theta, dis, self.speed)
    
