#!/usr/bin/env python
# -*- coding: utf-8 -*-
from scipy.stats import gamma
import numpy as np
import math

def cycle_closewise(x,y,R,r=1,u=1):
    dx = r*y + u*x*(R**2-x**2-y**2)
    dy = -r*x + u*y*(R**2-x**2-y**2)
    return dx, dy


# def cycle_counter_closewise(x,y,R, r=-1,u=1):
#     dx = r*y + u*x*(R**2-x**2-y**2)
#     dy = -r*x + u*y*(R**2-x**2-y**2)
#     return dx,dy


def Gamma(x,alpha_values = 5,beta_values = 0.5):
    dist = gamma(alpha_values, 0, beta_values)
    return 5*dist.pdf(0.6*x+0.1)


def Gauss(x,mu, sigma):
    value = 1/(np.sqrt(2*np.pi))*math.e**(-(x-mu)**2/(2*sigma**2))
    return value

def AsymmetricGauss1DForClosedLoop(u_star, dir, sig_l, sig_r, u):
    if u_star<0 or u<0 or sig_l<0 or sig_r<0:
        print('error for closedloop gauss')
        exit(1)
    # normalize_param = 2.0*math.sqrt(2*math.pi)*sig_l**2/(sig_l+sig_r)
    normalize_param = 1.0
    # print(sig_l, sig_r, normalize_param)
    # if u<u_star:
    #     return 100.0*np.exp(-((u - u_star)/20.0) ** 2 /(2* sig_l **2)) * normalize_param
    # else:
    #     return 100.0*np.exp(-((u - u_star)/20.0) ** 2 /(2* sig_r **2)) * normalize_param
    if dir == -1:
        if u<u_star:
            return np.exp(-((u - u_star)/20.0) ** 2 /(2* (1+0.1*sig_r) **2))
        else:
            return np.exp(-((u - u_star)/20.0) ** 2 /(2* (1+0.1*sig_l) **2))

    if dir == 1:
        if u<u_star:
            return np.exp(-((u - u_star)/20.0) ** 2 /(2* (1+0.1*sig_l) **2))
        else:
            return np.exp(-((u - u_star)/20.0) ** 2 /(2* (1+0.1*sig_r) **2))

# 从当前mu_theta开始，向左是负号，向右是正号
def AsymmetricGauss2DForOpenLoop(mu_theta, sig_l, theta, dis, speed):
    DETECTION_DURATION = 2.0
    sig_f = max(DETECTION_DURATION*1.0, DETECTION_DURATION*speed)
    # print('.:',theta, mu_theta)
    # print((dis*math.cos(theta - mu_theta))**2/(2*sig_l**2), (dis*math.sin(theta-mu_theta))**2/(2*sig_f**2) )
    # print('..:', np.exp(-( (dis*math.cos(theta - mu_theta))**2/(2*sig_l**2) + (dis*math.sin(theta-mu_theta))**2/(2*sig_f**2) ) ),(math.sqrt(2*math.pi)*sig_l))
    # return np.exp(-( (dis*math.sin(theta - mu_theta))**2/(2*sig_l**2) + (dis*math.cos(theta-mu_theta))**2/(2*sig_f**2) ) )/(math.sqrt(2*math.pi)*sig_l) 
    return np.exp(-( (dis*math.sin(theta - mu_theta))**2/(2*(0.5+sig_l*0.1)**2) + (dis*math.cos(theta-mu_theta))**2/(2*sig_f**2) ) )/(math.sqrt(2*math.pi)*sig_l) 


def thetaConstrain(theta):
    while theta<0:
        theta += 2*np.pi
    while theta > 2*np.pi:
        theta -= 2*np.pi
    return theta
