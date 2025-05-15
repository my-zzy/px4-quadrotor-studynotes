#!/usr/bin/env python

import math

def desire_trajectory(t):
    xd = 10*math.sin(0.15*t)
    yd = 10*math.cos(0.15*t)
    zd = 0.2*t
    return xd, yd, zd

def adaptive_controller(x, y, z, roll, pitch, yaw, xd, yd, zd):
    pass


def pd(x, x_dot, xd, xd_dot, xd_dot2):
    pass


def pd_controller(pos, pos_dot, att, att_dot, posd, posd_dot, posd_dot2, psid, psid_dot, psid_dot2, m):
    
    x, y, z = pos
    x_dot, y_dot, z_dot = pos_dot
    phi, theta, psi = att
    phi_dot, theta_dot, psi_dot = att_dot
    xd, yd, zd = posd
    xd_dot, yd_dot, zd_dot = posd_dot
    xd_dot2, yd_dot2, zd_dot2 = posd_dot2
    
    x_dot2 = 0
    y_dot2 = 0
    z_dot2 = 0
    g = 9.8
    U1 = math.sqrt((m*x_dot2)**2+(m*y_dot2)**2+(m*z_dot2+m*g)**2)

    tem = (x_dot2*math.sin(psi)-y_dot2*math.cos(psi))**2/(x_dot2**2+y_dot2**2+(z_dot2+g)**2)
    phid = math.asin(math.sqrt(tem))

    tem = (z_dot2+g)**2/((x_dot2*math.cos(psi)+y_dot2*math.sin(psi))**2+(z_dot2+g)**2)
    thetad = math.acos(math.sqrt(tem))

    phi_dot2 = 0
    theta_dot2 = 0
    psi_dot2 = 0

    U2 = 0
    U3 = 0
    U4 = 0

    return U1, U2, U3, U4