#!/usr/bin/env python

import math
from para import *

def hold():
    return 0, 0, -5

def circle(t):
    xd = 10*math.sin(0.15*t)
    yd = 10*math.cos(0.15*t)
    zd = 0.2*t
    return xd, yd, zd


def pd(x, x_dot, xd, xd_dot, xd_dot2, kp, kd):
    x_dot2 = - kd*(x_dot-xd_dot) - kp*(x-xd) + xd_dot2
    return x_dot2


def pd_controller(pos, pos_dot, att, att_dot, posd, posd_dot, posd_dot2, psid, psid_dot, psid_dot2, phid_old, thetad_old, m, l, Ixx, Iyy, Izz, dt):
    
    x, y, z = pos
    x_dot, y_dot, z_dot = pos_dot
    phi, theta, psi = att
    phi_dot, theta_dot, psi_dot = att_dot
    xd, yd, zd = posd
    xd_dot, yd_dot, zd_dot = posd_dot
    xd_dot2, yd_dot2, zd_dot2 = posd_dot2
    g = 9.8

    # PD control of position
    x_dot2 = pd(x, x_dot, xd, xd_dot, xd_dot2, kp1, kd1)
    y_dot2 = pd(y, y_dot, yd, yd_dot, yd_dot2, kp2, kd2)
    z_dot2 = pd(z, z_dot, zd, zd_dot, zd_dot2, kp3, kd3)

    # Note that U1 may be negative
    if z_dot2+g > 0:
        U1 = math.sqrt((m*x_dot2)**2+(m*y_dot2)**2+(m*z_dot2+m*g)**2)
    else:
        U1 = -math.sqrt((m*x_dot2)**2+(m*y_dot2)**2+(m*z_dot2+m*g)**2)

    # Calculate desired phi & theta from expected translation acceleration
    tem = (x_dot2*math.sin(psi)-y_dot2*math.cos(psi))**2/(x_dot2**2+y_dot2**2+(z_dot2+g)**2)
    if x_dot2*math.sin(psi)-y_dot2*math.cos(psi) > 0:
        phid = math.asin(math.sqrt(tem))
    else:
        phid = -math.asin(math.sqrt(tem))

    tem = (z_dot2+g)**2/((x_dot2*math.cos(psi)+y_dot2*math.sin(psi))**2+(z_dot2+g)**2)
    # if x_dot2*math.cos(psi)+y_dot2*math.sin(psi) > 0:
    if m*x_dot2/U1 - math.sin(phid)*math.sin(psid) > 0:
        thetad = math.acos(math.sqrt(tem))
    else:
        thetad = -math.acos(math.sqrt(tem))

    # Calculate derivative of desired phi & theta from previous
    phid_dot = (phid - phid_old)/dt
    thetad_dot = (thetad - thetad_old)/dt
    phid_dot2 = 0   # TODO
    thetad_dot2 = 0

    # PD control of attitude
    phi_dot2 = pd(phi, phi_dot, phid, phid_dot, phid_dot2, kp4, kd4)
    theta_dot2 = pd(theta, theta_dot, thetad, thetad_dot, thetad_dot2, kp5, kd5)
    psi_dot2 = pd(psi, psi_dot, psid, psid_dot, psid_dot2, kp6, kd6)

    # TODO: how to get air friction
    # U2 = phi_dot2 * Ixx + l*k4*phi_dot
    U2 = phi_dot2 * Ixx
    U3 = theta_dot2 * Iyy
    U4 = psi_dot2 * Izz

    return U1, U2, U3, U4, phid, thetad

    # Why we need to return phid & thetad?
    # To calculate phid_dot(2) & thetad_dot(2)


def adaptive_controller(x, y, z, roll, pitch, yaw, xd, yd, zd):
    pass
