#!/usr/bin/env python

import math
from px4_offboard.para import *


def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion into roll, pitch, yaw (in radians)
    Roll  = rotation around x-axis
    Pitch = rotation around y-axis
    Yaw   = rotation around z-axis
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def hold():
    return 0, 0, -2, 1.78

def circle(t):
    xd = 10*math.sin(0.15*t)
    yd = 10*math.cos(0.15*t)
    zd = -0.2*t
    psid = 1.78
    return xd, yd, zd, psid


def pd(x, x_dot, xd, xd_dot, kp, kd):
    x_dot2 = - kd*(x_dot-xd_dot) - kp*(x-xd)
    return x_dot2


def pd_controller(pos, att, posd, attd, dt):
    
    x, y, z = pos
    phi, theta, psi = att
    xd, yd, zd = posd
    phid, thetad, psid = attd
    g = 9.8

    # calculate pos_dot & att_dot
    x_dot = (pos[0][-1] - pos[0][-2])/dt    # x,y,z
    y_dot = (pos[1][-1] - pos[1][-2])/dt
    z_dot = (pos[2][-1] - pos[2][-2])/dt

    phi_dot = (att[0][-1] - att[0][-2])/dt
    theta_dot = (att[1][-1] - att[1][-2])/dt
    psi_dot = (att[2][-1] - att[2][-2])/dt

    xd_dot = (xd[-1] - xd[-2])/dt
    yd_dot = (yd[-1] - yd[-2])/dt
    zd_dot = (zd[-1] - zd[-2])/dt

    phid_dot = (phid[-1] - phid[-2])/dt
    thetad_dot = (thetad[-1] - thetad[-2])/dt
    psid_dot = (psid[-1] - psid[-2])/dt


    # PD control of position
    # x_dot2 = pd(x, x_dot, xd, xd_dot, kp1, kd1)
    # y_dot2 = pd(y, y_dot, yd, yd_dot, kp2, kd2)
    # z_dot2 = pd(z, z_dot, zd, zd_dot, kp3, kd3)
    x_dot2 = pd(x[-1], x_dot, xd[-1], xd_dot, kp1, kd1)
    y_dot2 = pd(y[-1], y_dot, yd[-1], yd_dot, kp2, kd2)
    z_dot2 = pd(z[-1], z_dot, zd[-1], zd_dot, kp3, kd3)

    # Note that U1 may be negative
    if z_dot2+g > 0:
        U1 = math.sqrt((m*x_dot2)**2+(m*y_dot2)**2+(m*z_dot2+m*g)**2)
    else:
        U1 = -math.sqrt((m*x_dot2)**2+(m*y_dot2)**2+(m*z_dot2+m*g)**2)

    # Calculate desired phi & theta from expected translation acceleration
    # !! non-standard operation
    psi = psi[-1]
    tem = (x_dot2*math.sin(psi)-y_dot2*math.cos(psi))**2/(x_dot2**2+y_dot2**2+(z_dot2+g)**2)
    if x_dot2*math.sin(psi)-y_dot2*math.cos(psi) > 0:
        phid = math.asin(math.sqrt(tem))
    else:
        phid = -math.asin(math.sqrt(tem))

    tem = (z_dot2+g)**2/((x_dot2*math.cos(psi)+y_dot2*math.sin(psi))**2+(z_dot2+g)**2)
    # if x_dot2*math.cos(psi)+y_dot2*math.sin(psi) > 0:
    if m*x_dot2/U1 - math.sin(phi[-1])*math.sin(psi) > 0:
        thetad = math.acos(math.sqrt(tem))
    else:
        thetad = -math.acos(math.sqrt(tem))

    # Calculate derivative of desired phi & theta from previous
    # phid_dot = (phid - phid_old)/dt
    # thetad_dot = (thetad - thetad_old)/dt
    # phid_dot2 = 0
    # thetad_dot2 = 0

    # PD control of attitude
    phi_dot2 = pd(phi[-1], phi_dot, phid, phid_dot, kp4, kd4)
    theta_dot2 = pd(theta[-1], theta_dot, thetad, thetad_dot, kp5, kd5)
    psi_dot2 = pd(psi, psi_dot, psid[-1], psid_dot, kp6, kd6)

    # TODO: how to get air friction
    # U2 = phi_dot2 * Ixx + l*k4*phi_dot
    U2 = phi_dot2 * Ixx
    U3 = theta_dot2 * Iyy
    U4 = psi_dot2 * Izz

    return U1, U2, U3, U4, phid, thetad

    # Why we need to return phid & thetad?
    # To calculate phid_dot & thetad_dot


def adaptive_controller(pos, att, posd, attd, dhat, dt):
    x = pos[0][-1]
    y = pos[1][-1]
    z = pos[2][-1]
    phi = att[0][-1]
    theta = att[1][-1]
    psi = att[2][-1]

    xd = posd[0][-1]
    yd = posd[1][-1]
    zd = posd[2][-1]
    phid = attd[0][-1]
    thetad = attd[1][-1]
    psid = attd[2][-1]

    dx_hat, dy_hat, dz_hat = dhat
    g = 9.8

    # calculate pos_dot & att_dot
    u = (pos[0][-1] - pos[0][-2])/dt    # x,y,z
    v = (pos[1][-1] - pos[1][-2])/dt
    w = (pos[2][-1] - pos[2][-2])/dt

    phi_dot = (att[0][-1] - att[0][-2])/dt
    theta_dot = (att[1][-1] - att[1][-2])/dt
    psi_dot = (att[2][-1] - att[2][-2])/dt

    xd_dot = (posd[0][-1] - posd[0][-2])/dt
    yd_dot = (posd[1][-1] - posd[1][-2])/dt
    zd_dot = (posd[2][-1] - posd[2][-2])/dt

    xd_dot2 = ((posd[0][-1] - posd[0][-2])/dt - (posd[0][-2] - posd[0][-3])/dt)/dt
    yd_dot2 = ((posd[1][-1] - posd[1][-2])/dt - (posd[1][-2] - posd[1][-3])/dt)/dt
    zd_dot2 = ((posd[2][-1] - posd[2][-2])/dt - (posd[2][-2] - posd[2][-3])/dt)/dt

    phid_dot = (attd[0][-1] - attd[0][-2])/dt
    thetad_dot = (attd[1][-1] - attd[1][-2])/dt
    psid_dot = (attd[2][-1] - attd[2][-2])/dt

    # position control
    ez = z - zd
    ew = w - zd_dot + cz*ez
    ez_dot = ew - cz*ez
    w_dot = -cw*ew - ez + zd_dot2 - cz*ez_dot
    dz_hat_dot = lamz*ew
    U1 = (w_dot - dz_hat +g)*m/(math.cos(phi)*math.cos(theta))

    ex = x - xd
    eu = u - xd_dot + cu*eu
    ex_dot = eu - cx*ex
    u_dot = -cu*eu - ex + xd_dot2 - cx*ex_dot
    dx_hat_dot = lamx*eu
    Ux = (u_dot - dx_hat +g)*m/U1

    ey = y - yd
    ev = v - yd_dot + cv*ey
    ey_dot = ev - cy*ey
    v_dot = -cv*ev - ey + yd_dot2 - cy*ey_dot
    dy_hat_dot = lamy*ev
    Uy = (v_dot - dy_hat)*m/U1


    return U1, dz_hat_dot