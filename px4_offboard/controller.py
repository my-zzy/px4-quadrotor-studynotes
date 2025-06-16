#!/usr/bin/env python

import math
from px4_offboard.para import *

import rclpy.logging
logger = rclpy.logging.get_logger("controller")

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
    # in radians

    return roll, pitch, yaw


def pd(x, x_dot, xd, xd_dot, kp, kd):
    x_dot2 = - kd*(x_dot-xd_dot) - kp*(x-xd)
    return x_dot2


def pd_controller(pos, att, posd, attd, dt, t):
    
    x, y, z = pos
    phi, theta, psi = att
    xd, yd, zd = posd
    phid, thetad, psid = attd
    g = -9.8    # negative?!

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


    # PD control of position
    # x_dot2 = pd(x, x_dot, xd, xd_dot, kp1, kd1)
    # y_dot2 = pd(y, y_dot, yd, yd_dot, kp2, kd2)
    # z_dot2 = pd(z, z_dot, zd, zd_dot, kp3, kd3)
    x_dot2 = -pd(x[-1], x_dot, xd[-1], xd_dot, kp1, kd1)
    y_dot2 = -pd(y[-1], y_dot, yd[-1], yd_dot, kp2, kd2)
    z_dot2 = pd(z[-1], z_dot, zd[-1], zd_dot, kp3, kd3)
    print(1111111)
    # logger.info(f"{z[-1]}, {z_dot}, {zd[-1]}, {zd_dot}")
    # logger.info(f"z_dot2: {z_dot2}")
    # logger.info(f"x_dot2: {x_dot2}")

    # !!for testing only
    # x_dot2 = 0
    # y_dot2 = 0

    # Note that U1 may be negative
    if z_dot2+g > 0:
        U1 = math.sqrt((m*x_dot2)**2+(m*y_dot2)**2+(m*z_dot2+m*g)**2)
    else:
        U1 = -math.sqrt((m*x_dot2)**2+(m*y_dot2)**2+(m*z_dot2+m*g)**2)

    # Calculate desired phi & theta from expected translation acceleration
    # !! non-standard operation
    psi = psi[-1]
    # g = 9.8


    tem = (x_dot2*math.sin(psi)-y_dot2*math.cos(psi))**2/(x_dot2**2+y_dot2**2+(z_dot2+g)**2)
    if x_dot2*math.sin(psi)-y_dot2*math.cos(psi) > 0:
        phidd = math.asin(math.sqrt(tem))
    else:
        phidd = -math.asin(math.sqrt(tem))

    tem = (z_dot2+g)**2/((x_dot2*math.cos(psi)+y_dot2*math.sin(psi))**2+(z_dot2+g)**2)
    if x_dot2*math.cos(psi)+y_dot2*math.sin(psi) > 0:
    # if m*x_dot2/U1 - math.sin(phi[-1])*math.sin(psi) > 0:
        thetadd = math.acos(math.sqrt(tem))
    else:
        thetadd = -math.acos(math.sqrt(tem))

    # !!for testing only
    # phidd = 0
    # thetadd = 0
    # if t > 2:
    #     # phidd = 0.2
    #     # thetadd = 0.2
    #     phidd = 0.1*math.sin(2*t)
    #     thetadd = 0.2*math.sin(2*t)
    


    # Calculate derivative of desired phi & theta from previous
    phid_dot = (phidd - phid[-1])/dt
    thetad_dot = (thetadd - thetad[-1])/dt
    psid_dot = (psid[-1] - psid[-2])/dt


    # PD control of attitude
    phi_dot2 = pd(phi[-1], phi_dot, phidd, phid_dot, kp4, kd4)
    theta_dot2 = pd(theta[-1], theta_dot, thetadd, thetad_dot, kp5, kd5)
    psi_dot2 = pd(psi, psi_dot, psid[-1], psid_dot, kp6, kd6)

    # TODO: how to get air friction
    # U2 = phi_dot2 * Ixx + l*k4*phi_dot
    U2 = phi_dot2 * Ixx
    U3 = theta_dot2 * Iyy
    U4 = psi_dot2 * Izz

    return U1, U2, U3, U4, phidd, thetadd, x_dot2, y_dot2

    # Why we need to return phid & thetad?
    # To calculate phid_dot & thetad_dot


def adaptive_controller(pos, att, posd, attd, dhat, jifen, dt, t):
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

    dx_hat, dy_hat, dz_hat, dphi_hat, dtheta_hat, dpsi_hat = dhat
    xphi, xtheta, xpsi = jifen
    g = 9.8

    # calculate pos_dot & att_dot
    u = (pos[0][-1] - pos[0][-2])/dt    # x,y,z_dot
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

    # wrong! use new phid & thetad
    phid_dot2 = ((attd[0][-1] - attd[0][-2])/dt - (attd[0][-2] - attd[0][-3])/dt)/dt
    thetad_dot2 = ((attd[1][-1] - attd[1][-2])/dt - (attd[1][-2] - attd[1][-3])/dt)/dt
    psid_dot2 = ((attd[2][-1] - attd[2][-2])/dt - (attd[2][-2] - attd[2][-3])/dt)/dt

    # wrong! use new phid & thetad
    phid_dot = (attd[0][-1] - attd[0][-2])/dt
    thetad_dot = (attd[1][-1] - attd[1][-2])/dt
    psid_dot = (attd[2][-1] - attd[2][-2])/dt

    # position control
    ez = z - zd
    ew = w - zd_dot + cz*ez
    ez_dot = ew - cz*ez
    w_dot = -cw*ew - ez + zd_dot2 - cz*ez_dot
    dz_hat_dot = lamz*ew
    dz_hat += dz_hat_dot*dt
    U1 = (w_dot - dz_hat +g)*m/(math.cos(phi)*math.cos(theta))
    logger.info(f"U1: {U1}")

    ex = x - xd
    eu = u - xd_dot + cu*ex
    ex_dot = eu - cx*ex
    u_dot = -cu*eu - ex + xd_dot2 - cx*ex_dot
    # u_dot = 0   # for testing
    dx_hat_dot = lamx*eu
    dx_hat += dx_hat_dot*dt
    Ux = (u_dot - dx_hat)*m/U1
    logger.info(f"Ux: {Ux}")
    # logger.info(f"dx_hat: {dx_hat}, u_dot: {u_dot}")

    ey = y - yd
    ev = v - yd_dot + cv*ey
    ey_dot = ev - cy*ey
    v_dot = -cv*ev - ey + yd_dot2 - cy*ey_dot
    # v_dot = 0   # for testing
    dy_hat_dot = lamy*ev
    dy_hat += dy_hat_dot*dt
    Uy = (v_dot - dy_hat)*m/U1
    logger.info(f"Uy: {Uy}")

    # for testing only
    if t < 50000*dt:
        Ux = 0
        Uy = 0


    # attitude control
    logger.info(f"{Ux*math.sin(psi) - Uy*math.cos(psi)}")
    phid_new = math.asin(Ux*math.sin(psi) - Uy*math.cos(psi))
    logger.info(f"{(Ux*math.cos(psi) + Uy*math.sin(psi))/math.cos(phid_new)}")
    thetad_new = math.asin((Ux*math.cos(psi) + Uy*math.sin(psi))/math.cos(phid_new))

    if t > 500*dt:
        phid_new = 0.1
        thetad_new = 0.2

    epsi = psi - psid
    # logger.info(f"psid: {round(psid, 4)}, psi: {round(psi, 4)}, epsi: {round(epsi, 4)}")
    epsi_dot = psi_dot - psid_dot
    # logger.info(f"psid_dot: {round(psid_dot, 4)}, epsi_dot: {round(epsi_dot, 4)}")
    xpsi += epsi    # TODO:initialize xpsi
    alpha_psi = psid_dot - cpsi*epsi
    beta_psi = psi_dot - alpha_psi + lampsi*xpsi
    psi_dot2 = -cr*beta_psi + psid_dot2 - cpsi*epsi_dot - lampsi*epsi - epsi
    dpsi_hat_dot = lampsi_star*beta_psi
    dpsi_hat += dpsi_hat_dot*dt
    U4 = (psi_dot2 - dpsi_hat - theta_dot*phi_dot*(Ixx-Iyy)/Izz)*Izz/l
    # logger.info(f"U4: {round(U4, 4)}")
    # logger.info(f"dpsi_hat: {round(dpsi_hat, 4)}, phi_dot2: {round(psi_dot2, 4)}")

    ephi = phi - phid_new
    ephi_dot = phi_dot - phid_dot
    xphi += ephi
    alpha_phi = phid_dot - cphi*ephi
    beta_phi = phi_dot - alpha_phi + lamphi*xphi
    phi_dot2 = -cp*beta_phi + phid_dot2 - cphi*ephi_dot - lamphi*ephi - ephi
    dphi_hat_dot = lamphi_star*beta_phi
    dphi_hat += dphi_hat_dot*dt
    U2 = (phi_dot2 - dphi_hat - theta_dot*psi_dot*(Iyy-Izz)/Ixx)*Ixx/l
    # logger.info(f"U2: {round(U2, 4)}")
    # logger.info(f"dphi_hat: {round(dphi_hat, 4)}, phi_dot2: {round(phi_dot2, 4)}")
    

    ethata = theta - thetad_new
    etheta_dot = theta_dot - thetad_dot
    xtheta += ethata
    alpha_theta = thetad_dot - cthe*ethata
    beta_theta = theta_dot - alpha_theta + lamthe*xtheta
    theta_dot2 = -cq*beta_theta + thetad_dot2 - cthe*etheta_dot - lamthe*ethata - ethata
    dtheta_hat_dot = lamthe_star*beta_theta
    dtheta_hat += dtheta_hat_dot*dt
    U3 = (theta_dot2 - dtheta_hat - phi_dot*psi_dot*(Izz-Ixx)/Iyy)*Iyy/l

    dhat_old = [dx_hat, dy_hat, dz_hat, dphi_hat, dtheta_hat, dpsi_hat]
    jifen_old = [xphi, xtheta, xpsi]

    # if t < 100*dt:
    #     U2 = 0
    #     U3 = 0

    return U1, U2, U3, U4, phid_new, thetad_new, dhat_old, jifen_old, Ux, Uy