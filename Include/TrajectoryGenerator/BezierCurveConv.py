# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : BezierCurveConf.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.02.10 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
from enum import IntEnum, unique, auto
from typing import Tuple, List, Union
from numpy import sin, cos, arctan2, deg2rad, sqrt, sign, ones, round, cumsum, diff


class CBezierCurveTraj:
    fMaxVel: float
    fMaxAcc: float
    fMaxJerk: float
    fXStart: float
    fYStart: float
    fThetaStart: float
    nSampling: int

    def __init__(self, afMaxV: float, afMaxA: float, afMaxJ: float):
        self.fMaxVel = afMaxV
        self.fMaxAcc = afMaxA
        self.fMaxJerk = afMaxJ
        self.nSampling = 1000

    def _calc_conv(self, listX1: list, listX2: list, dT: float) -> List[float]:
        f_x = listX1
        f_h = listX2
        m_x = len(f_x)
        n_h = len(f_h)

        listConv = [0 * i for i in range(m_x + n_h)]

        for i in range(m_x):
            for j in range(n_h):
                listConv[i + j] += f_x[i] * f_h[j] * dT

        return listConv

    # Convolution velocity profile -> Geon Lee (Convolution-Based Trajectory Generation Methods Using Physical System
    # Limits)
    def calc_conv_vel(self, afPathLen: float, afVstart: float, afVtarget: float, dT: float) -> List[float]:
        v_max = self.fMaxVel
        v_max_1 = self.fMaxAcc
        v_max_2 = self.fMaxJerk
        v_i, v_f = afVstart, afVtarget
        S_n = afPathLen

        if v_max == 0 or v_max_1 == 0 or v_max_2 == 0:
            raise ValueError("Max Vel, Acc, or Jerk should not be zero!")

        t2 = v_max_1 / v_max_2
        v0 = sign(S_n) * v_max - v_i
        t1_plus = (v_max - sign(v0) * v_i) / v_max_1
        t1 = (v_max - sign(v0) * v_f) / v_max_1
        t0 = (sign(S_n) / v_max) * (S_n - (((v_f + v_i) / 2) * (t1 + t2)) + ((v_i / 2) * (t1 - t1_plus))) - (
                t1 - t1_plus) / 2

        # first convolution
        m_x = int(round(t0 / dT))
        n_h = int(round(t1 / dT))
        tmpX1 = ones((1, m_x)).tolist()[0]
        tmpH1 = ones((1, n_h)).tolist()[0]
        X_1 = [v0 * x1 for x1 in tmpX1]
        H_1 = [(1 / t1) * h1 for h1 in tmpH1]
        conv1 = self._calc_conv(X_1, H_1, dT)

        # second convolution
        X_2 = conv1
        n_h2 = int(round(t2 / dT))
        tmpH2 = ones((1, n_h2)).tolist()[0]
        H_2 = [(1 / t2) * h2 for h2 in tmpH2]
        conv2 = self._calc_conv(X_2, H_2, dT)
        return conv2

    def calc_bezier_curve(self, aInit: Tuple[float, float, float], aTarg: Tuple[float, float, float], d1: float,
                          d2: float, au_param: List[float] = None) -> Tuple[List[Tuple[float, float, float]], float]:
        x_start, y_start, theta_start = aInit
        x_targ, y_targ, theta_targ = aTarg

        x_1 = x_start + d1 * cos(deg2rad(theta_start))
        y_1 = y_start + d1 * sin(deg2rad(theta_start))
        x_2 = x_targ + d2 * cos(deg2rad(180 + theta_targ))
        y_2 = y_targ + d2 * sin(deg2rad(180 + theta_targ))

        if au_param is None:
            u_param = 0.
            list_u = [u_param]
            for i in range(self.nSampling):
                u_param += 1 / self.nSampling
                list_u.append(u_param)
        else:
            list_u = au_param

        listXYTheta = []
        theta_u = theta_start
        prev_x = x_start
        prev_y = y_start
        fPathLength = 0.
        for u in list_u:
            x_u = (x_start * (1 - u) ** 3) + (3 * x_1 * u * (1 - u) ** 2) + (3 * x_2 * (u ** 2) * (1 - u)) + (
                    x_targ * (u ** 3))
            y_u = (y_start * (1 - u) ** 3) + (3 * y_1 * u * (1 - u) ** 2) + (3 * y_2 * (u ** 2) * (1 - u)) + (
                    y_targ * (u ** 3))

            listXYTheta.append((x_u, y_u, theta_u))
            dX = x_u - prev_x
            dY = y_u - prev_y
            theta_u = arctan2(dY, dX)
            fPathLength += sqrt(dX ** 2 + dY ** 2)

            prev_x = x_u
            prev_y = y_u

        return listXYTheta, fPathLength

    def generate_bezier_trajectory(self, aInit: Tuple[float, float, float], aTarg: Tuple[float, float, float],
                                   afVstart: float, afVend: float, dT: float):  # -> Tuple[List[float], List[float]]:
        MAXA = self.fMaxAcc
        MAXV = self.fMaxVel
        X_i, Y_i, Theta_i = aInit
        X_f, Y_f, Theta_f = aTarg
        V_i = afVstart
        V_f = afVend

        if X_i == X_f and Y_i == Y_f:
            raise ValueError('Init and Target Pose are the same')

        try:
            d1 = (1 - (MAXA * (1 - (V_i / MAXV)))) * (sqrt((X_f - X_i) * (X_f - X_i) + (Y_f - Y_i) * (Y_f - Y_i)) / 2)
            d2 = (1 - (MAXA * (1 - (V_f / MAXV)))) * (sqrt((X_f - X_i) * (X_f - X_i) + (Y_f - Y_i) * (Y_f - Y_i)) / 2)
        except:
            raise ValueError("Max Vel, Acc, or Jerk should not be zero!")

        # generate Bezier curve in the u-domain
        BezierXYTheta, s_n = self.calc_bezier_curve(aInit, aTarg, 0.005, 0.005)
        try:
            V_c = self.calc_conv_vel(s_n, V_i, V_f, dT)
        except ValueError as err:
            raise ValueError(err)

        # generate Bezier curve in the time domain
        V_dis = cumsum(V_c)
        Bd = sum(V_c) * dT
        u_param = 0.
        list_uc = [u_param]
        for v_dis in V_dis:
            u_param = (v_dis * dT) / Bd
            if u_param >= 1.:
                u_param = 1.

            list_uc.append(u_param)

        BezierXYThetaC, s_nc = self.calc_bezier_curve(aInit, aTarg, d1, d2, list_uc)

        # calculate yaw rate (rads)
        theta = [angles for x, y, angles in BezierXYThetaC]
        tmpYawRate = diff(theta)
        ThetaDot_c = [yaw / dT for yaw in tmpYawRate]

        return V_c, ThetaDot_c, BezierXYThetaC

    def set_phy_limits(self, afMaxVel: float, afMaxAcc: float, afMaxJerk: float):
        self.fMaxVel = afMaxVel
        self.fMaxAcc = afMaxAcc
        self.fMaxJerk = afMaxJerk


if __name__ == '__main__':
    test = CBezierCurveTraj(0.5, 0.4, 0.2)
    start = (0., 0., 0.)
    targ = (4., 4., 0.)
    BezierXY, pathLength = test.calc_bezier_curve(start, targ, 2.2627, 2.2627)
    vel = test.calc_conv_vel(pathLength, 0., 0., 0.01)
    test.generate_bezier_trajectory(start, targ, 0., 0., 0.01)

# x_all = [xy[0] for xy in BezierXY]
# y_all = [xy[1] for xy in BezierXY]
# dx = numpy.diff(x_all)
# dy = numpy.diff(y_all)
# #
# for x in dx:
# 	print(x)
#
# print("HHHHHHHH")
# for x in dy:
# 	print(x)

# u_param = 0.
# list_u = [u_param]
# for i in range(1000):
# 	u_param += 1 / (1000-1)
# 	list_u.append(u_param)
# 	print(u_param)
#
#
# for u in list_u:
#
#
