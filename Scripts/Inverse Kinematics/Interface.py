import tkinter as tk
import serial
import time
from numpy import *
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
'''
serialcomm = serial.Serial('COM7', 115200)
serialcomm.bytesize = serial.EIGHTBITS
serialcomm.parity = serial.PARITY_NONE
serialcomm.stopbits = serial.STOPBITS_ONE
serialcomm.timeout = 2
time.sleep(2)
print(serialcomm.readline().decode('ascii'))


def mvmt(C1, C2, C3, C4, C5, C6):
    t = .1
    if int(C1) != 0:
        M1 = '#MTR1' + C1
        serialcomm.write((M1 + '\n').encode())
        print(serialcomm.readline().decode('ascii'))
        print(serialcomm.readline().decode('ascii'))
        time.sleep(t)
    if int(C2) != 0:
        M2 = '#MTR2' + C2
        serialcomm.write((M2 + '\n').encode())
        print(serialcomm.readline().decode('ascii'))
        print(serialcomm.readline().decode('ascii'))
        time.sleep(t)
    if int(C3) != 0:
        M3 = '#MTR3' + C3
        serialcomm.write((M3 + '\n').encode())
        print(serialcomm.readline().decode('ascii'))
        print(serialcomm.readline().decode('ascii'))
        time.sleep(t)
    if int(C4) != 0:
        M4 = '#MTR4' + C4
        serialcomm.write((M4 + '\n').encode())
        print(serialcomm.readline().decode('ascii'))
        print(serialcomm.readline().decode('ascii'))
        time.sleep(t)
    if int(C5) != 0:
        M5 = '#MTR5' + C5
        serialcomm.write((M5 + '\n').encode())
        print(serialcomm.readline().decode('ascii'))
        print(serialcomm.readline().decode('ascii'))
        time.sleep(t)
    if int(C6) != 0:
        M6 = '#MTR6' + C6
        serialcomm.write((M6 + '\n').encode())
        print(serialcomm.readline().decode('ascii'))
        print(serialcomm.readline().decode('ascii'))
        time.sleep(t)

    i = '#EXEC'
    serialcomm.write((i + '\n').encode())
    print(serialcomm.readline().decode('ascii'))
    time.sleep(t)
'''


def transform_matrix(i, DHT):
    return [
        [cos(DHT[i][0]), -sin(DHT[i][0]) * cos(DHT[i][1]), sin(DHT[i][0]) * sin(DHT[i][1]),
         DHT[i][2] * cos(DHT[i][0])],
        [sin(DHT[i][0]), cos(DHT[i][0]) * cos(DHT[i][1]), -cos(DHT[i][0]) * sin(DHT[i][1]),
         DHT[i][2] * sin(DHT[i][0])],
        [0, sin(DHT[i][1]), cos(DHT[i][1]), DHT[i][3]],
        [0, 0, 0, 1]]


def transform_matrix2(ti, alpha_i_m1, a_i_m1, di):
    return [[cos(ti), -sin(ti), 0, a_i_m1],
            [sin(ti) * cos(alpha_i_m1), cos(ti) * cos(alpha_i_m1), -
             sin(alpha_i_m1), -sin(alpha_i_m1) * di],
            [sin(ti) * sin(alpha_i_m1), cos(ti) * sin(alpha_i_m1),
             cos(alpha_i_m1), cos(alpha_i_m1) * di],
            [0, 0, 0, 1]]


# class defines robot joint vertices and stores & plots vertex definition
# in cartesian space relative to the origin of the base frame
# those vertices get plotted within MPL's 3d visualization plotter using their plot function.
class FKSixR:

    def __init__(self, ax, theta=(90, 0, 0, 90, 90, 0)):
        self.ax = ax
        self.d1 = 72.39
        self.a2 = 150.114
        self.a3 = 150.114
        self.d4 = 3.556
        self.d5 = 83.82
        self.d6 = 27.178
        self.theta = theta  # theta angles in degrees
        # self.pt determines Denavit-Hartenberg Table using joint angle, rotational link offset, link offset,
        # and link length, respectively
        self.pt = [[theta[0], 0, 0, self.d1],
                   [theta[1], pi / 2, 0, 0],
                   [theta[2], 0, -self.a2, 0],
                   [theta[3], 0, -self.a3, self.d4],
                   [theta[4], pi / 2, 0, self.d5],
                   [theta[5], -pi / 2, 0, self.d6]]
        # self.fk determines link i frame and link i-1 frame transformation matrices
        self.fk = []
        for i in range(6):
            self.fk.append(transform_matrix2(
                self.pt[i][0] / 180 * pi, self.pt[i][1], self.pt[i][2], self.pt[i][3]))

        # determination of transformation matrix for link i frame relative to base frame
        self.H0_1 = self.fk[0]
        self.H0_2 = dot(self.fk[0], self.fk[1])
        self.H0_3 = dot(self.H0_2, self.fk[2])
        self.H0_4 = dot(self.H0_3, self.fk[3])
        self.H0_5 = dot(self.H0_4, self.fk[4])
        self.H0_6 = dot(self.H0_5, self.fk[5])

        # stores limb vertex locations in x,y,z relative to base frame
        self.limb_vertecies = [[self.H0_1[0][3], self.H0_1[1][3], self.H0_1[2][3]],
                               [self.H0_2[0][3], self.H0_2[1][3], self.H0_2[2][3]],
                               [self.H0_3[0][3], self.H0_3[1][3], self.H0_3[2][3]],
                               [self.H0_4[0][3], self.H0_4[1][3], self.H0_4[2][3]],
                               [self.H0_5[0][3], self.H0_5[1][3], self.H0_5[2][3]],
                               [self.H0_6[0][3], self.H0_6[1][3], self.H0_6[2][3]],
                               ]

    # takes stored limb vertices and plots the data points using .plot() function, generating a wireframe representation
    def draw_limbs(self):
        limb_vectors = [[0, 0, 0]]
        for i, limb, in enumerate(self.limb_vertecies):
            limb_vectors.append(self.limb_vertecies[i])
        # limb_vectors = [[3, 2, 2], [14, 13, 51], [43, 53, 75]]
        x_data = [vector[0] for vector in limb_vectors]
        y_data = [vector[1] for vector in limb_vectors]
        z_data = [vector[2] for vector in limb_vectors]
        self.ax.plot(x_data, y_data, z_data)


class IKSixR:

    def __init__(self, ax, H06=(1, 0, 0, -3.07495448e+02, 0, 0, -1, -1.21587006e+02, 0, 1, 0, 3.59115360e+01)):
        self.ax = ax
        self.d1 = 72.39
        self.a2 = 150.114
        self.a3 = 150.114
        self.d4 = 3.556
        self.d5 = 83.82
        self.d6 = 27.178
        self.H06 = H06

        # Final frame in reference to the base frame
        self.H0_6 = [[H06[0], H06[1], H06[2], H06[3]],
                     [H06[4], H06[5], H06[6], H06[7]],
                     [H06[8], H06[9], H06[10], H06[11]],
                     [0, 0, 0, 1]]

        # Denavit Hartenberg Table
        self.PT = [[0, 0, 0, self.d1],
                   [0, pi / 2, 0, 0],
                   [0, 0, -self.a2, 0],
                   [0, 0, -self.a3, self.d4],
                   [0, pi / 2, 0, self.d5],
                   [0, -pi / 2, 0, self.d6]]
        # Angle variable list init
        self.t1 = []
        self.t2 = []
        self.t3 = []
        self.t4 = []
        self.t5 = []
        self.t6 = []

    # Inverse Kinematics method
    def IK(self):
        try:

            Md6 = [[0],
                   [0],
                   [-self.d6],
                   [1]]
            P0_5 = dot(self.H0_6, Md6)

            R = (sqrt(square(P0_5[1][0]) + square(P0_5[0][0])))

            val = math.atan2(P0_5[1][0], P0_5[0][0]) + \
                math.acos(self.d4 / R) + pi / 2
            # Angles get added to list
            if -0.005 < val < 0.005:
                self.t1.append(0)
            else:
                self.t1.append(val)

            val = math.atan2(P0_5[1][0], P0_5[0][0]) - \
                math.acos(self.d4 / R) + pi / 2
            if -0.005 < val < 0.005:
                self.t1.append(0)
            else:
                self.t1.append(val)

            val = (self.H0_6[0][3] * sin(self.t1[0]) - self.H0_6[1]
                   [3] * cos(self.t1[0]) - self.d4) / self.d6

            if 1 > val > -1:
                self.t5.append(
                    math.acos(
                        (self.H0_6[0][3] * sin(self.t1[0]) - self.H0_6[1][3] * cos(self.t1[0]) - self.d4) / self.d6))
                self.t5.append(
                    -math.acos(
                        (self.H0_6[0][3] * sin(self.t1[0]) - self.H0_6[1][3] * cos(self.t1[0]) - self.d4) / self.d6))
            elif val > 1 > val - 0.005 and val > 0:
                self.t5.append(math.acos(1))
                self.t5.append(math.acos(1))
            elif val + 0.005 > -1 > val and val < 0:
                self.t5.append(math.acos(-1))
                self.t5.append(math.acos(-1))

            val = (self.H0_6[0][3] * sin(self.t1[1]) - self.H0_6[1]
                   [3] * cos(self.t1[1]) - self.d4) / self.d6
            if 1 > val > -1:
                self.t5.append(
                    math.acos(
                        (self.H0_6[0][3] * sin(self.t1[1]) - self.H0_6[1][3] * cos(self.t1[1]) - self.d4) / self.d6))
                self.t5.append(
                    -math.acos(
                        (self.H0_6[0][3] * sin(self.t1[1]) - self.H0_6[1][3] * cos(self.t1[1]) - self.d4) / self.d6))
            elif val > 1 > val - 0.005 and val > 0:
                self.t5.append(math.acos(1))
                self.t5.append(math.acos(1))
            elif val + 0.005 > -1 > val and val < 0:
                self.t5.append(math.acos(-1))
                self.t5.append(math.acos(-1))

            for g in range(2):
                H0_1 = transform_matrix2(
                    self.t1[0], self.PT[0][1], self.PT[0][2], self.PT[0][3])
                H4_5 = transform_matrix2(
                    self.t5[g], self.PT[4][1], self.PT[4][2], self.PT[4][3])
                Md4 = [[0],
                       [0],
                       [0],
                       [1]]
                invH0_6 = linalg.inv(self.H0_6)
                Tt1 = [[cos(self.t1[0]), sin(self.t1[0]), 0, 0],
                       [-sin(self.t1[0]), cos(self.t1[0]), 0, 0],
                       [0, 0, 1, -self.d1]]
                if sin(self.t5[g]) == 0:
                    self.t6.append(0)
                elif sin(self.t5[g]) != 0:
                    self.t6.append(math.atan2(
                        ((-(invH0_6[1][0] * sin(self.t1[0])) + (invH0_6[1]
                         [1] * cos(self.t1[0]))) / sin(self.t5[g])),
                        (((invH0_6[0][0] * sin(self.t1[0])) - (invH0_6[0][1] * cos(self.t1[0]))) / sin(self.t5[g]))))
                H5_6 = transform_matrix2(
                    self.t6[g], self.PT[5][1], self.PT[5][2], self.PT[5][3])
                P0_4 = dot(dot(self.H0_6, linalg.inv(dot(H4_5, H5_6))), Md4)
                P1_4 = dot(Tt1, P0_4)
                P1_4xzSQR = float(square(P1_4[0][0]) + square(P1_4[2][0]))
                if -1 <= ((P1_4xzSQR - square(self.a2) - square(self.a3)) / (2 * self.a2 * self.a3)) <= 1:
                    self.t3.append(
                        math.acos(((P1_4xzSQR - square(self.a2) - square(self.a3)) / (2 * self.a2 * self.a3))))
                    self.t2.append(math.atan2(-P1_4[2][0], -P1_4[0][0]) - math.asin(
                        -self.a3 * sin(self.t3[len(self.t2)]) / sqrt(P1_4xzSQR)))
                    H1_2 = transform_matrix2(
                        self.t2[len(self.t2) - 1], self.PT[1][1], self.PT[1][2], self.PT[1][3])
                    H0_4 = dot(self.H0_6, linalg.inv(dot(H4_5, H5_6)))
                    H2_3 = transform_matrix2(-self.t3[len(self.t3) - 1],
                                             self.PT[2][1], self.PT[2][2], self.PT[2][3])
                    H1_4 = dot(linalg.inv(H0_1), H0_4)
                    H2_4 = dot(linalg.inv(H1_2), H1_4)
                    H3_4 = dot(linalg.inv(H2_3), H2_4)
                    self.t4.append(math.atan2(H3_4[1][0], H3_4[0][0]))

                    self.t3.append(
                        -math.acos(((P1_4xzSQR - square(self.a2) - square(self.a3)) / (2 * self.a2 * self.a3))))
                    self.t2.append(math.atan2(-P1_4[2][0], -P1_4[0][0]) - math.asin(
                        -self.a3 * sin(self.t3[len(self.t2)]) / sqrt(P1_4xzSQR)))
                    H1_2 = transform_matrix2(
                        self.t2[len(self.t2) - 1], self.PT[1][1], self.PT[1][2], self.PT[1][3])
                    H0_4 = dot(self.H0_6, linalg.inv(dot(H4_5, H5_6)))
                    H2_3 = transform_matrix2(-self.t3[len(self.t3) - 1],
                                             self.PT[2][1], self.PT[2][2], self.PT[2][3])
                    H1_4 = dot(linalg.inv(H0_1), H0_4)
                    H2_4 = dot(linalg.inv(H1_2), H1_4)
                    H3_4 = dot(linalg.inv(H2_3), H2_4)
                    self.t4.append(math.atan2(H3_4[1][0], H3_4[0][0]))

            for f in range(2, 4):
                H0_1 = transform_matrix2(
                    self.t1[1], self.PT[0][1], self.PT[0][2], self.PT[0][3])
                H4_5 = transform_matrix2(
                    self.t5[f], self.PT[4][1], self.PT[4][2], self.PT[4][3])
                Md4 = [[0],
                       [0],
                       [0],
                       [1]]
                invH0_6 = linalg.inv(self.H0_6)
                Tt1 = [[cos(self.t1[1]), sin(self.t1[1]), 0, 0],
                       [-sin(self.t1[1]), cos(self.t1[1]), 0, 0],
                       [0, 0, 1, -self.d1]]
                if sin(self.t5[f]) == 0:
                    self.t6.append(0)
                elif sin(self.t5[f]) != 0:
                    self.t6.append(math.atan2(
                        ((-(invH0_6[1][0] * sin(self.t1[1])) + (invH0_6[1]
                         [1] * cos(self.t1[1]))) / sin(self.t5[f])),
                        (((invH0_6[0][0] * sin(self.t1[1])) - (invH0_6[0][1] * cos(self.t1[1]))) / sin(self.t5[f]))))
                H5_6 = transform_matrix2(
                    self.t6[f], self.PT[5][1], self.PT[5][2], self.PT[5][3])
                P0_4 = dot(dot(self.H0_6, linalg.inv(dot(H4_5, H5_6))), Md4)
                P1_4 = dot(Tt1, P0_4)
                P1_4xzSQR = float(square(P1_4[0][0]) + square(P1_4[2][0]))
                if -1 <= ((P1_4xzSQR - square(self.a2) - square(self.a3)) / (2 * self.a2 * self.a3)) <= 1:
                    self.t3.append(
                        math.acos(((P1_4xzSQR - square(self.a2) - square(self.a3)) / (2 * self.a2 * self.a3))))

                    self.t2.append(math.atan2(-P1_4[2][0], -P1_4[0][0]) - math.asin(
                        -self.a3 * sin(self.t3[len(self.t2)]) / sqrt(P1_4xzSQR)))
                    H1_2 = transform_matrix2(
                        self.t2[len(self.t2) - 1], self.PT[1][1], self.PT[1][2], self.PT[1][3])
                    H0_4 = dot(self.H0_6, linalg.inv(dot(H4_5, H5_6)))
                    H2_3 = transform_matrix2(-self.t3[len(self.t3) - 1],
                                             self.PT[2][1], self.PT[2][2], self.PT[2][3])
                    H1_4 = dot(linalg.inv(H0_1), H0_4)
                    H2_4 = dot(linalg.inv(H1_2), H1_4)
                    H3_4 = dot(linalg.inv(H2_3), H2_4)
                    self.t4.append(math.atan2(H3_4[1][0], H3_4[0][0]))

                    self.t3.append(
                        -math.acos(((P1_4xzSQR - square(self.a2) - square(self.a3)) / (2 * self.a2 * self.a3))))

                    self.t2.append(math.atan2(-P1_4[2][0], -P1_4[0][0]) - math.asin(
                        -self.a3 * sin(self.t3[len(self.t2)]) / sqrt(P1_4xzSQR)))
                    H1_2 = transform_matrix2(
                        self.t2[len(self.t2) - 1], self.PT[1][1], self.PT[1][2], self.PT[1][3])
                    H0_4 = dot(self.H0_6, linalg.inv(dot(H4_5, H5_6)))
                    H2_3 = transform_matrix2(-self.t3[len(self.t3) - 1],
                                             self.PT[2][1], self.PT[2][2], self.PT[2][3])
                    H1_4 = dot(linalg.inv(H0_1), H0_4)
                    H2_4 = dot(linalg.inv(H1_2), H1_4)
                    H3_4 = dot(linalg.inv(H2_3), H2_4)
                    self.t4.append(math.atan2(H3_4[1][0], H3_4[0][0]))
        except ValueError:
            print("Domain Value Error:")

    # determination of arm pose vertices via forward kinematics, given angle vals

    def draw_limbs(self, tbn=0):

        t1 = 0
        t2 = 0
        t3 = 0
        t4 = 0
        t5 = 0
        t6 = 0
        # pose angle combinations

        if tbn == 1:
            t1 = self.t1[0]
            t2 = self.t2[0]
            t3 = -self.t3[0]
            t4 = self.t4[0]
            t5 = self.t5[0]
            t6 = self.t6[0]

        if tbn == 2:
            t1 = self.t1[0]
            t2 = self.t2[1]
            t3 = -self.t3[1]
            t4 = self.t4[1]
            t5 = self.t5[0]
            t6 = self.t6[0]

        if tbn == 3:
            t1 = self.t1[0]
            t2 = self.t2[2]
            t3 = -self.t3[2]
            t4 = self.t4[2]
            t5 = self.t5[1]
            t6 = self.t6[1]

        if tbn == 4:
            t1 = self.t1[0]
            t2 = self.t2[3]
            t3 = -self.t3[3]
            t4 = self.t4[3]
            t5 = self.t5[1]
            t6 = self.t6[1]

        if tbn == 5:
            t1 = self.t1[1]
            t2 = self.t2[4]
            t3 = -self.t3[4]
            t4 = self.t4[4]
            t5 = self.t5[2]
            t6 = self.t6[2]

        if tbn == 6:
            t1 = self.t1[1]
            t2 = self.t2[5]
            t3 = -self.t3[5]
            t4 = self.t4[5]
            t5 = self.t5[2]
            t6 = self.t6[2]

        if tbn == 7:
            t1 = self.t1[1]
            t2 = self.t2[6]
            t3 = -self.t3[6]
            t4 = self.t4[6]
            t5 = self.t5[3]
            t6 = self.t6[3]

        if tbn == 8:
            t1 = self.t1[1]
            t2 = self.t2[7]
            t3 = -self.t3[7]
            t4 = self.t4[7]
            t5 = self.t5[3]
            t6 = self.t6[3]

        # Forward Kinematics Denavit Hartenberg Table
        pt = [[t1, 0, 0, self.d1],
              [t2, pi / 2, 0, 0],
              [t3, 0, -self.a2, 0],
              [t4, 0, -self.a3, self.d4],
              [t5, pi / 2, 0, self.d5],
              [t6, -pi / 2, 0, self.d6]]

        # self.fk determines link i frame and link i-1 frame transformation matrices
        fk = []
        for i in range(6):
            fk.append(transform_matrix2(
                pt[i][0], pt[i][1], pt[i][2], pt[i][3]))

        # determination of transformation matrix for link i frame relative to base frame
        H0_1 = fk[0]
        H0_2 = dot(fk[0], fk[1])
        H0_3 = dot(H0_2, fk[2])
        H0_4 = dot(H0_3, fk[3])
        H0_5 = dot(H0_4, fk[4])
        H0_6 = dot(H0_5, fk[5])

        # stores limb vertex locations in x,y,z relative to base frame
        limb_vertices = [[H0_1[0][3], H0_1[1][3], H0_1[2][3]],
                         [H0_2[0][3], H0_2[1][3], H0_2[2][3]],
                         [H0_3[0][3], H0_3[1][3], H0_3[2][3]],
                         [H0_4[0][3], H0_4[1][3], H0_4[2][3]],
                         [H0_5[0][3], H0_5[1][3], H0_5[2][3]],
                         [H0_6[0][3], H0_6[1][3], H0_6[2][3]],
                         ]
        limb_vectors = [[0, 0, 0]]
        for i, limb, in enumerate(limb_vertices):
            limb_vectors.append(limb_vertices[i])
        # limb_vectors = [[3, 2, 2], [14, 13, 51], [43, 53, 75]]
        x_data = [vector[0] for vector in limb_vectors]
        y_data = [vector[1] for vector in limb_vectors]
        z_data = [vector[2] for vector in limb_vectors]
        self.ax.plot(x_data, y_data, z_data)

    def rtnposeang(self, j, i):
        if j == 1:
            return self.t1[i]
        if j == 2:
            return self.t2[i]
        if j == 3:
            return -self.t3[i]
        if j == 4:
            return self.t4[i]
        if j == 5:
            return self.t5[i]
        if j == 6:
            return self.t6[i]


def RXT(theta):
    rx = [[1, 0, 0],
          [0, cos(theta), -(sin(theta))],
          [0, sin(theta), cos(theta)]]
    return rx


def RYT(theta):
    ry = [[cos(theta), 0, sin(theta)],
          [0, 1, 0],
          [-(sin(theta)), 0, cos(theta)]]
    return ry


def RZT(theta):
    rz = [[cos(theta), -(sin(theta)), 0],
          [sin(theta), cos(theta), 0],
          [0, 0, 1]]
    return rz


# noinspection PyAttributeOutsideInit
class Application(tk.Frame):
    def __init__(self, window):
        super().__init__(window)
        self.fig = Figure(figsize=(6, 5), dpi=150)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.window = window
        self.pack(side="top")
        self.create_widgets()
        self.IKplt()

    def PreviousArmU(self, T1, T2, T3, T4, T5, T6):
        self.A1 = T1
        self.A2 = T2
        self.A3 = T3
        self.A4 = T4
        self.A5 = T5
        self.A6 = T6

    def PreviousArmD(self, ArmNum):
        if ArmNum == 1:
            return self.A1
        elif ArmNum == 2:
            return self.A2
        elif ArmNum == 3:
            return self.A3
        elif ArmNum == 4:
            return self.A4
        elif ArmNum == 5:
            return self.A5
        elif ArmNum == 6:
            return self.A6

    def IKupdate(self, r11, r12, r13, x, r21, r22, r23, y, r31, r32, r33, z):
        self.ax.clear()
        WINDOW_SIZE = 500
        self.ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_zlim3d(0, WINDOW_SIZE)

        self.ax.set_xlabel('x (mm)')
        self.ax.set_ylabel('y (mm)')
        self.ax.set_zlabel('z (mm)')

        self.robot = IKSixR(self.ax, H06=(
            r11, r12, r13, x, r21, r22, r23, y, r31, r32, r33, z))
        self.robot.IK()
        try:
            self.robot.draw_limbs(tbn=1)
            self.robot.draw_limbs(tbn=2)
            self.robot.draw_limbs(tbn=3)
            self.robot.draw_limbs(tbn=4)
            self.robot.draw_limbs(tbn=5)
            self.robot.draw_limbs(tbn=6)
            self.robot.draw_limbs(tbn=7)
            self.robot.draw_limbs(tbn=8)
            self.OOCSLabel["text"] = ""
        except IndexError:
            self.OOCSLabel["text"] = "Out of Config Space!"
            print(" Out of config space")
        self.toolbar.update()
        self.canvas.draw()

    def FKupdate(self, T1, T2, T3, T4, T5, T6):
        self.ax.clear()
        WINDOW_SIZE = 500
        self.ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_zlim3d(0, WINDOW_SIZE)

        self.ax.set_xlabel('x (mm)')
        self.ax.set_ylabel('y (mm)')
        self.ax.set_zlabel('z (mm)')
        robot = FKSixR(self.ax, theta=(T1, T2, T3, T4, T5, T6))

        robot.draw_limbs()
        self.toolbar.update()
        self.canvas.draw()

    def create_widgets(self):

        frame0 = tk.Frame(master=window, height=50, borderwidth=5)
        frame0.pack(side="top", fill="x")

        frame1 = tk.Frame(master=window, height=50, borderwidth=5)
        frame1.pack(side="top", fill="x")

        frame2 = tk.Frame(master=window, height=50, borderwidth=5)
        frame2.pack(side="top", fill="x")

        frame3 = tk.Frame(master=window, height=50, borderwidth=5)
        frame3.pack(side="top", fill="x")

        frame4 = tk.Frame(master=window, height=50, borderwidth=5)
        frame4.pack(side="top", fill="x")

        frame5 = tk.Frame(master=window, height=50, borderwidth=5)
        frame5.pack(side="top", fill="x")

        frame6 = tk.Frame(master=window, height=50, borderwidth=5)
        frame6.pack(side="top", fill="x")

        frame7 = tk.Frame(master=window, height=50, borderwidth=5)
        frame7.pack(side="top", fill="x")

        frame8 = tk.Frame(master=window, height=50, borderwidth=5)
        frame8.pack(side="top", fill="x")

        self.iter = tk.Text(master=frame0, height=1, width=20)
        self.iter.insert(tk.END, "1")
        self.iter.pack(side="left")
        iterlabel = tk.Label(
            master=frame0, text="Increment/Decrement Step Size")
        iterlabel.pack(side="left")

        RCtrlInter = tk.Label(self, text="Robot Control Interface")
        RCtrlInter.pack(side="top")

        Update = tk.Button(master=frame7, command=self.FKassign)
        Update["text"] = "Update FK"
        Update.pack(side="left")

        Update2 = tk.Button(master=frame7, command=self.IKassign)
        Update2["text"] = "Update IK"
        Update2.pack(side="right")

        self.Update3 = tk.Button(
            master=frame8, state="disabled", command=self.push)
        self.Update3["text"] = "Push FK"
        self.Update3.pack(side="left")

        blue = tk.Button(master=frame8, bg="blue",
                         fg="white", command=self.blue)
        blue["text"] = "blue"
        blue.pack(side="right")

        orange = tk.Button(master=frame8, bg="orange",
                           fg="white", command=self.orange)
        orange["text"] = "orange"
        orange.pack(side="right")

        green = tk.Button(master=frame8, bg="green",
                          fg="white", command=self.green)
        green["text"] = "green"
        green.pack(side="right")

        red = tk.Button(master=frame8, bg="red", fg="white", command=self.red)
        red["text"] = "red"
        red.pack(side="right")

        purple = tk.Button(master=frame8, bg="purple",
                           fg="white", command=self.purple)
        purple["text"] = "purple"
        purple.pack(side="right")

        brown = tk.Button(master=frame8, bg="brown",
                          fg="white", command=self.brown)
        brown["text"] = "brown"
        brown.pack(side="right")

        pink = tk.Button(master=frame8, bg="pink",
                         fg="white", command=self.pink)
        pink["text"] = "pink"
        pink.pack(side="right")

        grey = tk.Button(master=frame8, bg="grey",
                         fg="white", command=self.grey)
        grey["text"] = "grey"
        grey.pack(side="right")

        posetransfer = tk.Label(master=frame8)
        posetransfer["text"] = "Transfer Pose"
        posetransfer.pack(side="right")

        T1L = tk.Label(master=frame1, text="Theta 1", height=1, width=10)
        T1L.pack(side="left")

        btn_decreaseT1 = tk.Button(
            master=frame1, text="-", command=self.decrease1)
        btn_decreaseT1.pack(side="left")

        self.T1 = tk.Text(master=frame1, height=1, width=20)
        self.T1.insert(tk.END, "0")
        self.T1.pack(side="left")

        btn_increaseT1 = tk.Button(
            master=frame1, text="+", command=self.increase1)
        btn_increaseT1.pack(side="left")

        self.previousPoseT1 = tk.Label(master=frame1, text="0.0")
        self.previousPoseT1.pack(side="left")

        XL = tk.Label(master=frame1, text="X(mm)", height=1, width=10)
        XL.pack(side="right")

        btn_increaseX = tk.Button(
            master=frame1, text="+", command=self.increaseX)
        btn_increaseX.pack(side="right")

        self.X = tk.Text(master=frame1, height=1, width=20)
        self.X.insert(tk.END, "-100")
        self.X.pack(side="right")

        btn_decreaseX = tk.Button(
            master=frame1, text="-", command=self.decreaseX)
        btn_decreaseX.pack(side="right")

        T2L = tk.Label(master=frame2, text="Theta 2", height=1, width=10)
        T2L.pack(side="left")

        btn_decreaseT2 = tk.Button(
            master=frame2, text="-", command=self.decrease2)
        btn_decreaseT2.pack(side="left")

        self.T2 = tk.Text(master=frame2, height=1, width=20)
        self.T2.insert(tk.END, "-90")
        self.T2.pack(side="left")

        btn_increaseT2 = tk.Button(
            master=frame2, text="+", command=self.increase2)
        btn_increaseT2.pack(side="left")

        self.previousPoseT2 = tk.Label(master=frame2, text="-90.0")
        self.previousPoseT2.pack(side="left")

        YL = tk.Label(master=frame2, text="Y(mm)", height=1, width=10)
        YL.pack(side="right")

        btn_increaseY = tk.Button(
            master=frame2, text="+", command=self.increaseY)
        btn_increaseY.pack(side="right")

        self.Y = tk.Text(master=frame2, height=1, width=20)
        self.Y.insert(tk.END, "-100")
        self.Y.pack(side="right")

        btn_decreaseY = tk.Button(
            master=frame2, text="-", command=self.decreaseY)
        btn_decreaseY.pack(side="right")

        T3L = tk.Label(master=frame3, text="Theta 3", height=1, width=10)
        T3L.pack(side="left")

        btn_decreaseT3 = tk.Button(
            master=frame3, text="-", command=self.decrease3)
        btn_decreaseT3.pack(side="left")

        self.T3 = tk.Text(master=frame3, height=1, width=20)
        self.T3.insert(tk.END, "0")
        self.T3.pack(side="left")

        btn_increaseT3 = tk.Button(
            master=frame3, text="+", command=self.increase3)
        btn_increaseT3.pack(side="left")

        self.previousPoseT3 = tk.Label(master=frame3, text="0.0")
        self.previousPoseT3.pack(side="left")

        ZL = tk.Label(master=frame3, text="Z(mm)", height=1, width=10)
        ZL.pack(side="right")

        btn_increaseZ = tk.Button(
            master=frame3, text="+", command=self.increaseZ)
        btn_increaseZ.pack(side="right")

        self.Z = tk.Text(master=frame3, height=1, width=20)
        self.Z.insert(tk.END, "100")
        self.Z.pack(side="right")

        btn_decreaseZ = tk.Button(
            master=frame3, text="-", command=self.decreaseZ)
        btn_decreaseZ.pack(side="right")

        T4L = tk.Label(master=frame4, text="Theta 4", height=1, width=10)
        T4L.pack(side="left")

        btn_decreaseT4 = tk.Button(
            master=frame4, text="-", command=self.decrease4)
        btn_decreaseT4.pack(side="left")

        self.T4 = tk.Text(master=frame4, height=1, width=20)
        self.T4.insert(tk.END, "-90")
        self.T4.pack(side="left")

        btn_increaseT4 = tk.Button(
            master=frame4, text="+", command=self.increase4)
        btn_increaseT4.pack(side="left")

        self.previousPoseT4 = tk.Label(master=frame4, text="-90.0")
        self.previousPoseT4.pack(side="left")

        R1L = tk.Label(master=frame4, text="R1(theta)", height=1, width=10)
        R1L.pack(side="right")

        btn_increaseR1 = tk.Button(
            master=frame4, text="+", command=self.increaseR1)
        btn_increaseR1.pack(side="right")

        self.R1 = tk.Text(master=frame4, height=1, width=20)
        self.R1.insert(tk.END, "90")
        self.R1.pack(side="right")

        btn_decreaseR1 = tk.Button(
            master=frame4, text="-", command=self.decreaseR1)
        btn_decreaseR1.pack(side="right")

        T5L = tk.Label(master=frame5, text="Theta 5", height=1, width=10)
        T5L.pack(side="left")

        btn_decreaseT5 = tk.Button(
            master=frame5, text="-", command=self.decrease5)
        btn_decreaseT5.pack(side="left")

        self.T5 = tk.Text(master=frame5, height=1, width=20)
        self.T5.insert(tk.END, "0")
        self.T5.pack(side="left")

        btn_increaseT5 = tk.Button(
            master=frame5, text="+", command=self.increase5)
        btn_increaseT5.pack(side="left")

        self.previousPoseT5 = tk.Label(master=frame5, text="0.0")
        self.previousPoseT5.pack(side="left")

        R2L = tk.Label(master=frame5, text="R2(theta)", height=1, width=10)
        R2L.pack(side="right")

        btn_increaseR2 = tk.Button(
            master=frame5, text="+", command=self.increaseR2)
        btn_increaseR2.pack(side="right")

        self.R2 = tk.Text(master=frame5, height=1, width=20)
        self.R2.insert(tk.END, "0")
        self.R2.pack(side="right")

        btn_decreaseR2 = tk.Button(
            master=frame5, text="-", command=self.decreaseR2)
        btn_decreaseR2.pack(side="right")

        T6L = tk.Label(master=frame6, text="Theta 6", height=1, width=10)
        T6L.pack(side="left")

        btn_decreaseT6 = tk.Button(
            master=frame6, text="-", command=self.decrease6)
        btn_decreaseT6.pack(side="left")

        self.T6 = tk.Text(master=frame6, height=1, width=20)
        self.T6.insert(tk.END, "0")
        self.T6.pack(side="left")

        btn_increaseT6 = tk.Button(
            master=frame6, text="+", command=self.increase6)
        btn_increaseT6.pack(side="left")

        self.previousPoseT6 = tk.Label(master=frame6, text="0.0")
        self.previousPoseT6.pack(side="left")

        R3L = tk.Label(master=frame6, text="R3(theta)", height=1, width=10)
        R3L.pack(side="right")

        btn_increaseR3 = tk.Button(
            master=frame6, text="+", command=self.increaseR3)
        btn_increaseR3.pack(side="right")

        self.R3 = tk.Text(master=frame6, height=1, width=20)
        self.R3.insert(tk.END, "90")
        self.R3.pack(side="right")

        btn_decreaseR3 = tk.Button(
            master=frame6, text="-", command=self.decreaseR3)
        btn_decreaseR3.pack(side="right")

        self.OOCSLabel = tk.Label(master=frame7, text="", fg="red")
        self.OOCSLabel.pack()

    def FKinsert(self, a1, a2, a3, a4, a5, a6):
        self.T1.delete("1.0", tk.END)
        self.T2.delete("1.0", tk.END)
        self.T3.delete("1.0", tk.END)
        self.T4.delete("1.0", tk.END)
        self.T5.delete("1.0", tk.END)
        self.T6.delete("1.0", tk.END)
        self.T1.insert(tk.END, str(a1 * 180 / pi))
        self.T2.insert(tk.END, str(a2 * 180 / pi))
        self.T3.insert(tk.END, str(a3 * 180 / pi))
        self.T4.insert(tk.END, str(a4 * 180 / pi))
        self.T5.insert(tk.END, str(a5 * 180 / pi))
        self.T6.insert(tk.END, str(a6 * 180 / pi))
        self.FKupdate(a1 * 180 / pi, a2 * 180 / pi, a3 * 180 / pi, a4 * 180 / pi, a5 * 180 / pi,
                      a6 * 180 / pi)

    def decrease1(self):
        value = float(self.T1.get("1.0", tk.END))
        self.T1.delete("1.0", tk.END)
        self.T1.insert(tk.END, str(
            value - float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def increase1(self):
        value = float(self.T1.get("1.0", tk.END))
        self.T1.delete("1.0", tk.END)
        self.T1.insert(tk.END, str(
            value + float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def decrease2(self):
        value = float(self.T2.get("1.0", tk.END))
        self.T2.delete("1.0", tk.END)
        self.T2.insert(tk.END, str(
            value - float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def increase2(self):
        value = float(self.T2.get("1.0", tk.END))
        self.T2.delete("1.0", tk.END)
        self.T2.insert(tk.END, str(
            value + float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def decrease3(self):
        value = float(self.T3.get("1.0", tk.END))
        self.T3.delete("1.0", tk.END)
        self.T3.insert(tk.END, str(
            value - float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def increase3(self):
        value = float(self.T3.get("1.0", tk.END))
        self.T3.delete("1.0", tk.END)
        self.T3.insert(tk.END, str(
            value + float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def decrease4(self):
        value = float(self.T4.get("1.0", tk.END))
        self.T4.delete("1.0", tk.END)
        self.T4.insert(tk.END, str(
            value - float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def increase4(self):
        value = float(self.T4.get("1.0", tk.END))
        self.T4.delete("1.0", tk.END)
        self.T4.insert(tk.END, str(
            value + float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def decrease5(self):
        value = float(self.T5.get("1.0", tk.END))
        self.T5.delete("1.0", tk.END)
        self.T5.insert(tk.END, str(
            value - float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def increase5(self):
        value = float(self.T5.get("1.0", tk.END))
        self.T5.delete("1.0", tk.END)
        self.T5.insert(tk.END, str(
            value + float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def decrease6(self):
        value = float(self.T6.get("1.0", tk.END))
        self.T6.delete("1.0", tk.END)
        self.T6.insert(tk.END, str(
            value - float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def increase6(self):
        value = float(self.T6.get("1.0", tk.END))
        self.T6.delete("1.0", tk.END)
        self.T6.insert(tk.END, str(
            value + float(self.iter.get("1.0", tk.END))))
        self.FKassign()

    def increaseX(self):
        value = float(self.X.get("1.0", tk.END))
        self.X.delete("1.0", tk.END)
        self.X.insert(tk.END, str(value + float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def decreaseX(self):
        value = float(self.X.get("1.0", tk.END))
        self.X.delete("1.0", tk.END)
        self.X.insert(tk.END, str(value - float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def increaseY(self):
        value = float(self.Y.get("1.0", tk.END))
        self.Y.delete("1.0", tk.END)
        self.Y.insert(tk.END, str(value + float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def decreaseY(self):
        value = float(self.Y.get("1.0", tk.END))
        self.Y.delete("1.0", tk.END)
        self.Y.insert(tk.END, str(value - float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def increaseZ(self):
        value = float(self.Z.get("1.0", tk.END))
        self.Z.delete("1.0", tk.END)
        self.Z.insert(tk.END, str(value + float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def decreaseZ(self):
        value = float(self.Z.get("1.0", tk.END))
        self.Z.delete("1.0", tk.END)
        self.Z.insert(tk.END, str(value - float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def increaseR1(self):
        value = float(self.R1.get("1.0", tk.END))
        self.R1.delete("1.0", tk.END)
        self.R1.insert(tk.END, str(
            value + float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def decreaseR1(self):
        value = float(self.R1.get("1.0", tk.END))
        self.R1.delete("1.0", tk.END)
        self.R1.insert(tk.END, str(
            value - float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def increaseR2(self):
        value = float(self.R2.get("1.0", tk.END))
        self.R2.delete("1.0", tk.END)
        self.R2.insert(tk.END, str(
            value + float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def decreaseR2(self):
        value = float(self.R2.get("1.0", tk.END))
        self.R2.delete("1.0", tk.END)
        self.R2.insert(tk.END, str(
            value - float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def increaseR3(self):
        value = float(self.R3.get("1.0", tk.END))
        self.R3.delete("1.0", tk.END)
        self.R3.insert(tk.END, str(
            value + float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def decreaseR3(self):
        value = float(self.R3.get("1.0", tk.END))
        self.R3.delete("1.0", tk.END)
        self.R3.insert(tk.END, str(
            value - float(self.iter.get("1.0", tk.END))))
        self.IKassign()

    def blue(self):
        self.FKinsert(self.robot.rtnposeang(1, 0), self.robot.rtnposeang(2, 0), self.robot.rtnposeang(3, 0),
                      self.robot.rtnposeang(4, 0), self.robot.rtnposeang(5, 0), self.robot.rtnposeang(6, 0))

    def orange(self):
        self.FKinsert(self.robot.rtnposeang(1, 0), self.robot.rtnposeang(2, 1), self.robot.rtnposeang(3, 1),
                      self.robot.rtnposeang(4, 1), self.robot.rtnposeang(5, 0), self.robot.rtnposeang(6, 0))

    def green(self):
        self.FKinsert(self.robot.rtnposeang(1, 0), self.robot.rtnposeang(2, 2), self.robot.rtnposeang(3, 2),
                      self.robot.rtnposeang(4, 2), self.robot.rtnposeang(5, 1), self.robot.rtnposeang(6, 1))

    def red(self):
        self.FKinsert(self.robot.rtnposeang(1, 0), self.robot.rtnposeang(2, 3), self.robot.rtnposeang(3, 3),
                      self.robot.rtnposeang(4, 3), self.robot.rtnposeang(5, 1), self.robot.rtnposeang(6, 1))

    def purple(self):
        self.FKinsert(self.robot.rtnposeang(1, 1), self.robot.rtnposeang(2, 4), self.robot.rtnposeang(3, 4),
                      self.robot.rtnposeang(4, 4), self.robot.rtnposeang(5, 2), self.robot.rtnposeang(6, 2))

    def brown(self):
        self.FKinsert(self.robot.rtnposeang(1, 1), self.robot.rtnposeang(2, 5), self.robot.rtnposeang(3, 5),
                      self.robot.rtnposeang(4, 5), self.robot.rtnposeang(5, 2), self.robot.rtnposeang(6, 2))

    def pink(self):
        self.FKinsert(self.robot.rtnposeang(1, 1), self.robot.rtnposeang(2, 6), self.robot.rtnposeang(3, 6),
                      self.robot.rtnposeang(4, 6), self.robot.rtnposeang(5, 3), self.robot.rtnposeang(6, 3))

    def grey(self):
        self.FKinsert(self.robot.rtnposeang(1, 1), self.robot.rtnposeang(2, 7), self.robot.rtnposeang(3, 7),
                      self.robot.rtnposeang(4, 7), self.robot.rtnposeang(5, 3), self.robot.rtnposeang(6, 3))

    def push(self):
        C1 = str(int((float(self.T1.get("1.0", tk.END)) -
                 float(self.previousPoseT1["text"])) * 7680 / 360))
        C2 = str(int((float(self.T2.get("1.0", tk.END)) -
                 float(self.previousPoseT2["text"])) * 7680 / 360))
        C3 = str(int(-(float(self.T3.get("1.0", tk.END)) -
                 float(self.previousPoseT3["text"])) * 7680 / 360))
        C4 = str(int(-(float(self.T4.get("1.0", tk.END)) -
                 float(self.previousPoseT4["text"])) * 7680 / 360))
        C5 = str(int(-(float(self.T5.get("1.0", tk.END)) -
                 float(self.previousPoseT5["text"])) * 7680 / 360))
        C6 = str(int((float(self.T6.get("1.0", tk.END)) -
                 float(self.previousPoseT6["text"])) * 7680 / 360))
        # mvmt(C1=C1, C2=C2, C3=C3, C4=C4, C5=C5, C6=C6)

        self.previousPoseT1["text"] = float(self.T1.get("1.0", tk.END))
        self.previousPoseT2["text"] = float(self.T2.get("1.0", tk.END))
        self.previousPoseT3["text"] = float(self.T3.get("1.0", tk.END))
        self.previousPoseT4["text"] = float(self.T4.get("1.0", tk.END))
        self.previousPoseT5["text"] = float(self.T5.get("1.0", tk.END))
        self.previousPoseT6["text"] = float(self.T6.get("1.0", tk.END))

    def IKassign(self):
        rotation_TbTmatrix = dot(RXT(float(self.R1.get("1.0", tk.END)) * pi / 180),
                                 dot(RYT(float(self.R2.get("1.0", tk.END)) * pi / 180),
                                     RZT(float(self.R3.get("1.0", tk.END)) * pi / 180)))

        self.IKupdate(r11=rotation_TbTmatrix[0][0],
                      r12=rotation_TbTmatrix[0][1],
                      r13=rotation_TbTmatrix[0][2],
                      x=float(self.X.get("1.0", tk.END)),
                      r21=rotation_TbTmatrix[1][0],
                      r22=rotation_TbTmatrix[1][1],
                      r23=rotation_TbTmatrix[1][2],
                      y=float(self.Y.get("1.0", tk.END)),
                      r31=rotation_TbTmatrix[2][0],
                      r32=rotation_TbTmatrix[2][1],
                      r33=rotation_TbTmatrix[2][2],
                      z=float(self.Z.get("1.0", tk.END)))
        self.Update3["state"] = "disabled"

    def FKassign(self):
        self.FKupdate(T1=float(self.T1.get("1.0", tk.END)), T2=float(self.T2.get("1.0", tk.END)),
                      T3=float(self.T3.get("1.0", tk.END)), T4=float(self.T4.get("1.0", tk.END)),
                      T5=float(self.T5.get("1.0", tk.END)), T6=float(self.T6.get("1.0", tk.END)))
        self.Update3["state"] = "normal"

    def IKplt(self):
        fig = Figure(figsize=(6, 5), dpi=150)

        # A tk.DrawingArea.
        self.canvas = FigureCanvasTkAgg(fig, master=window)
        self.canvas.draw()

        self.ax = fig.add_subplot(111, projection="3d")
        WINDOW_SIZE = 500
        self.ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_zlim3d(0, WINDOW_SIZE)

        self.ax.set_xlabel('x (mm)')
        self.ax.set_ylabel('y (mm)')
        self.ax.set_zlabel('z (mm)')

        r11 = 8.59293961e-01
        r12 = -4.61824089e-01
        r13 = 2.19846310e-01
        x = float(-.9e+02)
        r21 = 5.93911746e-02
        r22 = -3.36824089e-01
        r23 = -9.39692621e-01
        y = float(-1e+02)
        r31 = 5.08022222e-01
        r32 = 8.20529125e-01
        r33 = -2.62002630e-01
        z = float(1.5e+02)

        self.robot = IKSixR(self.ax, H06=(
            r11, r12, r13, x, r21, r22, r23, y, r31, r32, r33, z))
        self.robot.IK()
        try:
            self.robot.draw_limbs(tbn=1)
            self.robot.draw_limbs(tbn=2)
            self.robot.draw_limbs(tbn=3)
            self.robot.draw_limbs(tbn=4)
            self.robot.draw_limbs(tbn=5)
            self.robot.draw_limbs(tbn=6)
            self.robot.draw_limbs(tbn=7)
            self.robot.draw_limbs(tbn=8)
            self.OOCSLabel["text"] = ""

        except IndexError:
            self.OOCSLabel["text"] = "Out of Config Space!"
            print(" Out of config space")

        self.toolbar = NavigationToolbar2Tk(self.canvas, window)
        self.toolbar.update()
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand="true")


window = tk.Tk()
app = Application(window)
window.wm_title("Robot Control Interface")
window.mainloop()
