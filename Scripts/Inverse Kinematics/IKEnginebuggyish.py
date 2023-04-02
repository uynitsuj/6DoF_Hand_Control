
from numpy import *
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
import pyqtgraph as pg


def transform_matrix2(ti, alpha_i_m1, a_i_m1, di):
    return [[cos(ti), -sin(ti), 0, a_i_m1],
            [sin(ti) * cos(alpha_i_m1), cos(ti) * cos(alpha_i_m1), -
             sin(alpha_i_m1), -sin(alpha_i_m1) * di],
            [sin(ti) * sin(alpha_i_m1), cos(ti) * sin(alpha_i_m1),
             cos(alpha_i_m1), cos(alpha_i_m1) * di],
            [0, 0, 0, 1]]


class IKSixR:

    def __init__(self, w, H06=(1, 0, 0, -3.07495448e+02, 0, 0, -1, -1.21587006e+02, 0, 1, 0, 3.59115360e+01)):
        self.w = w
        self.d1 = 72.39/1000
        self.a2 = 150.114/1000
        self.a3 = 150.114/1000
        self.d4 = 3.556/1000
        self.d5 = 83.82/1000
        self.d6 = 27.178/1000
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
                self.t5.append(math.acos(1))  # should these be val?
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
        # x_data = [vector[0] for vector in limb_vectors]
        # y_data = [vector[1] for vector in limb_vectors]
        # z_data = [vector[2] for vector in limb_vectors]
        self.w.addItem(gl.GLLinePlotItem(
            pos=limb_vectors, color=pg.glColor((3, 50)), width=5, antialias=True))

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
