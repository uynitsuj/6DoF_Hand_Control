import cv2
from multiprocessing import Process, SimpleQueue, shared_memory
import handtracker
import Calibrate.cameracalibrate as cal
import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import sys
import socket
import pickle
from IKEngine import IKSixR
import time
# import copy


def normalize_quaternion(quat):
    norm = np.linalg.norm(quat)
    return tuple(q / norm for q in quat)


def quaternion_to_se3(quat):
    w, x, y, z = quat
    Nq = w*w + x*x + y*y + z*z
    if Nq < np.finfo(float).eps:
        raise ValueError("Input quaternion has zero length.")

    s = 2.0 / Nq
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs

    se3 = np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy, 0],
        [xy + wz, 1.0 - (xx + zz), yz - wx, 0],
        [xz - wy, yz + wx, 1.0 - (xx + yy), 0],
        [0, 0, 0, 1]
    ])

    return se3


class Visualizer(object):
    def __init__(self):
        self.lm3d = []
        self.lm3dil = []
        self.lm3dir = []
        self.pose = []
        self.pfilter = 0.05
        self.app = QtWidgets.QApplication(sys.argv)
        self.w = gl.GLViewWidget()
        self.w.orbit(45, 1)
        self.w.opts['distance'] = 1
        self.w.setWindowTitle('pyqtgraph Hand Pose')
        self.w.setGeometry(1000, 500, 800, 500)
        self.w.show()
        self.setup()

    def setup(self):
        gsz = 1
        gsp = .1
        gx = gl.GLGridItem(color=(255, 255, 255, 60))
        gx.setSize(gsz, gsz, gsz)
        gx.setSpacing(gsp, gsp, gsp)
        gx.rotate(90, 0, 1, 0)
        gx.translate(-gsz/2, 0, gsz/2)
        self.w.addItem(gx)
        gy = gl.GLGridItem(color=(255, 255, 255, 60))
        gy.setSize(gsz, gsz, gsz)
        gy.setSpacing(gsp, gsp, gsp)
        gy.rotate(90, 1, 0, 0)
        gy.translate(0, -gsz/2, gsz/2)
        self.w.addItem(gy)
        gz = gl.GLGridItem(color=(255, 255, 255, 100))
        gz.setSize(gsz, gsz, gsz)
        gz.setSpacing(gsp, gsp, gsp)
        self.w.addItem(gz)

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtWidgets.QApplication.instance().exec_()

    def update(self):
        del self.w.items[:]
        self.w.clear()
        self.setup()
        lm3dlist = np.ndarray((21, 3), dtype=np.float64)
        self.w.opts['azimuth'] += 0.05
        if True:
            desired = np.ndarray((21, 3), dtype=np.float64,
                                 buffer=self.lm3d.buf)
            if self.pfilter:
                lm3dlist += self.pfilter * (desired - lm3dlist)
            else:
                lm3dlist = desired
            if lm3dlist.tolist():

                # print(lm3dlist)
                width = 10

                # Thumb
                self.w.addItem(gl.GLLinePlotItem(
                    pos=lm3dlist[0:5], color=pg.glColor((4, 100)), width=width, antialias=True))
                # Index
                self.w.addItem(gl.GLLinePlotItem(
                    pos=np.append([lm3dlist[0]], lm3dlist[5:9], axis=0), color=pg.glColor((4, 100)), width=width, antialias=True))
                # Middle
                self.w.addItem(gl.GLLinePlotItem(
                    pos=lm3dlist[9:13], color=pg.glColor((4, 100)), width=width, antialias=True))
                # Ring
                self.w.addItem(gl.GLLinePlotItem(
                    pos=lm3dlist[13:17], color=pg.glColor((4, 100)), width=width, antialias=True))
                # Pinky
                self.w.addItem(gl.GLLinePlotItem(
                    pos=np.append([lm3dlist[0]],
                                  lm3dlist[17:21], axis=0), color=pg.glColor((4, 100)), width=width, antialias=True))
                # Knuckle
                knuckle = np.append([lm3dlist[5]], [lm3dlist[9]], axis=0)
                knuckle = np.append(knuckle, [lm3dlist[13]], axis=0)
                knuckle = np.append(knuckle, [lm3dlist[17]], axis=0)
                self.w.addItem(gl.GLLinePlotItem(
                    pos=knuckle, color=pg.glColor((4, 100)), width=width, antialias=True))

                self.w.addItem(gl.GLScatterPlotItem(
                    pos=lm3dlist, color=pg.glColor((2, 50)), size=28))
            '''
            lm3dlist = np.ndarray((21, 3), dtype=np.float64,
                                  buffer=self.lm3dil.buf)

            if lm3dlist.tolist():

                # print(lm3dlist)
                width = 30

                # Thumb
                self.w.addItem(gl.GLLinePlotItem(
                    pos=lm3dlist[0:5], color=pg.glColor((4, 100)), width=width, antialias=True))
                # Index
                self.w.addItem(gl.GLLinePlotItem(
                    pos=np.append([lm3dlist[0]], lm3dlist[5:9], axis=0), color=pg.glColor((4, 100)), width=width, antialias=True))
                # Middle
                self.w.addItem(gl.GLLinePlotItem(
                    pos=lm3dlist[9:13], color=pg.glColor((4, 100)), width=width, antialias=True))
                # Ring
                self.w.addItem(gl.GLLinePlotItem(
                    pos=lm3dlist[13:17], color=pg.glColor((4, 100)), width=width, antialias=True))
                # Pinky
                self.w.addItem(gl.GLLinePlotItem(
                    pos=np.append([lm3dlist[0]],
                                  lm3dlist[17:21], axis=0), color=pg.glColor((4, 100)), width=width, antialias=True))
                # Knuckle
                knuckle = np.append([lm3dlist[5]], [lm3dlist[9]], axis=0)
                knuckle = np.append(knuckle, [lm3dlist[13]], axis=0)
                knuckle = np.append(knuckle, [lm3dlist[17]], axis=0)
                self.w.addItem(gl.GLLinePlotItem(
                    pos=knuckle, color=pg.glColor((4, 100)), width=width, antialias=True))

                self.w.addItem(gl.GLScatterPlotItem(
                    pos=lm3dlist, color=pg.glColor((30, 50)), size=28))

            lm3dlist = np.ndarray((21, 3), dtype=np.float64,
                                  buffer=self.lm3dir.buf)
            if lm3dlist.tolist():

                # print(lm3dlist)
                width = 30

                # Thumb
                self.w.addItem(gl.GLLinePlotItem(
                    pos=lm3dlist[0:5], color=pg.glColor((4, 100)), width=width, antialias=True))
                # Index
                self.w.addItem(gl.GLLinePlotItem(
                    pos=np.append([lm3dlist[0]], lm3dlist[5:9], axis=0), color=pg.glColor((4, 100)), width=width, antialias=True))
                # Middle
                self.w.addItem(gl.GLLinePlotItem(
                    pos=lm3dlist[9:13], color=pg.glColor((4, 100)), width=width, antialias=True))
                # Ring
                self.w.addItem(gl.GLLinePlotItem(
                    pos=lm3dlist[13:17], color=pg.glColor((4, 100)), width=width, antialias=True))
                # Pinky
                self.w.addItem(gl.GLLinePlotItem(
                    pos=np.append([lm3dlist[0]],
                                  lm3dlist[17:21], axis=0), color=pg.glColor((4, 100)), width=width, antialias=True))
                # Knuckle
                knuckle = np.append([lm3dlist[5]], [lm3dlist[9]], axis=0)
                knuckle = np.append(knuckle, [lm3dlist[13]], axis=0)
                knuckle = np.append(knuckle, [lm3dlist[17]], axis=0)
                self.w.addItem(gl.GLLinePlotItem(
                    pos=knuckle, color=pg.glColor((4, 100)), width=width, antialias=True))

                self.w.addItem(gl.GLScatterPlotItem(
                    pos=lm3dlist, color=pg.glColor((30, 50)), size=28))'''

        quatdisp = np.ndarray((1, 7), dtype=np.float64)
        desiredpose = np.ndarray(
            (1, 7), dtype=np.float64, buffer=self.pose.buf)
        if self.pfilter:
            # print("filtered")
            quatdisp += self.pfilter * (desiredpose - quatdisp)
        else:
            quatdisp = desiredpose
        # print(quatdisp)
        if quatdisp[0][0]:
            pose = quaternion_to_se3(normalize_quaternion(
                (quatdisp[0][0], quatdisp[0][1], quatdisp[0][2], quatdisp[0][3])))
            pose[0][3] = quatdisp[0][4]
            pose[1][3] = quatdisp[0][5]
            pose[2][3] = quatdisp[0][6]
            # print(pose)
            width = 10
            w = [pose[0][3], pose[1][3], pose[2][3]]
            v1 = [pose[0][0]/10, pose[1][0]/10, pose[2][0]/10]
            v2 = [pose[0][1]/10, pose[1][1]/10, pose[2][1]/10]
            v3 = [pose[0][2]/10, pose[1][2]/10, pose[2][2]/10]
            v1 = np.append([w], [np.add(w, v1)], axis=0)
            # print(v1)
            v2 = np.append([w], [np.add(w, v2)], axis=0)

            v3 = np.append([w], [np.add(w, v3)], axis=0)

            self.w.addItem(gl.GLLinePlotItem(
                pos=v1, color=pg.glColor((2, 120)), width=width, antialias=True))
            self.w.addItem(gl.GLLinePlotItem(
                pos=v2, color=pg.glColor((4, 50)), width=width, antialias=True))
            self.w.addItem(gl.GLLinePlotItem(
                pos=v3, color=pg.glColor((4, 10)), width=width, antialias=True))

            # Inverse Kinematics

            robot = IKSixR(self.w, H06=(
                pose[0][0], pose[0][1], pose[0][2], pose[0][3], pose[1][0], pose[1][1], pose[1][2], pose[1][3], pose[2][0], pose[2][1], pose[2][2], pose[2][3]))
            robot.IK()
            try:
                # robot.draw_limbs(tbn=1) nah
                # robot.draw_limbs(tbn=2)
                # decent
                # robot.draw_limbs(tbn=3) nah
                # robot.draw_limbs(tbn=4) nah
                # robot.draw_limbs(tbn=5) nah
                # robot.draw_limbs(tbn=6) nah
                robot.draw_limbs(tbn=7)
                # decent
                # robot.draw_limbs(tbn=8) nah
                print(str(robot.rtnposeang(1, 1)*57.29578+180) + " "+str(robot.rtnposeang(2, 6)*57.29578+180)+" "+str(robot.rtnposeang(
                    3, 6)*-57.29578+115)+" "+str(robot.rtnposeang(4, 6)*57.29578+180)+" "+str(robot.rtnposeang(5, 3)*57.29578+180)+" "+str(robot.rtnposeang(6, 3)*57.29578+180))
            except Exception as e:
                print(e)
            # except IndexError:
            #    print(" Out of config space")

            # print("theta1"+robot.rtnposeang(1, 7) + " "+robot.rtnposeang(2, 7)+" "+robot.rtnposeang(
            #    3, 7)+" "+robot.rtnposeang(4, 7)+" "+robot.rtnposeang(5, 7)+" "+robot.rtnposeang(6, 7))'''

    def animation(self, pfilter):
        self.pfilter = pfilter
        self.lm3d = shared_memory.SharedMemory(name='lm3d_q')
        self.lm3dil = shared_memory.SharedMemory(name='lm4_q')
        self.lm3dir = shared_memory.SharedMemory(name='lm5_q')
        self.pose = shared_memory.SharedMemory(name='pose')
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(40)
        self.start()


def so3_to_quaternion(R):
    """
    Convert a rotation matrix R in SO(3) to a quaternion.
    """
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2  # S=4*qw
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S=4*qx
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S=4*qy
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S=4*qz
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return np.array([qw, qx, qy, qz])


def orthonormalize_matrix(M):
    u1 = M[:, 0]
    u2 = M[:, 1] - np.dot(M[:, 1], u1) * u1
    u3 = M[:, 2] - np.dot(M[:, 2], u1) * u1 - np.dot(M[:, 2], u2) * u2

    u1 /= np.linalg.norm(u1)
    u2 /= np.linalg.norm(u2)
    u3 /= np.linalg.norm(u3)

    return np.column_stack((u1, u2, u3))


def send(sok):
    while True:
        time.sleep(60/1000)
        selfpose = shared_memory.SharedMemory(name='pose')
        pose = np.ndarray((1, 7), dtype=np.float64)
        desiredpose = np.ndarray(
            (1, 7), dtype=np.float64, buffer=selfpose.buf)
        # if self.pfilter:
        #    print("filtered")
        pose += 0.05 * (desiredpose - pose)
        # else:
        #    pose = copy.copy(desiredpose)
        posecopy = pose
        # print("Memory size of numpy array in bytes:",
        #      pose.size * pose.itemsize)
        # posecopy = posecopy.tolist()
        data_string = pickle.dumps(posecopy)
        # message = arr.encode()
        # print("posecopy")
        # print(posecopy)
        if pose[0][0]:
            # print(pose[3][3])
            sok.sendall(data_string)


def find_orthonormal_frame(outshm, pfilter: bool):
    lm3dshm = shared_memory.SharedMemory(name='lm3d_q')
    lm3dil = shared_memory.SharedMemory(name='lm4_q')
    lm3dir = shared_memory.SharedMemory(name='lm5_q')
    lm3dlist = np.ndarray((21, 3), dtype=np.float64)
    lm3dillist = np.ndarray((21, 3), dtype=np.float64)
    lm3dirlist = np.ndarray((21, 3), dtype=np.float64)
    desired = np.ndarray((21, 3), dtype=np.float64, buffer=lm3dshm.buf)
    desiredil = np.ndarray((21, 3), dtype=np.float64, buffer=lm3dil.buf)
    desiredir = np.ndarray((21, 3), dtype=np.float64, buffer=lm3dir.buf)
    # print(desired)
    while True:
        if pfilter:
            lm3dlist += 0.01 * (desired - lm3dlist)
            lm3dillist += 0.01 * (desiredil - lm3dillist)
            lm3dirlist += 0.01 * (desiredir - lm3dirlist)
        else:
            lm3dlist = desired
            lm3dillist = desiredil
            lm3dirlist = desiredir
        # print(lm3dlist)
        if lm3dlist.tolist():
            # Wrist
            wrist = lm3dlist[0]
            # print(wrist)
            # Index
            index1 = (lm3dlist[5] - lm3dlist[0])
            indexl = (lm3dillist[5] - lm3dillist[0])
            indexr = (lm3dirlist[5] - lm3dirlist[0])
            if np.linalg.norm(index1) and np.linalg.norm(indexl) and np.linalg.norm(indexr):
                v1 = index1/np.linalg.norm(index1)/10
                v1l = indexl/np.linalg.norm(indexl)/10
                v1r = indexr/np.linalg.norm(indexr)/10
                # Pinky
                pinky = (lm3dlist[17] - lm3dlist[0])
                v2 = pinky - \
                    np.dot((np.dot(v1.T, pinky)/np.dot(v1.T, v1)), v1)
                try:
                    v2 = -v2/np.linalg.norm(v2)/10
                except:
                    print("Divide by zero probably lol")

                pinkyl = (lm3dillist[17] - lm3dillist[0])
                v2l = pinkyl - \
                    np.dot((np.dot(v1l.T, pinkyl)/np.dot(v1l.T, v1l)), v1l)
                try:
                    v2l = -v2l/np.linalg.norm(v2l)/10
                except:
                    print("Divide by zero probably lol")

                pinkyr = (lm3dirlist[17] - lm3dirlist[0])
                v2r = pinkyr - \
                    np.dot((np.dot(v1r.T, pinkyr)/np.dot(v1r.T, v1r)), v1r)
                try:
                    v2r = -v2r/np.linalg.norm(v2r)/10
                except:
                    print("Divide by zero probably lol")

                v3 = np.cross(v1, v2)
                try:
                    v3 = v3/np.linalg.norm(v3)/10
                except:
                    print("Divide by zero probably lol")

                v3l = np.cross(v1l, v2l)
                try:
                    v3l = v3l/np.linalg.norm(v3l)/10
                except:
                    print("Divide by zero probably lol")

                v3r = np.cross(v1r, v2r)
                try:
                    v3r = v3r/np.linalg.norm(v3r)/10
                except:
                    print("Divide by zero probably lol")

                pose1 = [[v1[0], v2[0], v3[0]], [
                    v1[1], v2[1], v3[1]], [v1[2], v2[2], v3[2]]]
                pose1 = np.array(pose1)

                posel = [[v1l[0], v2l[0], v3l[0]], [
                    v1l[1], v2l[1], v3l[1]], [v1l[2], v2l[2], v3l[2]]]
                posel = np.array(posel)

                poser = [[v1r[0], v2r[0], v3r[0]], [
                    v1r[1], v2r[1], v3r[1]], [v1r[2], v2r[2], v3r[2]]]
                poser = np.array(poser)

                pose = np.mean([pose1, posel, poser], axis=0)
                pose = orthonormalize_matrix(pose)
                quat = so3_to_quaternion(pose)
                # print(quat)
                quatdisp = [quat[0], quat[1], quat[2],
                            quat[3], wrist[0], wrist[1], wrist[2]]
                quatdisp = np.array(quatdisp)
                # print(quatdisp)
                buffer = np.ndarray(
                    quatdisp.shape, dtype=np.float64, buffer=outshm.buf)
                buffer[:] = quatdisp[:]


def stereo_process(outshm, mtx, b) -> None:
    """
    Takes two landmark list queue objects and comput_manyes the stereoscopic projection.
    Intended to be used in a multiprocessing Process callback.
    Result is a list of vectors with translation position. Stereo camera center is the origin frame.
    :param queue1: landmark list for camera device 1
    :param queue2: landmark list for camera device 2
    :param queueout: stereoscopic projection, a list of 3-vectors
    """
    fx = mtx[0][0]
    fy = mtx[1][1]
    ox = mtx[0][2]
    oy = mtx[1][2]

    shm1 = shared_memory.SharedMemory(name='lm1_q')
    shm2 = shared_memory.SharedMemory(name='lm2_q')
    while True:
        lmlist1 = np.ndarray((21, 2), dtype=np.int32, buffer=shm1.buf)
        lmlist2 = np.ndarray((21, 2), dtype=np.int32, buffer=shm2.buf)
        # print(lmlist1)
        # iterate and calculate disparity between corresponding landmarks
        xyz = []
        if lmlist1.tolist() and lmlist2.tolist():
            lmcat = np.concatenate(
                (np.array(lmlist1), np.array(lmlist2)), axis=1)
            for idx, lm in enumerate(lmcat):
                # comput_manye disparity for each landmark, then find x, y, z
                ur = lm[0]
                vr = lm[1]
                ul = lm[2]
                vl = lm[3]
                d = ul - ur
                if not d:
                    d = 1
                x = b*(ul-ox)/(d)
                y = b*fx*(vl-oy)/(fy*(d))
                z = b*fx/(d)
                xyz.append([-x, y, -z])
        xyz = np.array(xyz)
        # print(xyz)

        buffer = np.ndarray(xyz.shape, dtype=np.float64, buffer=outshm.buf)
        buffer[:] = xyz[:]


def updateHandTrack(capid: int, shm, shm3d, mtx, dist, newcameramtx, roi, imshow=False) -> None:
    """
    Update loop for hand tracker pipeline.
    Intended to be used in a multiprocessing Process callback.
    :param capid: capture device ID as a valid parameter to cv2.VideoCapture()
    :param queue: multiprocessing Queue() object. Queue is updated with hand landmark list
    """
    tracker = handtracker.handTracker()
    cap = cv2.VideoCapture(capid)

    # cap.set(cv2.CAP_PROP_FPS, 15)
    while True:
        _, dst = cap.read()

        # dst = cv2.undistort(image, mtx, dist, None, newcameramtx)
        # x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]
        dst = cv2.flip(dst, 0)
        dst = tracker.handsFinder(dst)
        lmList = tracker.positionFinder(dst, draw=False)
        lmList = np.array(lmList)
        lm3dlist = tracker.find3D()
        lm3dlist = np.array(lm3dlist).reshape(21, 3)

        # print(np.array(lm3dlist).shape)
        # print(np.array(lm3dlist).dtype)
        # print(np.array(lm3dlist).nbytes)

        buffer = np.ndarray(lmList.shape, dtype=np.int32, buffer=shm.buf)
        buffer[:] = lmList[:]

        buffer3d = np.ndarray(
            lm3dlist.shape, dtype=np.float64, buffer=shm3d.buf)
        buffer3d[:] = lm3dlist[:]

        if imshow:
            cv2.imshow("Video", dst)
            cv2.waitKey(1)


def main():
    try:
        lm1_q = shared_memory.SharedMemory(name='lm1_q',
                                           create=True, size=8)
        lm2_q = shared_memory.SharedMemory(name='lm2_q',
                                           create=True, size=8)
        lm3d_q = shared_memory.SharedMemory(
            name='lm3d_q', create=True, size=507)

        lm4_q = shared_memory.SharedMemory(
            name='lm4_q', create=True, size=507)

        lm5_q = shared_memory.SharedMemory(
            name='lm5_q', create=True, size=507)

        pose = shared_memory.SharedMemory(
            name='pose', create=True, size=200)
    except:
        print("Obliterating existing shm")
        lm1_q = shared_memory.SharedMemory(name='lm1_q',
                                           create=False, size=8)
        lm2_q = shared_memory.SharedMemory(name='lm2_q',
                                           create=False, size=8)
        lm3d_q = shared_memory.SharedMemory(
            name='lm3d_q', create=False, size=508)
        lm4_q = shared_memory.SharedMemory(
            name='lm4_q', create=False, size=508)
        lm5_q = shared_memory.SharedMemory(
            name='lm5_q', create=False, size=508)
        pose = shared_memory.SharedMemory(
            name='pose', create=False, size=100)

        lm1_q.close()
        lm1_q.unlink()
        lm2_q.close()
        lm2_q.unlink()
        lm3d_q.close()
        lm3d_q.unlink()
        lm4_q.close()
        lm4_q.unlink()
        lm5_q.close()
        lm5_q.unlink()
        pose.close()
        pose.unlink()
    try:
        ethernet = True
        viz = True

        if ethernet:
            sok = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            ip = "169.254.96.16"
            port = 5005
            serverAddress = (ip, port)
            sok.connect(serverAddress)
        else:
            sok = 0

        b = 48/1000  # baseline distance (m)
        # convention cap1-left cap2-right from perspective of cameras
        cap1 = 0  # device id for capture device 1
        cap2 = 1  # device id for capture device 2
        # lm3d_q = SimpleQueue()  # 3d projection of landmarks

        mtx, dist, rvecs, tvecs = cal.calibrate('./Calibrate/*.jpg')
        w = 1280
        h = 720
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            mtx, dist, (w, h), 1, (w, h))
        capture1 = Process(target=updateHandTrack, args=(
            cap1, lm1_q, lm4_q, mtx, dist, newcameramtx, roi))
        capture2 = Process(target=updateHandTrack, args=(
            cap2, lm2_q, lm5_q, mtx, dist, newcameramtx, roi))
        lm_to_3d = Process(target=stereo_process,
                           args=(lm3d_q, mtx, b))
        orthoframe = Process(
            target=find_orthonormal_frame, args=(pose, True))
        eth = Process(target=send, args=(sok,))
        if viz:
            v = Visualizer()
        capture1.start()
        capture2.start()
        lm_to_3d.start()
        orthoframe.start()
        if ethernet:
            eth.start()
        if viz:
            v.animation(0.1)
        else:
            poseo = np.ndarray((4, 4), dtype=np.float64)
            desiredpose = np.ndarray(
                (4, 4), dtype=np.float64, buffer=pose.buf)
            while True:
                poseo += 0.05 * (desiredpose - poseo)
                print(poseo)
    except Exception as e:
        print(e)
    finally:
        try:
            print("Exiting...")
            v.app.quit()
            capture1.join()
            capture2.join()
            lm_to_3d.join()
            orthoframe.join()
            eth.join()
            lm1_q.close()
            lm1_q.unlink()
            lm2_q.close()
            lm2_q.unlink()
            lm3d_q.close()
            lm3d_q.unlink()
            lm4_q.close()
            lm4_q.unlink()
            lm5_q.close()
            lm5_q.unlink()
            pose.close()
            pose.unlink()
        finally:
            sys.exit()


if __name__ == "__main__":
    main()
