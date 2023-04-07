import cv2
from multiprocessing import Process, shared_memory
import handtracker
import Calibrate.cameracalibrate as cal
import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import sys
import socket
import pickle
from IKEngineviz import IKSixR
from quatlinalg import normalize_quaternion, quaternion_to_se3, orthonormalize_matrix, so3_to_quaternion
import time


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

        quatdisp = np.ndarray((1, 7), dtype=np.float64)
        desiredpose = np.ndarray(
            (1, 7), dtype=np.float64, buffer=self.pose.buf)
        if self.pfilter:

            quatdisp += self.pfilter * (desiredpose - quatdisp)
        else:
            quatdisp = desiredpose

        if quatdisp[0][0]:
            pose = quaternion_to_se3(normalize_quaternion(
                (quatdisp[0][0], quatdisp[0][1], quatdisp[0][2], quatdisp[0][3])))
            pose[0][3] = quatdisp[0][4]
            pose[1][3] = quatdisp[0][5]
            pose[2][3] = quatdisp[0][6]
            print(pose)

            width = 10
            w = [pose[0][3], pose[1][3], pose[2][3]]
            v1 = [pose[0][0]/10, pose[1][0]/10, pose[2][0]/10]
            v2 = [pose[0][1]/10, pose[1][1]/10, pose[2][1]/10]
            v3 = [pose[0][2]/10, pose[1][2]/10, pose[2][2]/10]
            v1 = np.append([w], [np.add(w, v1)], axis=0)

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

    def animation(self, pfilter):
        self.pfilter = pfilter
        self.lm3d = shared_memory.SharedMemory(name='lm3d_q')
        self.lm3dil = shared_memory.SharedMemory(name='lm4_q')
        self.lm3dir = shared_memory.SharedMemory(name='lm5_q')
        self.pose = shared_memory.SharedMemory(name='pose')
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(50)
        self.start()


def send(sok):
    """
    Brief: Continuously sends the updated pose to a connected client via a socket connection.

    This function reads a desired pose from shared memory, calculates an updated pose using an
    proportional filter, and sends the updated pose to the connected client as a
    pickled numpy array.

    Args:
        sok (socket.socket): The socket object used to send the pickled pose data to the connected client.

    Returns:
        None
    """
    while True:
        time.sleep(60/1000)  # Function sends the data every 60 ms
        selfpose = shared_memory.SharedMemory(name='pose')
        pose = np.ndarray((1, 7), dtype=np.float64)
        desiredpose = np.ndarray(
            (1, 7), dtype=np.float64, buffer=selfpose.buf)
        pose += 0.09 * (desiredpose - pose)  # moving average filter impl
        posecopy = pose
        data_string = pickle.dumps(posecopy)
        if pose[0][0]:
            sok.sendall(data_string)


def find_orthonormal_frame(outshm, pfilter: bool):
    """
    Brief: Finds an orthonormal frame for a hand pose and writes the result as a quaternion and wrist position to shared memory.

    This function calculates the orthonormal frame for the hand pose using data from shared memory. It can apply a proportional filter if pfilter is True.
    The orthonormal frame is calculated using the positions of the index finger and pinky, and the resulting rotation matrix is converted to a quaternion.
    The calculated quaternion and wrist position are written to the shared memory specified by outshm.

    Args:
    outshm (shared_memory.SharedMemory): A shared memory object used to write the output quaternion and wrist position.
    pfilter (bool): If True, apply a proportional filter to the input data. If False, use the input data directly.

    Returns:
    None
    """
    lm3dshm = shared_memory.SharedMemory(name='lm3d_q')
    lm3dil = shared_memory.SharedMemory(name='lm4_q')
    lm3dir = shared_memory.SharedMemory(name='lm5_q')
    lm3dlist = np.ndarray((21, 3), dtype=np.float64)
    lm3dillist = np.ndarray((21, 3), dtype=np.float64)
    lm3dirlist = np.ndarray((21, 3), dtype=np.float64)
    desired = np.ndarray((21, 3), dtype=np.float64, buffer=lm3dshm.buf)
    desiredil = np.ndarray((21, 3), dtype=np.float64, buffer=lm3dil.buf)
    desiredir = np.ndarray((21, 3), dtype=np.float64, buffer=lm3dir.buf)

    while True:
        if pfilter:
            lm3dlist += 0.01 * (desired - lm3dlist)
            lm3dillist += 0.01 * (desiredil - lm3dillist)
            lm3dirlist += 0.01 * (desiredir - lm3dirlist)
        else:
            lm3dlist = desired
            lm3dillist = desiredil
            lm3dirlist = desiredir
        if lm3dlist.tolist():
            # Wrist
            wrist = lm3dlist[0]

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

                quatdisp = [quat[0], quat[1], quat[2],
                            quat[3], wrist[0], wrist[1], wrist[2]]
                quatdisp = np.array(quatdisp)

                buffer = np.ndarray(
                    quatdisp.shape, dtype=np.float64, buffer=outshm.buf)
                buffer[:] = quatdisp[:]


def stereo_process(outshm, mtx, b) -> None:
    """
    Brief: Processes stereo image data to compute 3D positions of hand landmarks using stereoscopic projection.

    This function takes shared memory objects containing 2D landmark lists for two camera devices, and computes the 3D positions of the hand landmarks
    using stereoscopic projection. The stereo camera center is considered the origin frame. The resulting list of 3D vectors is written to the shared
    memory specified by outshm. This function is intended to be used in a multiprocessing Process callback.

    Args:
    outshm (shared_memory.SharedMemory): A shared memory object used to write the output 3D positions of hand landmarks.

    mtx (numpy.ndarray): A 3x3 camera intrinsic matrix containing focal lengths (fx, fy) and optical center (ox, oy).

    b (float): The baseline distance between the two cameras.

    Returns:
    None

    GPT-4 Response: The stereoscopic projection algorithm seems correct. It calculates the disparity (d) between the corresponding landmarks from both cameras,
    and then uses the disparity, camera intrinsic matrix (mtx), and baseline distance (b) to compute the 3D positions (x, y, z) of the hand landmarks.
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

        # iterate and calculate disparity between corresponding landmarks
        xyz = []
        if lmlist1.tolist() and lmlist2.tolist():
            lmcat = np.concatenate(
                (np.array(lmlist1), np.array(lmlist2)), axis=1)
            for idx, lm in enumerate(lmcat):
                # compute disparity for each landmark, then find x, y, z
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

        buffer = np.ndarray(xyz.shape, dtype=np.float64, buffer=outshm.buf)
        buffer[:] = xyz[:]


def updateHandTrack(capid: int, shm, shm3d, mtx, dist, newcameramtx, roi, imshow=False) -> None:
    """
    Brief: Main update loop for the hand tracking pipeline.

    This function is intended to be used in a multiprocessing Process callback. It captures frames from the specified camera, processes the frames
    to find hand landmarks using the handtracker class, and writes the 2D and 3D hand landmark positions to the provided shared memory objects.

    Args:
    capid (int): The capture device ID, which is a valid parameter for cv2.VideoCapture().

    shm (shared_memory.SharedMemory): A shared memory object used to write the 2D hand landmark positions.

    shm3d (shared_memory.SharedMemory): A shared memory object used to write the 3D hand landmark positions.

    mtx (numpy.ndarray): A 3x3 camera intrinsic matrix. (unused)

    dist (numpy.ndarray): The distortion coefficients of the camera. (unused)

    newcameramtx (numpy.ndarray): A new camera matrix obtained after undistorting the image. (unused)

    roi (tuple): A tuple containing the region of interest (x, y, w, h) after undistorting the image. (unused)

    imshow (bool, optional): If True, the processed frame with hand landmarks will be displayed using cv2.imshow(). Defaults to False.

    Returns:
    None
    """
    tracker = handtracker.handTracker()
    cap = cv2.VideoCapture(capid)

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
            ip = "169.254.96.16"  # remote device IP
            port = 5005
            serverAddress = (ip, port)
            sok.connect(serverAddress)
        else:
            sok = 0

        b = 48/1000  # baseline distance (m)
        # convention: cap1-left cap2-right from perspective behind cameras
        cap1 = 0  # device id for capture device 1
        cap2 = 1  # device id for capture device 2

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
