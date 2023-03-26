import cv2
from multiprocessing import Process, SimpleQueue, shared_memory
import handtracker
import Calibrate.cameracalibrate as cal
import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import sys


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
                    pos=lm3dlist, color=pg.glColor((30, 50)), size=28))

        pose = np.ndarray((4, 4), dtype=np.float64)
        desiredpose = np.ndarray(
            (4, 4), dtype=np.float64, buffer=self.pose.buf)
        if self.pfilter:
            pose += self.pfilter * (desiredpose - pose)
        else:
            pose = desiredpose
        if True:
            # print(pose)
            width = 10
            w = [pose[0][3], pose[1][3], pose[2][3]]
            v1 = [pose[0][0], pose[1][0], pose[2][0]]
            v2 = [pose[0][1], pose[1][1], pose[2][1]]
            v3 = [pose[0][2], pose[1][2], pose[2][2]]
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

    def animation(self, pfilter):
        self.pfilter = pfilter
        self.lm3d = shared_memory.SharedMemory(name='lm3d_q')
        self.lm3dil = shared_memory.SharedMemory(name='lm4_q')
        self.lm3dir = shared_memory.SharedMemory(name='lm5_q')
        self.pose = shared_memory.SharedMemory(name='pose')
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(1)
        self.start()


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
                v2 = v2/np.linalg.norm(v2)/10

                pinkyl = (lm3dillist[17] - lm3dillist[0])
                v2l = pinkyl - \
                    np.dot((np.dot(v1l.T, pinkyl)/np.dot(v1l.T, v1l)), v1l)
                v2l = v2l/np.linalg.norm(v2l)/10

                pinkyr = (lm3dirlist[17] - lm3dirlist[0])
                v2r = pinkyr - \
                    np.dot((np.dot(v1r.T, pinkyr)/np.dot(v1r.T, v1r)), v1r)
                v2r = v2r/np.linalg.norm(v2r)/10

                v3 = np.cross(v1, v2)
                v3 = v3/np.linalg.norm(v3)/10

                v3l = np.cross(v1l, v2l)
                v3l = v3l/np.linalg.norm(v3l)/10

                v3r = np.cross(v1r, v2r)
                v3r = v3r/np.linalg.norm(v3r)/10

                pose1 = [[v1[0], v2[0], v3[0], wrist[0]], [
                    v1[1], v2[1], v3[1], wrist[1]], [v1[2], v2[2], v3[2], wrist[2]], [0, 0, 0, 1]]
                pose1 = np.array(pose1)

                posel = [[v1l[0], v2l[0], v3l[0], wrist[0]], [
                    v1l[1], v2l[1], v3l[1], wrist[1]], [v1l[2], v2l[2], v3l[2], wrist[2]], [0, 0, 0, 1]]
                posel = np.array(posel)

                poser = [[v1r[0], v2r[0], v3r[0], wrist[0]], [
                    v1r[1], v2r[1], v3r[1], wrist[1]], [v1r[2], v2r[2], v3r[2], wrist[2]], [0, 0, 0, 1]]
                poser = np.array(poser)

                pose = np.mean([pose1, posel, poser], axis=0)

                buffer = np.ndarray(
                    pose.shape, dtype=np.float64, buffer=outshm.buf)
                buffer[:] = pose[:]


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
                xyz.append([0.13-x, y, -z])
        xyz = np.array(xyz)
        # print(xyz)

        buffer = np.ndarray(xyz.shape, dtype=np.float64, buffer=outshm.buf)
        buffer[:] = xyz[:]


def updateHandTrack(capid: int, shm, shm3d, mtx, dist, newcameramtx, roi, imshow=True) -> None:
    """
    Update loop for hand tracker pipeline.
    Intended to be used in a multiprocessing Process callback.
    :param capid: capture device ID as a valid parameter to cv2.VideoCapture()
    :param queue: multiprocessing Queue() object. Queue is updated with hand landmark list
    """
    tracker = handtracker.handTracker()
    cap = cv2.VideoCapture(capid)

    cap.set(cv2.CAP_PROP_FPS, 15)
    while True:
        _, image = cap.read()

        dst = cv2.undistort(image, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
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
        viz = False
        b = 48/1000  # baseline distance (m)
        # convention cap1-left cap2-right from perspective of cameras
        cap1 = 0  # device id for capture device 1
        cap2 = 2  # device id for capture device 2
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
        if viz:
            v = Visualizer()
        capture1.start()
        capture2.start()
        lm_to_3d.start()
        orthoframe.start()
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
