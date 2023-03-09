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
        self.pfilter = 0
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

        desired = np.ndarray((21, 3), dtype=np.float64, buffer=self.lm3d.buf)
        if self.pfilter:
            lm3dlist += self.pfilter * (desired - lm3dlist)
        else:
            lm3dlist = desired
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

    def animation(self, pfilter):
        self.pfilter = pfilter
        self.lm3d = shared_memory.SharedMemory(name='lm3d_q')
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(1)
        self.start()


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
                xyz.append([z, x, -y])
        # print(np.array(xyz).dtype)
        # print(np.array(xyz).nbytes)
        # print(np.array(xyz).shape)
        # qout.put(xyz)
        xyz = np.array(xyz)
        # print(xyz)

        buffer = np.ndarray(xyz.shape, dtype=np.float64, buffer=outshm.buf)
        buffer[:] = xyz[:]


def updateHandTrack(capid: int, shm, mtx, dist, newcameramtx, roi, imshow=False) -> None:
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

        buffer = np.ndarray(lmList.shape, dtype=np.int32, buffer=shm.buf)
        buffer[:] = lmList[:]

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
    except:
        print("Obliterating existing shm")
        lm1_q = shared_memory.SharedMemory(name='lm1_q',
                                           create=False, size=8)
        lm2_q = shared_memory.SharedMemory(name='lm2_q',
                                           create=False, size=8)
        lm3d_q = shared_memory.SharedMemory(
            name='lm3d_q', create=False, size=508)
        lm1_q.close()
        lm1_q.unlink()
        lm2_q.close()
        lm3d_q.unlink()
        lm3d_q.unlink()
    try:
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
            cap1, lm1_q, mtx, dist, newcameramtx, roi))
        capture2 = Process(target=updateHandTrack, args=(
            cap2, lm2_q, mtx, dist, newcameramtx, roi))
        lm_to_3d = Process(target=stereo_process,
                           args=(lm3d_q, mtx, b))

        v = Visualizer()
        capture1.start()
        capture2.start()
        lm_to_3d.start()
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
            lm1_q.close()
            lm1_q.unlink()
            lm2_q.close()
            lm2_q.unlink()
            lm3d_q.unlink()
            lm3d_q.unlink()
        finally:
            sys.exit()


if __name__ == "__main__":
    main()
