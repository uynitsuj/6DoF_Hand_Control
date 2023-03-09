import cv2
from multiprocessing import Process, Queue
from cvzone.HandTrackingModule import HandDetector
import Calibrate.cameracalibrate as cal
import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import sys


class Visualizer(object):
    def __init__(self):
        self.lm3d = []
        self.traces = dict()
        self.app = QtWidgets.QApplication(sys.argv)
        self.w = gl.GLViewWidget()
        self.w.opts['distance'] = 20
        self.w.setWindowTitle('pyqtgraph Hand Pose')
        self.w.setGeometry(1000, 500, 800, 500)
        self.w.show()
        self.setup()

    def setup(self):
        gsz = 10
        gsp = 1
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
        lm3dlist = self.lm3d.get()
        if lm3dlist:
            print(lm3dlist)
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

    def animation(self, lm3d_q: Queue):
        self.lm3d = lm3d_q
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(1)
        self.start()


def updateHandTrack(capid: int, lm_q: Queue, lm3d_q: Queue, mtx, dist, newcameramtx, roi) -> None:
    """
    Update loop for hand tracker pipeline.
    Intended to be used in a multiprocessing Process callback.
    :param capid: capture device ID as a valid parameter to cv2.VideoCapture()
    :param queue: multiprocessing Queue() object. Queue is updated with hand landmark list
    """
    tracker = HandDetector(detectionCon=0.8, maxHands=1)
    cap = cv2.VideoCapture(capid)
    # cap.set(3, 1280)
    # cap.set(4, 720)
    success, img = cap.read()
    h, w, _ = img.shape
    while True:
        _, image = cap.read()

        # image = cv2.undistort(image, mtx, dist, None, newcameramtx)
        # x, y, w, h = roi
        # image = image[y:y+h, x:x+w]
        # image = cv2.flip(image, 0)
        hands, image = tracker.findHands(image)  # with draw
        data = []

        if hands:
            # Hand 1
            hand = hands[0]
            lmList = hand["lmList"]  # List of 21 Landmark points
            for lm in lmList:
                data.append([lm[2]/75, (w/2-lm[0])/75, (h/2 - lm[1])/75])
        lm3d_q.put(data)
        # print(data)
        # image = tracker.handsFinder(image)
        # lm_q.put(tracker.positionFinder(image))
        cv2.imshow("Video", image)
        cv2.waitKey(1)


def main():
    cap1 = 0  # device id for capture device 1
    lm_q = Queue()  # landmarks
    lm3d_q = Queue()  # landmarks
    mtx, dist, rvecs, tvecs = cal.calibrate('./Calibrate/*.jpg')
    w = 1280
    h = 720
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        mtx, dist, (w, h), 1, (w, h))
    capture1 = Process(target=updateHandTrack, args=(
        cap1, lm_q, lm3d_q, mtx, dist, newcameramtx, roi))

    v = Visualizer()
    capture1.start()
    v.animation(lm3d_q)
    capture1.join()


if __name__ == "__main__":
    main()
