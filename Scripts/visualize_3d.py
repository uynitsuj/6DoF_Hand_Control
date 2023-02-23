import cv2
from multiprocessing import Process, Queue
import handtracker
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import pyqtgraph as pg

# camera focal length = 3.5mm
# pixel size = 3.0um X 3.0um


def update(capid, queue):
    tracker = handtracker.handTracker()
    cap = cv2.VideoCapture(capid)
    while True:
        _, image = cap.read()
        image = tracker.handsFinder(image)
        lmList = tracker.positionFinder(image)
        queue.put(lmList)
        cv2.imshow("Video", image)
        cv2.waitKey(1)


'''
Stereo Vision
P = Target point in the physical world (scene point)
PL = (xl,yl) = Point P in left camera image
PR = (xr,yr) = Point P in right camera image
T = Baseline distance between center of left and right cameras
f = Focal length of the camera
d = Physical size of a pixel in camera sensor CMOS/CCD

D = Disparity between PL and PR
D = xl-xr

Z = Distance between point P and camera center
Z = (f*T)/(d*D)
'''


def stereo_process(queue1, queue2):
    while True:
        lmlist1 = queue1.get()
        lmlist2 = queue2.get()


def main():
    cap1 = 0  # device id for capture device 1
    cap2 = 1  # device id for capture device 2
    lm1 = Queue()  # landmarks for device 1
    lm2 = Queue()  # landmarks for device 2
    capture1 = Process(target=update, args=(cap1, lm1))
    capture2 = Process(target=update, args=(cap2, lm2))
    lm_to_3d = Process(target=stereo_process, args=(lm1, lm2))
    capture1.start()
    capture2.start()
    lm_to_3d.start()
    capture1.join()
    capture2.join()
    lm_to_3d.join()


if __name__ == "__main__":
    main()
