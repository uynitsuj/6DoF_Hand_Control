import cv2
from multiprocessing import Process, Queue
import handtracker
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import pyqtgraph as pg

global f  # camera focal length: 3.5e-3
global d  # Physical size of pixel: 3um X 3um
global T  # baseline distance: tbd

'''
Stereo Vision
TODO: Verify if math correct

P = Target point in the physical world (scene point)
PL = (xl,yl) = Point P in left camera image
PR = (xr,yr) = Point P in right camera image
T = Baseline distance between center of left and right cameras
f = Focal length of the camera
d = Physical size of a pixel in camera sensor CMOS/CCD

D = Disparity between PL and PR
D = xl-xr

Z = Distance between point P and camera center 
TODO: Is this right? Is Z distance between P and camera center or P and camera plane? Two different things and will lead to slightly different results
Z = (f*T)/(d*D)

Z = (f(m)*T(m))/(d(m)*D(delta_pixels))
'''


def stereo_process(queue1: Queue, queue2: Queue, queueout: Queue) -> None:
    """
    Takes two landmark list queue objects and computes the stereoscopic projection. 
    Intended to be used in a multiprocessing Process callback.
    Result is a list of vectors with translation position. Stereo camera center is the origin frame.
    :param queue1: landmark list for camera device 1
    :param queue2: landmark list for camera device 2
    :param queueout: stereoscopic projection, a list of 3-vectors
    """
    while True:
        lmlist1 = queue1.get()
        lmlist2 = queue2.get()
        # TODO: use numpy array and concatenate the two landmark lists together before iterating to find the disparity.
        lmcat = None

        # iterate and calculate disparity between corresponding landmarks
        for idx, lm in enumerate(lmcat):
            break


def updateHandTrack(capid: int, queue: Queue) -> None:
    """
    Update loop for hand tracker pipeline.
    Intended to be used in a multiprocessing Process callback.
    :param capid: capture device ID as a valid parameter to cv2.VideoCapture()
    :param queue: multiprocessing Queue() object. Queue is updated with hand landmark list
    """
    tracker = handtracker.handTracker()
    cap = cv2.VideoCapture(capid)
    while True:
        _, image = cap.read()
        image = tracker.handsFinder(image)
        lmList = tracker.positionFinder(image)
        queue.put(lmList)
        cv2.imshow("Video", image)
        cv2.waitKey(1)


def main():
    cap1 = 0  # device id for capture device 1
    cap2 = 1  # device id for capture device 2
    lm1 = Queue()  # landmarks for device 1
    lm2 = Queue()  # landmarks for device 2
    lm3d = Queue()  # 3d projection of landmarks
    capture1 = Process(target=updateHandTrack, args=(cap1, lm1))
    capture2 = Process(target=updateHandTrack, args=(cap2, lm2))
    lm_to_3d = Process(target=stereo_process, args=(lm1, lm2, lm3d))
    capture1.start()
    capture2.start()
    lm_to_3d.start()
    capture1.join()
    capture2.join()
    lm_to_3d.join()


if __name__ == "__main__":
    main()
