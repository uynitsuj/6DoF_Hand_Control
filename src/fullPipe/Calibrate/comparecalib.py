import cv2
from multiprocessing import Process, Queue
import cameracalibrate


def update(capid, mtx, dist, ncmtx, roi):
    cap = cv2.VideoCapture(capid)
    while True:
        _, image = cap.read()
        # image = tracker.handsFinder(image)
        # lmList = tracker.positionFinder(image)
        # print(lmList)
        # queue.put(lmList)
        cv2.imshow("Video", image)

        dst = cv2.undistort(image, mtx, dist, None, ncmtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv2.waitKey(1)
        cv2.imshow("Undistorted", dst)
        # print(dst.shape)


def main():
    cap1 = 0  # device id for capture device 1
    # cap2 = 1  # device id for capture device 1
    mtx, dist, rvecs, tvecs = cameracalibrate.calibrate()
    w = 1280
    h = 720
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        mtx, dist, (w, h), 1, (w, h))
    c1 = Process(target=update, args=(cap1, mtx, dist, newcameramtx, roi))
    c1.start()
    c1.join()


if __name__ == "__main__":
    main()
