import cv2
from multiprocessing import Process, Queue
import handtracker


def update(capid, queue):
    tracker = handtracker.handTracker()
    cap = cv2.VideoCapture(capid)
    while True:
        _, image = cap.read()
        image = tracker.handsFinder(image)
        lmList = tracker.positionFinder(image)
        # print(lmList)
        queue.put(lmList)
        cv2.imshow("Video", image)
        cv2.waitKey(1)


def main():
    cap1 = 0  # device id for capture device 1
    cap2 = 1  # device id for capture device 1
    q1 = Queue()
    q2 = Queue()
    c1 = Process(target=update, args=(cap1, q1))
    c2 = Process(target=update, args=(cap2, q2))
    c1.start()
    c2.start()
    while True:
        print("Camera 1 landmarks:")
        print(q1.get())
        print("\n")
        print("Camera 2 landmarks:")
        print(q2.get())
        print("\n")
    c1.join()
    c2.join()


if __name__ == "__main__":
    main()
