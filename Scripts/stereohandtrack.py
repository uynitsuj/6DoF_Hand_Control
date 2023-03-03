import cv2
from multiprocessing import Process, Queue
import handtracker
import cameracalibrate


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


def do_stuff_with_landmarks(capid, queue):
    while True:
        print("Camera " + str(capid) + " landmarks:")
        print(queue.get())
        print("\n")


def main():
    cap1 = 0  # device id for capture device 1
    cap2 = 1  # device id for capture device 1
    q1 = Queue()
    q2 = Queue()
    c1 = Process(target=update, args=(cap1, q1))
    c2 = Process(target=update, args=(cap2, q2))
    w1 = Process(target=do_stuff_with_landmarks, args=(cap1, q1))
    w2 = Process(target=do_stuff_with_landmarks, args=(cap2, q2))
    c1.start()
    c2.start()
    w1.start()
    w2.start()
    c1.join()
    c2.join()
    w1.join()
    w2.join()


if __name__ == "__main__":
    main()
