import cv2
import handtracker


def main():
    cap = cv2.VideoCapture(1)
    tracker = handtracker.handTracker()
    while True:
        success, image = cap.read()
        image = tracker.handsFinder(image)
        lmList = tracker.positionFinder(image)
        cv2.imshow("Video", image)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()
