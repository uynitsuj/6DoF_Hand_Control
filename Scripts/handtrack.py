import cv2
import mediapipe as mp
import dataclasses
import matplotlib.pyplot as plt
import math
import numpy as np


class handTracker():
    def __init__(self, mode=False, maxHands=1, detectionCon=0.5, modelComplexity=1, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.modelComplex = modelComplexity
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex,
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def handsFinder(self, image, draw=True):
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imageRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:

                if draw:
                    self.mpDraw.draw_landmarks(
                        image, handLms, self.mpHands.HAND_CONNECTIONS)
        return image

    def positionFinder(self, image, handNo=0, draw=True):
        lmlist = []
        if self.results.multi_hand_landmarks:
            Hand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(Hand.landmark):
                h, w, c = image.shape
                cx, cy = int(lm.x*w), int(lm.y*h)
                lmlist.append([id, cx, cy])
                if draw and id == 8:
                    cv2.circle(image, (cx, cy), 10, (255, 0, 255), cv2.FILLED)
                    txt = "index tip coord: " + \
                        "(" + str(cx) + "," + str(cy) + ")"
                    cv2.putText(image, txt, (30, 90), fontFace=cv2.FONT_HERSHEY_COMPLEX,
                                fontScale=2, color=(250, 225, 100))
        return lmlist

    def plot3D(self):
        if self.results.multi_hand_world_landmarks:
            for lm in self.results.multi_hand_world_landmarks:
                plot_landmarks(
                    lm, self.mpHands.HAND_CONNECTIONS)


def main():
    cap = cv2.VideoCapture(1)
    tracker = handTracker()

    #plt.figure(figsize=(10, 10))
    #ax = plt.axes(projection='3d')
    while True:
        success, image = cap.read()
        image = tracker.handsFinder(image)
        lmList = tracker.positionFinder(image)
        # if len(lmList) != 0:
        # print(lmList[4])

        # tracker.plot3D()
        cv2.imshow("Video", image)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()
