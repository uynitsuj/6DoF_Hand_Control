#!/usr/bin/env python3

import cv2
import mediapipe as mp
import numpy as np


class handTracker():
    # MediaPipe hand tracking pipeline class
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

    def handsFinder(self, image: cv2.VideoCapture, draw=True) -> cv2.VideoCapture:
        """
        Processes captured image and stores results in class object.
        :param image: cv2.VideoCapture object
        :param draw: True to annotate image with landmarks and segments
        :return: The captured image, possibly modified with annotations.
        """
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imageRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:

                if draw:
                    self.mpDraw.draw_landmarks(
                        image, handLms, self.mpHands.HAND_CONNECTIONS)
        return image

    def positionFinder(self, image: cv2.VideoCapture, handNo=0, draw=True) -> list:
        """
        Finds pixel coordinate of the 21 landmarks and returns in a list.
        :param image: cv2.VideoCapture object
        :param handNo: Hand index TODO: Verify?
        :param draw: if True, annotates image with landmarks and segments
        :return: A list of 21 landmarks in the format [[0,x_0,y_0],[1,x_1,y_1],...,[20,x_20,y_20]]
        """
        lmlist = []
        if self.results.multi_hand_landmarks:
            Hand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(Hand.landmark):
                h, w, c = image.shape
                cx, cy = int(lm.x*w), int(lm.y*h)
                lmlist.append([cx, cy])
                if draw and id == 8:
                    cv2.circle(image, (cx, cy), 10, (255, 0, 255), cv2.FILLED)
                    txt = "index tip coord: " + \
                        "(" + str(cx) + "," + str(cy) + ")"
                    cv2.putText(image, txt, (30, 90), fontFace=cv2.FONT_HERSHEY_COMPLEX,
                                fontScale=2, color=(250, 225, 100))
        return lmlist

    def find3D(self, handNo=0):
        if not self.results.multi_hand_world_landmarks:
            return np.zeros((21, 3))
        lm3dlist = []
        Hand = self.results.multi_hand_world_landmarks[handNo]
        for id, lm in enumerate(Hand.landmark):
            lm3dlist.append([float(lm.x), -float(lm.y), float(lm.z)])
        return lm3dlist
