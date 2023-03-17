## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import mediapipe as mp

fulposold = []
decimation = rs.decimation_filter()

class handTracker():
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5,modelComplexity=1,trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.modelComplex = modelComplexity
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,self.modelComplex,
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def handsFinder(self,image,draw=True):
        imageRGB = image #cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imageRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:

                if draw:
                    self.mpDraw.draw_landmarks(image, handLms, self.mpHands.HAND_CONNECTIONS)
        return image

    def positionFinder(self,image, handNo=0, draw=True):
        lmlist = []
        if self.results.multi_hand_landmarks:
            Hand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(Hand.landmark):
                h,w,c = image.shape
                cx,cy = int(lm.x*w), int(lm.y*h)
                lmlist.append([id,cx,cy])
            if draw:
                cv2.circle(image,(cx,cy), 15 , (255,0,255), cv2.FILLED)

        return lmlist
    
    
def main():
    # Configure depth and color streams
    global fulposold
    global decimation
    tracker = handTracker()
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            dec_depth = decimation.process(depth_frame)
#             dec_depth = np.asanyarray(dec_depth.get_data())
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            #dec_depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(dec_depth, alpha=0.03), cv2.COLORMAP_JET)

#             depth_colormap_dim = depth_colormap.shape
#             color_colormap_dim = color_image.shape
# 
#             # If depth and color resolutions are different, resize color image to match depth image for display
#             if depth_colormap_dim != color_colormap_dim:
#                 resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
#                 images = np.hstack((resized_color_image, depth_colormap))
#             else:
#                 images = np.hstack((color_image, depth_colormap))
                
            image = tracker.handsFinder(color_image)
            lmList = tracker.positionFinder(image)
            # Show images
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', images)
            # cv2.waitKey(1)
            fullpos = []
            #depth_frame = dec_depth
            for xy in lmList:
                try:
                    z = depth_frame.get_distance(xy[1], xy[2])
                except:
                    z = fullposold[xy[0]][2]
                fullpos.append([xy[1], xy[2], z])
            if len(fullpos) != 0:
                print([depth_frame.get_width(), depth_frame.get_height()])
                print(fullpos[4])
                
            fullposold = fullpos
            cv2.imshow("Video",image)
            cv2.imshow("Depth",depth_colormap)
            #cv2.imshow("Dec Depth",dec_depth_colormap)
            cv2.waitKey(1)

    finally:

        # Stop streaming
        pipeline.stop()


if __name__ == "__main__":
    main()
