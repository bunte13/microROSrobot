#!/usr/bin/env python3
# encoding: utf-8
""" Section for intitialising essential libraries """

import math  # * Importing the math module for mathematical functions
import time  # * Importing the time module for handling time-related tasks
import cv2 as cv  # * Importing the OpenCV library for computer vision tasks, and aliasing it as cv
import numpy as np  # * Importing the NumPy library for numerical operations, and aliasing it as np
import mediapipe as mp  # * Importing the MediaPipe library for hand tracking

""" Global Variables Initialization """
xp = yp = pTime = boxx = 0                          # * Previous coordinates of the pen, previous time for FPS calculation, and selected color box
tipIds = [4, 8, 12, 16, 20]                         # * Landmark IDs for the tips of fingers
# * 4: Thumb tip, 8: Index finger tip, 12: Middle finger tip, 16: Ring finger tip, 20: Pinky finger tip
imgCanvas = np.zeros((480, 640, 3), np.uint8)       # * Blank canvas for drawing
#!modified
imgCanvasHistory = []                               # * History of canvas states for undo functionality
#!modified
brushThickness = 5                                  # * Default brush thickness
top_height = 50                                     # * Height of the color selection bar
Color = "Red"                                       # * Initially selected color
ColorList = {
    'Red': (0, 0, 255),
    'Green': (0, 255, 0),
    'Blue': (255, 0, 0),
    'Yellow': (0, 255, 255),
    'Black': (0, 0, 0),
}  # * Dictionary mapping color names to BGR values

class handDetector:
    """Class to detect hands and find landmarks using MediaPipe"""
    def __init__(self, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        self.tipIds = [4, 8, 12, 16, 20]
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=5)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=1)

    def findHands(self, frame, draw=True):
        """Method to find hands in a frame and optionally draw landmarks"""
        self.lmList = []
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(frame, handLms, self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                for id, lm in enumerate(self.results.multi_hand_landmarks[0].landmark):
                    h, w, c = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    self.lmList.append([id, cx, cy])
        return frame, self.lmList

    def fingersUp(self):
        """Method to check which fingers are up"""
        fingers = []
        # * Thumb
        if (self.calc_angle(self.tipIds[0], self.tipIds[0] - 1, self.tipIds[0] - 2) > 150.0) and (
                self.calc_angle(self.tipIds[0] - 1, self.tipIds[0] - 2, self.tipIds[0] - 3) > 150.0):
            fingers.append(1)
        else:
            fingers.append(0)
        # * Other 4 fingers
        for id in range(1, 5):
            if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers

    def get_dist(self, point1, point2):
        """Method to calculate the distance between two points"""
        x1, y1 = point1
        x2, y2 = point2
        return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))

    def calc_angle(self, pt1, pt2, pt3):
        """Method to calculate the angle between three points"""
        point1 = self.lmList[pt1][1], self.lmList[pt1][2]
        point2 = self.lmList[pt2][1], self.lmList[pt2][2]
        point3 = self.lmList[pt3][1], self.lmList[pt3][2]
        a = self.get_dist(point1, point2)
        b = self.get_dist(point2, point3)
        c = self.get_dist(point1, point3)
        try:
            radian = math.acos((math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / (2 * a * b))
            angle = radian / math.pi * 180
        except:
            angle = 0
        return abs(angle)
    
    """standalone functions section"""
    
#!modified
def map_distance_to_thickness(distance, min_dist, max_dist, min_thickness, max_thickness):
    """Map the distance between fingers to brush thickness"""
    return min_thickness + (max_thickness - min_thickness) * (distance - min_dist) / (max_dist - min_dist)
#!modified
def undo_last_action():
    """Undo the last drawing action"""
    global imgCanvas, imgCanvasHistory
    if imgCanvasHistory:
        imgCanvas = imgCanvasHistory.pop()
        print(f"Undo: Restored canvas from history. History length: {len(imgCanvasHistory)}")
    else:
        imgCanvas = np.zeros((480, 640, 3), np.uint8)
        print("Undo: No history to restore.")
#!modified
def log_canvas_history():
    """Log the canvas history for debugging"""
    global imgCanvasHistory
    if imgCanvasHistory:
        print(f"Current history length: {len(imgCanvasHistory)}")
        print("Last canvas in history:")
        cv.imshow("Last Canvas", imgCanvasHistory[-1])
        cv.waitKey(1)  # * Briefly show the last canvas state
    else:
        print("Canvas history is empty")

if __name__ == '__main__':
                                # * Capture video from webcam
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    hand_detector = handDetector(detectorCon=0.85)
    
    #!modified
    min_dist = 20               # * Minimum distance for brush thickness mapping
    max_dist = 200              # * Maximum distance for brush thickness mapping
    min_thickness = 5           # * Minimum brush thickness
    max_thickness = 50          # * Maximum brush thickness
    drawing = False             # * Flag to check if drawing is in progress

    while capture.isOpened():
        ret, frame = capture.read()
        frame = cv.flip(frame, 1)
        h, w, c = frame.shape
        frame, lmList = hand_detector.findHands(frame, draw=True)
        
        if len(lmList) != 0:
            x1, y1 = lmList[8][1:]  # * Coordinates of the index finger tip
            x2, y2 = lmList[12][1:]  # * Coordinates of the middle finger tip
            fingers = hand_detector.fingersUp()
            
            # * Selection Mode: Two fingers up
            if fingers[1] and fingers[2]:
                if y1 < top_height:
                    if 0 < x1 < int(w / 5) - 1:
                        boxx = 0
                        Color = "Red"
                    elif int(w / 5) < x1 < int(w * 2 / 5) - 1:
                        boxx = int(w / 5)
                        Color = "Green"
                    elif int(w * 2 / 5) < x1 < int(w * 3 / 5) - 1:
                        boxx = int(w * 2 / 5)
                        Color = "Blue"
                    elif int(w * 3 / 5) < x1 < int(w * 4 / 5) - 1:
                        boxx = int(w * 3 / 5)
                        Color = "Yellow"
                    elif int(w * 4 / 5) < x1 < w - 1:
                        boxx = int(w * 4 / 5)
                        Color = "Black"
                
                # * Map distance between index and middle fingers to brush thickness
                #!modified
                distance = math.hypot(x2 - x1, y2 - y1)
                brushThickness = int(map_distance_to_thickness(distance, min_dist, max_dist, min_thickness, max_thickness))
                cv.circle(frame, (x1, y1), brushThickness // 2, (255, 0, 255), 2)
                
                # ! Debugging output
                print(f"Color: {Color}, Brush Thickness: {brushThickness}")

                # * Drawing color selection boxes
                cv.rectangle(frame, (boxx, 0), (boxx + int(w / 5), top_height), ColorList[Color], cv.FILLED)
                cv.rectangle(frame, (0, 0), (int(w / 5) - 1, top_height), ColorList['Red'], 3)
                cv.rectangle(frame, (int(w / 5) + 2, 0), (int(w * 2 / 5) - 1, top_height), ColorList['Green'], 3)
                cv.rectangle(frame, (int(w * 2 / 5) + 2, 0), (int(w * 3 / 5) - 1, top_height), ColorList['Blue'], 3)
                cv.rectangle(frame, (int(w * 3 / 5) + 2, 0), (int(w * 4 / 5) - 1, top_height), ColorList['Yellow'], 3)
                cv.rectangle(frame, (int(w * 4 / 5) + 2, 0), (w - 1, top_height), ColorList['Black'], 3)
            
            # * Drawing Mode: Index finger up and middle finger down
            #!modified
            if fingers[1] and not fingers[2] and math.hypot(x2 - x1, y2 - y1) > 50:
                if xp == yp == 0:
                    xp, yp = x1, y1
                if not drawing:
                    imgCanvasHistory.append(imgCanvas.copy())
                    drawing = True
                cv.line(frame, (xp, yp), (x1, y1), ColorList[Color], brushThickness)
                cv.line(imgCanvas, (xp, yp), (x1, y1), ColorList[Color], brushThickness)
                xp, yp = x1, y1
                
            else:
                xp = yp = 0
                if drawing:
                    imgCanvasHistory.append(imgCanvas.copy())
                    drawing = False

        # * Merge canvas with the video frame
        imgGray = cv.cvtColor(imgCanvas, cv.COLOR_BGR2GRAY)
        _, imgInv = cv.threshold(imgGray, 50, 255, cv.THRESH_BINARY_INV)
        imgInv = cv.cvtColor(imgInv, cv.COLOR_GRAY2BGR)
        frame = cv.bitwise_and(frame, imgInv)
        frame = cv.bitwise_or(frame, imgCanvas)

        #!modified
        # * Undo functionality on pressing 'u'
        if cv.waitKey(1) & 0xFF == ord('u'):
            print("Undo command received")
            undo_last_action()
        # * Exit on pressing 'q'
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        
        # * Display FPS on the frame
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        text = "FPS : " + str(int(fps))
        cv.rectangle(frame, (20, h - 100), (50, h - 70), ColorList[Color], cv.FILLED)
        cv.putText(frame, Color, (70, h - 75), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        cv.putText(frame, text, (20, h - 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        cv.imshow('frame', frame)
    
    capture.release()
    cv.destroyAllWindows()
