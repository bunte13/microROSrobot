import math
import cv2 as cv
import mediapipe as mp

class handDetector:
    """Class to detect hands and find landmarks using MediaPipe"""
    def __init__(self, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        self.tipIds = [4, 8, 12, 16, 20]
        self.mpHands = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHands.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=5)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=1)
        self.lmList = []

    def findHands(self, frame, draw=True):
        """Method to find hands in a frame and optionally draw landmarks"""
        self.lmList = []
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(frame, handLms, self.mpHands.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                for id, lm in enumerate(handLms.landmark):
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

    def get_dist(self, point1, point2):
        """Method to calculate the distance between two points"""
        x1, y1 = point1
        x2, y2 = point2
        return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))

if __name__ == '__main__':
    # * Capture video from webcam
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    hand_detector = handDetector(detectorCon=0.85)

    while capture.isOpened():
        ret, frame = capture.read()
        frame = cv.flip(frame, 1)
        frame, lmList = hand_detector.findHands(frame, draw=True)

        if len(lmList) != 0:
            fingers = hand_detector.fingersUp()
            print(f"Fingers Up: {fingers}")
            # Display fingers up on the frame
            cv.putText(frame, f'Fingers Up: {fingers}', (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv.LINE_AA)

        # * Exit on pressing 'q'
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        # Display the frame
        cv.imshow('Hand Tracking', frame)

    capture.release()
    cv.destroyAllWindows()
