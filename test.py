import mediapipe as mp
import cv2

img = cv2.imread("landscape.jpg")
print(ord('q'))
while True:
    cv2.imshow("window", img)
    if cv2.waitKey(0) == 113:
        break
cv2.destroyAllWindows()
