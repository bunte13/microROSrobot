import mediapipe as mp
import cv2

# Initialize MediaPipe face mesh
facemesh = mp.solutions.face_mesh.FaceMesh(
    static_image_mode=False,  # Set to False for real-time detection
    min_tracking_confidence=0.6,
    min_detection_confidence=0.6
)
draw = mp.solutions.drawing_utils

# Open a video capture stream (webcam)
cap = cv2.VideoCapture(0)

while True:
    # Read frame from webcam
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to capture frame from camera. Exiting...")
        break
    
    # Convert BGR to RGB
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Process image with MediaPipe
    results = facemesh.process(rgb)
    
    # Draw landmarks if faces are detected
    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            for id, landmark in enumerate(face_landmarks.landmark):
                # Extract landmark positions
                h, w, c = frame.shape  # Height, width, channels of the frame
                cx, cy = int(landmark.x * w), int(landmark.y * h)  # Convert normalized coordinates to pixel coordinates
                print(f"Landmark {id}: x={cx}, y={cy}")
                
            # Draw landmarks on face
            draw.draw_landmarks(
                image=frame,
                landmark_list=face_landmarks,
                connections=mp.solutions.face_mesh.FACEMESH_TESSELATION,
                landmark_drawing_spec=draw.DrawingSpec(color=(0, 255, 255), circle_radius=1),
                connection_drawing_spec=draw.DrawingSpec(color=(0, 255, 255), thickness=1)
            )
    
    # Display the frame
    cv2.imshow('Frame', frame)
    
    # Exit loop on ESC key press
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
