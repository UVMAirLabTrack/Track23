import cv2
import os

# Open the webcam
cap = cv2.VideoCapture(0)  # 0 corresponds to the default webcam

# Set the resolution (width, height)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Check if the webcam is opened successfully
if not cap.isOpened():
    print("Error: Couldn't open the webcam")
    exit()

# Prompt user for filename and directory
directory = input("Enter the directory to save photos: ")
filename = input("Enter the filename to save photos: ")

# Create the directory if it doesn't exist
if not os.path.exists(directory):
    os.makedirs(directory)

# Initialize photo index
photo_index = 1

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Check if the frame is captured successfully
    if not ret:
        print("Error: Couldn't capture frame")
        break

    # Display the captured frame
    cv2.imshow('Webcam Feed', frame)

    # Check for key press events
    key = cv2.waitKey(1)
    
    # Capture a photo when spacebar is pressed
    if key == 32:  # ASCII code for spacebar
        # Save photo with filename and index
        photo_path = os.path.join(directory, f"{filename}_{photo_index}.jpg")
        cv2.imwrite(photo_path, frame)
        print(f"Photo captured and saved as {photo_path}")
        photo_index += 1

    # Exit when 'q' key is pressed
    elif key & 0xFF == ord('q'):
        break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()