#sudo apt-get install python3-pip
#pip install opencv-python
# python webcam_capture.py

import cv2 as cv
import os

# Open the webcam
cap = cv.VideoCapture(0)  # 0 corresponds to the default webcam

# Check the actual resolution set by the webcam
# Check if the webcam is opened successfully
if not cap.isOpened():
    print("Error: Couldn't open the webcam")
    exit()

actual_width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
actual_height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)

lowres = [480,320]
medres = [640,480]
highres = [1280,720]
maxres = [1920,1080]
print(f"Camera Base Resolution: {actual_width}x{actual_height}")

# Set the resolution (width, height)  #will crop the image frame
print(f'Low Res "L" = {lowres[0]} X {lowres[1]}  Med Res "M" = {medres[0]}X{medres[1]}\n High Res "H" ={highres[0]}X{highres[1]}  Max Res "X"={maxres[0]}X{maxres[1]}')
check = input("change native resolution? any other key will leave default. L,M,H,X?:  ")
if check == "l":
    actual_width = lowres[0]
    actual_height = lowres[1]
    cap.set(cv.CAP_PROP_FRAME_WIDTH, actual_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, actual_height)
elif check == "m":
    actual_width = medres[0]
    actual_height = medres[1]
    cap.set(cv.CAP_PROP_FRAME_WIDTH, actual_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, actual_height)
elif check == "h":
    actual_width = highres[0]
    actual_height = highres[1]
    cap.set(cv.CAP_PROP_FRAME_WIDTH, actual_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, actual_height)
elif check == "x":
    actual_width = maxres[0]
    actual_height = maxres[1]
    cap.set(cv.CAP_PROP_FRAME_WIDTH, actual_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, actual_height)
else:
    print("Resolution left unchanged")
print(f"Camera Resolution: {actual_width}x{actual_height}")



# Prompt user for filename and directory
directory = input("Enter the directory to save photos: ")
filename = input("Enter the filename to save photos: ")
print("Press the spacebar to capture an image, press Q key to exit the capture")

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
    cv.imshow('Webcam Feed', frame)

    # Check for key press events
    key = cv.waitKey(1)
    
    # Capture a photo when spacebar is pressed
    if key == 32:  # ASCII code for spacebar
        # Save photo with filename and index
        photo_path = os.path.join(directory, f"{filename}_{photo_index}.jpg")
        cv.imwrite(photo_path, frame)
        print(f"Photo captured and saved as {photo_path}      press q to quit")
        photo_index += 1

    # Exit when 'q' key is pressed
    elif key & 0xFF == ord('q'):
        break

# Release the webcam and close all OpenCV windows
cap.release()
cv.destroyAllWindows()