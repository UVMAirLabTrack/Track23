import cv2
import cv2.aruco as aruco
import numpy as np

# Function to generate and display ARuco markers
def generate_and_display_aruco_markers(marker_size, num_markers):
    marker_size = marker_size / 0.2645833333 #mm
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)  # You can choose a different dictionary if needed
    parameters = aruco.DetectorParameters_create()

    # Create a blank image
    img = np.zeros((marker_size * num_markers, marker_size * num_markers), dtype=np.uint8)

    for i in range(num_markers):
        marker = aruco.drawMarker(aruco_dict, i, marker_size)
        row = (i // num_markers) * marker_size
        col = (i % num_markers) * marker_size
        img[row:row + marker_size, col:col + marker_size] = marker

    cv2.imshow('ARuco Markers', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    marker_size = 1.25*25.4  # Adjust the marker size as needed in mm
    num_markers = 5    # Adjust the number of markers as needed

    generate_and_display_aruco_markers(marker_size, num_markers)

    #comment v2
