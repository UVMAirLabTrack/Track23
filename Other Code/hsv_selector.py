import cv2
import numpy as np
import tkinter as tk
from tkinter import Scale

class WebcamApp:
    def __init__(self, window, window_title, video_source=0):
        self.window = window
        self.window.title(window_title)
        self.video_source = video_source
        
        # Open video source
        self.vid = cv2.VideoCapture(video_source)
        
        # Create sliders for HSV conversion
        self.hue_slider = Scale(window, from_=0, to=179, orient=tk.HORIZONTAL, label="Hue")
        self.hue_slider.pack()
        self.saturation_slider = Scale(window, from_=0, to=255, orient=tk.HORIZONTAL, label="Saturation")
        self.saturation_slider.pack()
        self.value_slider = Scale(window, from_=0, to=255, orient=tk.HORIZONTAL, label="Value")
        self.value_slider.pack()
        
        # Create canvas to display video
        self.canvas = tk.Canvas(window, width=self.vid.get(cv2.CAP_PROP_FRAME_WIDTH), 
                                 height=self.vid.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.canvas.pack()
        
        # Button to capture HSV image
        self.capture_button = tk.Button(window, text="Capture HSV", command=self.capture_hsv)
        self.capture_button.pack()
        
        # Start video playback
        self.update()
        
        self.window.mainloop()
        
    def capture_hsv(self):
        # Capture current HSV values from sliders
        hue = self.hue_slider.get()
        saturation = self.saturation_slider.get()
        value = self.value_slider.get()
        
        # Apply HSV conversion
        ret, frame = self.vid.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_frame[:,:,0] += hue  # Hue
        hsv_frame[:,:,1] += saturation  # Saturation
        hsv_frame[:,:,2] += value  # Value
        hsv_frame = np.clip(hsv_frame, 0, 255).astype(np.uint8)
        
        # Show the new HSV image
        cv2.imshow('HSV Image', cv2.cvtColor(hsv_frame, cv2.COLOR_HSV2BGR))
        
    def update(self):
        # Get a frame from the video source
        ret, frame = self.vid.read()
        
        if ret:
            # Display the frame in the GUI
            self.photo = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.photo = tk.PhotoImage(image=tk.Image.fromarray(self.photo))
            self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
            
        self.window.after(10, self.update)
        
        
# Create a window and pass it to the WebcamApp class
root = tk.Tk()
app = WebcamApp(root, "Webcam Application")