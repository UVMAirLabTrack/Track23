import cv2
import numpy as np
import tkinter as tk
from tkinter import messagebox, ttk
from PIL import Image, ImageTk

class HSVAdjustmentApp:
    def __init__(self, window, window_title):
        self.window = window
        self.window.title(window_title)
        
        self.cap = cv2.VideoCapture(0)
        
        # Initialize HSV range variables
        self.hue = tk.DoubleVar()
        self.saturation = tk.DoubleVar()
        self.value = tk.DoubleVar()
        self.bandwidth = tk.DoubleVar()
        self.band_center = tk.DoubleVar()
        
        # Set default values
        self.hue.set(110)
        self.saturation.set(50)
        self.value.set(50)
        self.bandwidth.set(30)
        self.band_center.set(120)
        
        self.create_widgets()
        self.update_feed()
        
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def create_widgets(self):
        # Canvas for displaying video feed
        self.canvas = tk.Canvas(self.window, width=640, height=480)
        self.canvas.pack()
        
        # Create a separate window for sliders
        self.slider_window = tk.Toplevel(self.window)
        self.slider_window.title("HSV Sliders")
        
        # Hue slider
        self.hue_label = tk.Label(self.slider_window, text="Hue:")
        self.hue_label.pack()
        self.hue_slider = tk.Scale(self.slider_window, from_=0, to=179, orient=tk.HORIZONTAL, variable=self.hue)
        self.hue_slider.pack()
        
        # Saturation slider
        self.saturation_label = tk.Label(self.slider_window, text="Saturation:")
        self.saturation_label.pack()
        self.saturation_slider = tk.Scale(self.slider_window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.saturation)
        self.saturation_slider.pack()
        
        # Value slider
        self.value_label = tk.Label(self.slider_window, text="Value:")
        self.value_label.pack()
        self.value_slider = tk.Scale(self.slider_window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.value)
        self.value_slider.pack()
        
        # Bandwidth slider
        self.bandwidth_label = tk.Label(self.slider_window, text="Bandwidth:")
        self.bandwidth_label.pack()
        self.bandwidth_slider = tk.Scale(self.slider_window, from_=0, to=100, orient=tk.HORIZONTAL, variable=self.bandwidth)
        self.bandwidth_slider.pack()
        
        # Band center slider
        self.band_center_label = tk.Label(self.slider_window, text="Band Center:")
        self.band_center_label.pack()
        self.band_center_slider = tk.Scale(self.slider_window, from_=0, to=179, orient=tk.HORIZONTAL, variable=self.band_center)
        self.band_center_slider.pack()
        
        # Update button
        self.update_button = tk.Button(self.slider_window, text="Update", command=self.update_hsv_values)
        self.update_button.pack()
        
    def update_hsv_values(self):
        # Get HSV range values from sliders
        hue = int(self.hue.get())
        saturation = int(self.saturation.get())
        value = int(self.value.get())
        bandwidth = int(self.bandwidth.get())
        band_center = int(self.band_center.get())
        
        # Calculate lower and upper bounds
        lower_hue = (band_center - bandwidth // 2) % 180
        upper_hue = (band_center + bandwidth // 2) % 180
        
        # Update sliders
        self.hue.set(hue)
        self.saturation.set(saturation)
        self.value.set(value)
        self.bandwidth.set(bandwidth)
        self.band_center.set(band_center)
        
   def update_feed(self):
        ret, frame = self.cap.read()
        if ret:
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Get HSV range values from sliders
            hue = int(self.hue.get())
            saturation = int(self.saturation.get())
            value = int(self.value.get())
            bandwidth = int(self.bandwidth.get())
            band_center = int(self.band_center.get())
            
            # Calculate lower and upper bounds
            lower_hue = (band_center - bandwidth // 2) % 180
            upper_hue = (band_center + bandwidth // 2) % 180
            
            # Apply HSV range filter
            lower_bound = np.array([lower_hue, saturation, value])
            upper_bound = np.array([upper_hue, 255, 255])
            mask = cv2.inRange(frame_hsv, lower_bound, upper_bound)
            result = cv2.bitwise_and(frame, frame, mask=mask)
            
            # Convert image format
            img = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (640, 480))
            img = Image.fromarray(img)
            imgtk = ImageTk.PhotoImage(image=img)
            
            # Update canvas with filtered image
            self.canvas.imgtk = imgtk
            self.canvas.create_image(0, 0, anchor=tk.NW, image=imgtk)
            
        # Repeat the update process
        self.window.after(10, self.update_feed)
        
    def on_closing(self):
        # Release the webcam capture and close the window
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.cap.release()
            self.window.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = HSVAdjustmentApp(root, "HSV Adjustment")
    root.mainloop()
