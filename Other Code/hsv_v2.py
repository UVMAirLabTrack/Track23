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
        self.hue_center = tk.DoubleVar()
        self.hue_bandwidth = tk.DoubleVar()
        self.saturation_center = tk.DoubleVar()
        self.saturation_bandwidth = tk.DoubleVar()
        self.value_center = tk.DoubleVar()
        self.value_bandwidth = tk.DoubleVar()
        
        # Set default values
        self.hue_center.set(120)
        self.hue_bandwidth.set(20)
        self.saturation_center.set(128)
        self.saturation_bandwidth.set(50)
        self.value_center.set(128)
        self.value_bandwidth.set(50)
        
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

        # Hue center slider
        self.hue_center_label = tk.Label(self.slider_window, text="Hue (Center: {0})".format(self.hue_center.get()))
        self.hue_center_label.pack()
        self.hue_center_slider = tk.Scale(self.slider_window, from_=0, to=179, orient=tk.HORIZONTAL, variable=self.hue_center, command=self.update_hue_label)
        self.hue_center_slider.pack()

        # Hue bandwidth slider
        self.hue_bandwidth_label = tk.Label(self.slider_window, text="Hue Bandwidth:")
        self.hue_bandwidth_label.pack()
        self.hue_bandwidth_slider = tk.Scale(self.slider_window, from_=0, to=179, orient=tk.HORIZONTAL, variable=self.hue_bandwidth)
        self.hue_bandwidth_slider.pack()

        # Saturation center slider
        self.saturation_center_label = tk.Label(self.slider_window, text="Saturation (Center: {0})".format(self.saturation_center.get()))
        self.saturation_center_label.pack()
        self.saturation_center_slider = tk.Scale(self.slider_window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.saturation_center, command=self.update_saturation_label)
        self.saturation_center_slider.pack()

        # Saturation bandwidth slider
        self.saturation_bandwidth_label = tk.Label(self.slider_window, text="Saturation Bandwidth:")
        self.saturation_bandwidth_label.pack()
        self.saturation_bandwidth_slider = tk.Scale(self.slider_window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.saturation_bandwidth)
        self.saturation_bandwidth_slider.pack()

        # Value center slider
        self.value_center_label = tk.Label(self.slider_window, text="Value (Center: {0})".format(self.value_center.get()))
        self.value_center_label.pack()
        self.value_center_slider = tk.Scale(self.slider_window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.value_center, command=self.update_value_label)
        self.value_center_slider.pack()

        # Value bandwidth slider
        self.value_bandwidth_label = tk.Label(self.slider_window, text="Value Bandwidth:")
        self.value_bandwidth_label.pack()
        self.value_bandwidth_slider = tk.Scale(self.slider_window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.value_bandwidth)
        self.value_bandwidth_slider.pack()

        # Update button
        self.update_button = tk.Button(self.slider_window, text="Update", command=self.update_hsv_values)
        self.update_button.pack()

    def update_hue_label(self, value):
        center = int(value)
        bandwidth = int(self.hue_bandwidth.get())
        lower_bound = max((center - bandwidth // 2) % 180,0)
        upper_bound = min((center + bandwidth // 2) % 180,179)
        self.hue_center_label.config(text="Hue (Center: {0}, Lower: {1}, Upper: {2})".format(center, lower_bound, upper_bound))

    def update_saturation_label(self, value):
        center = int(value)
        bandwidth = int(self.saturation_bandwidth.get())
        lower_bound = max(center - bandwidth // 2, 0)
        upper_bound = min(center + bandwidth // 2, 255)
        self.saturation_center_label.config(text="Saturation (Center: {0}, Lower: {1}, Upper: {2})".format(center, lower_bound, upper_bound))

    def update_value_label(self, value):
        center = int(value)
        bandwidth = int(self.value_bandwidth.get())
        lower_bound = max(center - bandwidth // 2, 0)
        upper_bound = min(center + bandwidth // 2, 255)
        self.value_center_label.config(text="Value (Center: {0}, Lower: {1}, Upper: {2})".format(center, lower_bound, upper_bound))

                
    def update_hsv_values(self):
        # Get HSV range values from sliders
        hue_center = int(self.hue_center.get())
        hue_bandwidth = int(self.hue_bandwidth.get())
        saturation_center = int(self.saturation_center.get())
        saturation_bandwidth = int(self.saturation_bandwidth.get())
        value_center = int(self.value_center.get())
        value_bandwidth = int(self.value_bandwidth.get())
        
        # Update sliders
        self.hue_center.set(hue_center)
        self.hue_bandwidth.set(hue_bandwidth)
        self.saturation_center.set(saturation_center)
        self.saturation_bandwidth.set(saturation_bandwidth)
        self.value_center.set(value_center)
        self.value_bandwidth.set(value_bandwidth)
        
    def update_feed(self):
        ret, frame = self.cap.read()
        if ret:
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Get HSV range values from sliders
            hue_center = int(self.hue_center.get())
            hue_bandwidth = int(self.hue_bandwidth.get())
            saturation_center = int(self.saturation_center.get())
            saturation_bandwidth = int(self.saturation_bandwidth.get())
            value_center = int(self.value_center.get())
            value_bandwidth = int(self.value_bandwidth.get())
            
            # Calculate lower and upper bounds for Hue
            lower_hue = max(min((hue_center - hue_bandwidth // 2) % 180, 179), 0)
            upper_hue = max(min((hue_center + hue_bandwidth // 2) % 180, 179), 0)

            
            # Calculate lower and upper bounds for Saturation
            lower_saturation = max(saturation_center - saturation_bandwidth // 2, 0)
            upper_saturation = min(saturation_center + saturation_bandwidth // 2, 255)

            
            # Calculate lower and upper bounds for Value
            lower_value = max(value_center - value_bandwidth // 2, 0)
            upper_value = min(value_center + value_bandwidth // 2, 255)
            
            # Apply HSV range filter
            lower_bound = np.array([lower_hue, lower_saturation, lower_value])
            upper_bound = np.array([upper_hue, upper_saturation, upper_value])
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
