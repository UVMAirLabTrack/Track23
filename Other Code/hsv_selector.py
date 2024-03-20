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
        self.lower_hue = tk.DoubleVar()
        self.upper_hue = tk.DoubleVar()
        self.lower_saturation = tk.DoubleVar()
        self.upper_saturation = tk.DoubleVar()
        self.lower_value = tk.DoubleVar()
        self.upper_value = tk.DoubleVar()
        
        # Set default HSV ranges
        self.lower_hue.set(110)
        self.upper_hue.set(130)
        self.lower_saturation.set(50)
        self.upper_saturation.set(255)
        self.lower_value.set(50)
        self.upper_value.set(255)
        
        # Initialize lower and upper bounds
        self.lower_bound = np.array([self.lower_hue.get(), self.lower_saturation.get(), self.lower_value.get()])
        self.upper_bound = np.array([self.upper_hue.get(), self.upper_saturation.get(), self.upper_value.get()])
        
        self.create_widgets()
        self.update_feed()
        
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def create_widgets(self):
        # Hue sliders
        self.lower_hue_label = tk.Label(self.window, text="Lower Hue:")
        self.lower_hue_label.pack()
        self.lower_hue_slider = tk.Scale(self.window, from_=0, to=179, orient=tk.HORIZONTAL, variable=self.lower_hue)
        self.lower_hue_slider.pack()
        
        self.upper_hue_label = tk.Label(self.window, text="Upper Hue:")
        self.upper_hue_label.pack()
        self.upper_hue_slider = tk.Scale(self.window, from_=0, to=179, orient=tk.HORIZONTAL, variable=self.upper_hue)
        self.upper_hue_slider.pack()
        
        # Saturation sliders
        self.lower_saturation_label = tk.Label(self.window, text="Lower Saturation:")
        self.lower_saturation_label.pack()
        self.lower_saturation_slider = tk.Scale(self.window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.lower_saturation)
        self.lower_saturation_slider.pack()
        
        self.upper_saturation_label = tk.Label(self.window, text="Upper Saturation:")
        self.upper_saturation_label.pack()
        self.upper_saturation_slider = tk.Scale(self.window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.upper_saturation)
        self.upper_saturation_slider.pack()
        
        # Value sliders
        self.lower_value_label = tk.Label(self.window, text="Lower Value:")
        self.lower_value_label.pack()
        self.lower_value_slider = tk.Scale(self.window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.lower_value)
        self.lower_value_slider.pack()
        
        self.upper_value_label = tk.Label(self.window, text="Upper Value:")
        self.upper_value_label.pack()
        self.upper_value_slider = tk.Scale(self.window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.upper_value)
        self.upper_value_slider.pack()
        
        # Update button
        self.update_button = tk.Button(self.window, text="Update", command=self.update_hsv_values)
        self.update_button.pack()
        
        # Canvas for displaying video feed
        self.canvas = tk.Canvas(self.window, width=640, height=480)
        self.canvas.pack()
        
    def update_hsv_values(self):
        # Get HSV range values from sliders
        lower_hue = int(self.lower_hue.get())
        upper_hue = int(self.upper_hue.get())
        lower_saturation = int(self.lower_saturation.get())
        upper_saturation = int(self.upper_saturation.get())
        lower_value = int(self.lower_value.get())
        upper_value = int(self.upper_value.get())
        
        # Update HSV range
        self.lower_bound = np.array([lower_hue, lower_saturation, lower_value])
        self.upper_bound = np.array([upper_hue, upper_saturation, upper_value])
        
    def update_feed(self):
        ret, frame = self.cap.read()
        if ret:
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Apply HSV range filter
            mask = cv2.inRange(frame_hsv, self.lower_bound, self.upper_bound)
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

