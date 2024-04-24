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
        self.hue_min = tk.DoubleVar()
        self.hue_max = tk.DoubleVar()
        self.saturation_min = tk.DoubleVar()
        self.saturation_max = tk.DoubleVar()
        self.value_min = tk.DoubleVar()
        self.value_max = tk.DoubleVar()
        
        # Set default values
        self.hue_min.set(0)
        self.hue_max.set(179)
        self.saturation_min.set(0)
        self.saturation_max.set(255)
        self.value_min.set(0)
        self.value_max.set(255)
        
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

        # Hue min slider
        self.hue_min_label = tk.Label(self.slider_window, text="Hue Min (0-179):")
        self.hue_min_label.pack()
        self.hue_min_slider = tk.Scale(self.slider_window, from_=0, to=179, orient=tk.HORIZONTAL, variable=self.hue_min, showvalue=1)
        self.hue_min_slider.pack()

        # Hue max slider
        self.hue_max_label = tk.Label(self.slider_window, text="Hue Max (0-179):")
        self.hue_max_label.pack()
        self.hue_max_slider = tk.Scale(self.slider_window, from_=0, to=179, orient=tk.HORIZONTAL, variable=self.hue_max, showvalue=1)
        self.hue_max_slider.pack()

        # Saturation min slider
        self.saturation_min_label = tk.Label(self.slider_window, text="Saturation Min (0-255):")
        self.saturation_min_label.pack()
        self.saturation_min_slider = tk.Scale(self.slider_window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.saturation_min, showvalue=1)
        self.saturation_min_slider.pack()

        # Saturation max slider
        self.saturation_max_label = tk.Label(self.slider_window, text="Saturation Max (0-255):")
        self.saturation_max_label.pack()
        self.saturation_max_slider = tk.Scale(self.slider_window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.saturation_max, showvalue=1)
        self.saturation_max_slider.pack()

        # Value min slider
        self.value_min_label = tk.Label(self.slider_window, text="Value Min (0-255):")
        self.value_min_label.pack()
        self.value_min_slider = tk.Scale(self.slider_window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.value_min, showvalue=1)
        self.value_min_slider.pack()

        # Value max slider
        self.value_max_label = tk.Label(self.slider_window, text="Value Max (0-255):")
        self.value_max_label.pack()
        self.value_max_slider = tk.Scale(self.slider_window, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.value_max, showvalue=1)
        self.value_max_slider.pack()

        # Update button

        # Create a separate window for HSV value entries
        self.entry_window = tk.Toplevel(self.window)
        self.entry_window.title("HSV Values")

        # Hue entry
        self.hue_min_entry_label = tk.Label(self.entry_window, text="Hue Min (0-179):")
        self.hue_min_entry_label.grid(row=0, column=0)
        self.hue_min_entry = tk.Entry(self.entry_window)
        self.hue_min_entry.grid(row=0, column=1)

        self.hue_max_entry_label = tk.Label(self.entry_window, text="Hue Max (0-179):")
        self.hue_max_entry_label.grid(row=1, column=0)
        self.hue_max_entry = tk.Entry(self.entry_window)
        self.hue_max_entry.grid(row=1, column=1)

        # Saturation entry
        self.saturation_min_entry_label = tk.Label(self.entry_window, text="Saturation Min (0-255):")
        self.saturation_min_entry_label.grid(row=2, column=0)
        self.saturation_min_entry = tk.Entry(self.entry_window)
        self.saturation_min_entry.grid(row=2, column=1)

        self.saturation_max_entry_label = tk.Label(self.entry_window, text="Saturation Max (0-255):")
        self.saturation_max_entry_label.grid(row=3, column=0)
        self.saturation_max_entry = tk.Entry(self.entry_window)
        self.saturation_max_entry.grid(row=3, column=1)

        # Value entry
        self.value_min_entry_label = tk.Label(self.entry_window, text="Value Min (0-255):")
        self.value_min_entry_label.grid(row=4, column=0)
        self.value_min_entry = tk.Entry(self.entry_window)
        self.value_min_entry.grid(row=4, column=1)

        self.value_max_entry_label = tk.Label(self.entry_window, text="Value Max (0-255):")
        self.value_max_entry_label.grid(row=5, column=0)
        self.value_max_entry = tk.Entry
        self.value_max_entry = tk.Entry(self.entry_window)
        self.value_max_entry.grid(row=5, column=1)

    # Update button for HSV values
        self.update_entry_button = tk.Button(self.entry_window, text="Update HSV Values", command=self.update_hsv_entries)
        self.update_entry_button.grid(row=6, columnspan=2)

    def update_hsv_entries(self):
        # Get HSV values from entry fields
        hue_min = int(self.hue_min_entry.get())
        hue_max = int(self.hue_max_entry.get())
        saturation_min = int(self.saturation_min_entry.get())
        saturation_max = int(self.saturation_max_entry.get())
        value_min = int(self.value_min_entry.get())
        value_max = int(self.value_max_entry.get())

        # Update slider values
        self.hue_min_slider.set(hue_min)
        self.hue_max_slider.set(hue_max)
        self.saturation_min_slider.set(saturation_min)
        self.saturation_max_slider.set(saturation_max)
        self.value_min_slider.set(value_min)
        self.value_max_slider.set(value_max)

        # Update labels (optional)
        self.update_hue_label(hue_min, hue_max)
        self.update_saturation_label(saturation_min, saturation_max)
        self.update_value_label(value_min, value_max)

    def update_feed(self):
        ret, frame = self.cap.read()
        if ret:
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Get HSV range values from sliders
            hue_min = int(self.hue_min.get())
            hue_max = int(self.hue_max.get())
            saturation_min = int(self.saturation_min.get())
            saturation_max = int(self.saturation_max.get())
            value_min = int(self.value_min.get())
            value_max = int(self.value_max.get())
            
            # Apply HSV range filter
            lower_bound = np.array([hue_min, saturation_min, value_min])
            upper_bound = np.array([hue_max, saturation_max, value_max])
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
        
    def update_hsv_values(self):
        # This function updates the HSV values based on slider positions
        
        # Get HSV range values from sliders
        hue_min = int(self.hue_min.get())
        hue_max = int(self.hue_max.get())
        saturation_min = int(self.saturation_min.get())
        saturation_max = int(self.saturation_max.get())
        value_min = int(self.value_min.get())
        value_max = int(self.value_max.get())
        
        # Update sliders
        self.hue_min.set(hue_min)
        self.hue_max.set(hue_max)
        self.saturation_min.set(saturation_min)
        self.saturation_max.set(saturation_max)
        self.value_min.set(value_min)
        self.value_max.set(value_max)
        
    def on_closing(self):
        # This function is called when the window is closed
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.cap.release()
            self.window.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = HSVAdjustmentApp(root, "HSV Adjustment")
    root.mainloop()
