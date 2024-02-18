import tkinter as tk
from std_msgs.msg import String
import rclpy

class ROS2CheckboxApp:
    def __init__(self):
        self.node = rclpy.create_node('ros2_checkbox_app')

        # Create a publisher for the "four_way_state" topic
        self.publisher = self.node.create_publisher(String, 'four_way_state', 10)

        # Create the main Tkinter window
        self.root = tk.Tk()
        self.root.title("ROS2 Tkinter Checkbox Publisher")

        # Create variables to store the state of checkboxes
        self.checkbox_var1 = tk.BooleanVar()
        self.checkbox_var2 = tk.BooleanVar()

        # Create checkboxes with the on_checkbox_change function as the command
        checkbox1 = tk.Checkbutton(self.root, text="Option 1", variable=self.checkbox_var1, command=self.on_checkbox_change)
        checkbox1.pack(padx=10, pady=5)

        checkbox2 = tk.Checkbutton(self.root, text="Option 2", variable=self.checkbox_var2, command=self.on_checkbox_change)
        checkbox2.pack(padx=10, pady=5)

        # Run the Tkinter event loop
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)  # Handle window close event
        self.root.mainloop()

    def on_checkbox_change(self):
        # Get the current state of checkboxes
        option1_state = "red" if self.checkbox_var1.get() else "green"
        option2_state = "yellow" if self.checkbox_var2.get() else "green"

        # Publish the current state to the "four_way_state" topic
        message = String()
        message.data = f"{option1_state},{option2_state}"
        self.publisher.publish(message)

    def on_close(self):
        # Close the ROS 2 node when the Tkinter window is closed
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

if __name__ == '__main__':
    rclpy.init()
    app = ROS2CheckboxApp()
