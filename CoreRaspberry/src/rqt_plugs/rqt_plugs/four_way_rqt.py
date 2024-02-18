from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QCheckBox, QPushButton
from std_msgs.msg import String
import rclpy

class FourWayStateRqtPlugin(QWidget):
    def __init__(self, context):
        super(FourWayStateRqtPlugin, self).__init__()

        self.publisher = None

        self.checkbox1 = QCheckBox("Option 1")
        self.checkbox2 = QCheckBox("Option 2")
        self.publish_button = QPushButton("Publish")

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.checkbox1)
        self.layout.addWidget(self.checkbox2)
        self.layout.addWidget(self.publish_button)

        self.setLayout(self.layout)

        self.publish_button.clicked.connect(self.publish_four_way_state)

        self.node = rclpy.create_node('four_way_state_rqt_plugin')

        # Adjust the topic name as needed
        self.publisher = self.node.create_publisher(String, 'four_way_state', 10)

        context.add_widget(self)

    def publish_four_way_state(self):
        option1_state = "red" if self.checkbox1.isChecked() else "green"
        option2_state = "yellow" if self.checkbox2.isChecked() else "green"

        message = String()
        message.data = f"{option1_state},{option2_state}"
        self.publisher.publish(message)

def main():
    rclpy.init()
    context = None  # Replace this with your actual RQT context if needed
    plugin = FourWayStateRqtPlugin(context)

    try:
        # This is a placeholder main loop for the RQT plugin
        while not rclpy.ok():
            rclpy.spin_once(plugin.node, timeout_sec=0.1)
            # Add any additional logic or updates needed in the main loop

    except KeyboardInterrupt:
        pass

    plugin.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
