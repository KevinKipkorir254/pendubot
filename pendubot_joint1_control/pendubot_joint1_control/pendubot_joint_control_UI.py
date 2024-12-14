import sys
import math
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QPainter, QPen, QColor
from std_msgs.msg import Float32


# ROS 2 Node to publish the radians value
class CirclePublisher(Node):
    def __init__(self):
        super().__init__("circle_position_publisher")
        self.publisher_ = self.create_publisher(Float32, "circle_position", 10)
        self.get_logger().info("Circle Position Publisher Node has been started.")

    def publish_position(self, radians):
        msg = Float32()
        msg.data = radians
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published degrees: {math.degrees(radians):.2f}")


# Custom widget for the circular slider
class CircularSlider(QWidget):
    def __init__(self, ros_publisher):
        super().__init__()
        self.radius = 100
        self.angle = 0  # Current angle in degrees
        self.ros_publisher = ros_publisher  # ROS 2 publisher node
        self.is_mouse_pressed = False  # Flag to track if mouse is pressed
        self.last_mouse_position = QPoint(0, 0)
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Circular Slider")
        self.setGeometry(100, 100, 300, 300)
        self.label = QLabel("Angle: 0° | Radians: 0.00", self)
        self.label.move(10, 10)
        self.label.setAlignment(Qt.AlignCenter)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw the circle
        pen = QPen(Qt.gray, 8)
        painter.setPen(pen)
        painter.drawEllipse(self.rect().center(), self.radius, self.radius)

        # Calculate dot position
        angle_rad = math.radians(self.angle)
        center = self.rect().center()
        dot_x = center.x() + self.radius * math.cos(angle_rad - math.pi / 2)
        dot_y = center.y() + self.radius * math.sin(angle_rad - math.pi / 2)

        # Draw the dot
        pen.setColor(Qt.red)
        painter.setPen(pen)
        painter.setBrush(Qt.red)
        painter.drawEllipse(QPoint(int(dot_x), int(dot_y)), 8, 8)

        # Display angle and radians
        painter.setPen(Qt.black)
        painter.drawText(10, 20, f"Angle: {self.angle:.1f}° | Radians: {math.radians(self.angle):.2f}")

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_mouse_pressed = True
            self.last_mouse_position = event.pos()  # Track the last position of the mouse

    def mouseMoveEvent(self, event):
        if self.is_mouse_pressed:
            self.update_angle(event.pos())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_mouse_pressed = False
        
    def update_angle(self, pos):
        center = self.rect().center()
        delta_x = pos.x() - center.x()
        delta_y = pos.y() - center.y()  # Invert Y-axis (Qt's Y increases downwards)

        # Use atan2 to calculate the angle (handles all quadrants)
        angle_rad = math.atan2(delta_y, delta_x) + math.radians(90)
        angle_deg = (math.degrees(angle_rad) + 360) % 360  # Ensure positive angles

        self.angle = angle_deg  # Update angle for painting
        radians = math.radians(self.angle)
        self.label.setText(f"Angle: {int(self.angle)}° | Radians: {radians:.2f}")
        self.update()

        # Publish the angle in radians to the ROS 2 topic
        self.ros_publisher.publish_position(radians)



# Main application
def main():
    rclpy.init()
    ros_publisher = CirclePublisher()

    app = QApplication(sys.argv)
    window = QWidget()
    layout = QVBoxLayout()

    circular_slider = CircularSlider(ros_publisher)
    layout.addWidget(circular_slider)
    window.setLayout(layout)

    window.setWindowTitle("Circular Slider ROS2 UI")
    window.show()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        ros_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
