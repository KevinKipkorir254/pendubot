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
        self.angle = 90  # Start at the top (90°)
        self.radius = 100  # Circle radius
        self.dot_radius = 8  # Radius for the dot
        self.ros_publisher = ros_publisher  # ROS 2 publisher node
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

        try:
            # Draw base circle
            pen = QPen(QColor(200, 200, 200), 10)
            painter.setPen(pen)
            center = self.rect().center()
            painter.drawEllipse(center, self.radius, self.radius)

            # Draw slider arc
            pen.setColor(QColor(255, 0, 255))  # Purple arc
            painter.setPen(pen)
            painter.drawArc(int(center.x() - self.radius),
                            int(center.y() - self.radius),
                            int(2 * self.radius),
                            int(2 * self.radius),
                            90 * 16, -int(self.angle * 16))

            # Draw the dot at the current angle
            angle_rad = math.radians(self.angle)
            dot_x = center.x() + self.radius * math.cos(angle_rad - math.pi / 2)
            dot_y = center.y() + self.radius * math.sin(angle_rad - math.pi / 2)

            painter.setBrush(QColor(255, 0, 255))  # Purple dot
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPoint(int(dot_x), int(dot_y)), self.dot_radius, self.dot_radius)

        finally:
            painter.end()


    def mousePressEvent(self, event):
        self.update_angle(event.pos())
        #self.paintEvent(event)

    def mouseMoveEvent(self, event):
        self.update_angle(event.pos())
        #self.paintEvent(event)
        
    def update_angle(self, pos):
        center = self.rect().center()
        delta_x = pos.x() - center.x()
        delta_y = center.y() - pos.y()  # Invert Y-axis (Qt's Y increases downwards)

        # Use atan2 to calculate the angle (handles all quadrants)
        angle_rad = math.atan2(delta_y, delta_x)
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
