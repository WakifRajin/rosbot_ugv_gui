import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QLabel, QProgressBar
)
from PyQt5.QtGui import QImage, QPixmap, QColor, QLinearGradient, QPainter, QFont, QPen
from PyQt5.QtCore import Qt, QTimer, QPoint
from rclpy.qos import QoSProfile


class CyberpunkFrame(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def paintEvent(self, event):
        painter = QPainter(self)
        gradient = QLinearGradient(QPoint(0, 0), QPoint(0, self.height()))
        gradient.setColorAt(0, QColor(40, 2, 55))
        gradient.setColorAt(1, QColor(92, 15, 75))
        painter.fillRect(self.rect(), gradient)

        # Neon grid
        pen = QPen(QColor(155, 25, 245, 50))
        painter.setPen(pen)
        spacing = 20
        for x in range(0, self.width(), spacing):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), spacing):
            painter.drawLine(0, y, self.width(), y)


class CameraControlGUI(Node):
    def __init__(self, camera_label, battery_bar, status_label):
        super().__init__('erc_camera_gui')
        self.bridge = CvBridge()
        self.camera_topic = '/camera/image_raw'
        self.camera_label = camera_label
        self.battery_bar = battery_bar
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.status_label = status_label

        self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.create_subscription(BatteryState, '/battery/battery_status', self.battery_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #status labeler
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.check_cmd_vel_connection)
        self.status_timer.start(500)  # every 0.5 seconds

    def battery_callback(self, msg):
        voltage = msg.voltage
        percentage = min(max(int((voltage - 10.5) / (12.6 - 10.5) * 100), 0), 100)
        self.battery_bar.setValue(percentage)

    def check_cmd_vel_connection(self):
        info = self.get_publishers_info_by_topic('/battery/battery_status')
        if info:  # if anyone is subscribed or publishing
            self.status_label.setText("🟢 Connected")
            self.status_label.setStyleSheet("color: lightgreen; font-weight: bold; font-size: 16px;")
        else:
            self.status_label.setText("🔴 Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold; font-size: 16px;")


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            with self.image_lock:
                self.latest_image = rgb_image
        except Exception as e:
            print(f"CV Bridge error: {e}")

    def update_camera_view(self):
        pixmap = QPixmap(self.camera_label.size())
        with self.image_lock:
            img = self.latest_image

        if img is not None:
            h, w, ch = img.shape
            bytes_per_line = ch * w
            q_img = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_img).scaled(
                self.camera_label.width(), self.camera_label.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        else:
            pixmap.fill(Qt.black)
            painter = QPainter(pixmap)
            painter.setPen(QColor(255, 255, 255))
            painter.setFont(QFont("Courier New", 14, QFont.Bold))
            painter.drawText(pixmap.rect(), Qt.AlignCenter,
                             f"No camera feed\ndetected on '{self.camera_topic}'")
            painter.end()

        self.camera_label.setPixmap(pixmap)

    def send_cmd(self, linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)


def create_main_window():
    main_window = QWidget()
    main_window.setWindowTitle("ROVER CONTROL PANEL")
    main_window.setGeometry(100, 100, 980, 800)

    # Background
    background = CyberpunkFrame(main_window)
    background.setGeometry(0, 0, 980, 900)

    layout = QVBoxLayout(main_window)
    layout.setContentsMargins(20, 20, 20, 20)
    layout.setSpacing(20)

    # status bar
    # layout = QHBoxLayout(main_window)
    # layout.setContentsMargins(20, 20, 20, 20)
    # layout.setSpacing(20)

    status_label = QLabel("🔴 Disconnected")
    status_label.setStyleSheet("color: red; font-weight: bold; font-size: 16px;")
    status_label.setAlignment(Qt.AlignCenter)
    layout.addWidget(status_label)

    # Battery Bar
    battery_layout = QHBoxLayout()
    battery_label = QLabel("POWER CELL:")
    battery_label.setStyleSheet("color: #9f19f5; font-family: 'Courier New'; font-weight: bold;")
    battery_bar = QProgressBar()
    battery_bar.setRange(0, 100)
    battery_bar.setStyleSheet("""
        QProgressBar {
            border: 2px solid #9f19f5;
            border-radius: 5px;
            background: #1a0a24;
            color: white;
        }
        QProgressBar::chunk {
            background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #6a11cb, stop:1 #2575fc);
        }
    """)
    battery_layout.addWidget(battery_label)
    battery_layout.addWidget(battery_bar)
    layout.addLayout(battery_layout)

    # Camera Feed
    camera_label = QLabel()
    camera_label.setMinimumSize(640, 480)
    camera_label.setStyleSheet("background-color: black; border: 3px solid #9f19f5;")
    layout.addWidget(camera_label)

    # Controls
    control_layout = QGridLayout()
    buttons = [
        ("▲ FORWARD", lambda: gui.send_cmd(0.5, 0.0), 0, 1),
        ("◀ LEFT", lambda: gui.send_cmd(0.0, 0.5), 1, 0),
        ("STOP", lambda: gui.send_cmd(0.0, 0.0), 1, 1),
        ("RIGHT ▶", lambda: gui.send_cmd(0.0, -0.5), 1, 2),
        ("▼ BACK", lambda: gui.send_cmd(-0.5, 0.0), 2, 1)
    ]
    for text, handler, row, col in buttons:
        btn = QPushButton(text)
        btn.setFixedSize(120, 60)
        btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #6a11cb, stop:1 #2575fc);
                border: 2px solid #9f19f5;
                border-radius: 8px;
                color: white;
                font-family: 'Courier New';
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #2575fc, stop:1 #6a11cb);
            }
        """)
        btn.clicked.connect(handler)
        control_layout.addWidget(btn, row, col)
    layout.addLayout(control_layout)

    return main_window, camera_label, battery_bar, status_label


def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()

    app = QApplication(sys.argv)
    app.setFont(QFont("Courier New", 10, QFont.Bold))

    window, camera_label, battery_bar, status_label = create_main_window()

    # Enable key events
    window.setFocusPolicy(Qt.StrongFocus)

    # Define keypress behavior
    def handle_key_press(event):
        key = event.key()
        if key == Qt.Key_W:
            gui.send_cmd(0.5, 0.0)     # Forward
        elif key == Qt.Key_S:
            gui.send_cmd(-0.5, 0.0)    # Backward
        elif key == Qt.Key_A:
            gui.send_cmd(0.0, 0.5)     # Left
        elif key == Qt.Key_D:
            gui.send_cmd(0.0, -0.5)    # Right
        elif key == Qt.Key_Space:
            gui.send_cmd(0.0, 0.0)     # Stop

    # Inject into the window
    window.keyPressEvent = handle_key_press

    global gui
    gui = CameraControlGUI(camera_label, battery_bar, status_label)

    # ROS spinning thread
    thread = threading.Thread(target=ros_spin_thread, args=(gui,))
    thread.daemon = True
    thread.start()

    # QTimer for GUI refresh
    timer = QTimer()
    timer.timeout.connect(gui.update_camera_view)
    timer.start(50)

    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()