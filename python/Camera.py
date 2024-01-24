#common lib
import os
import sys
import time
import cv2 as cv

#ros2 lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

print("import lib success")

class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0
        self.cap = cv.VideoCapture(0)
    def timer_callback(self):
        ret, frame = self.cap.read()
        msg = Image()
        msg.data = frame
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
