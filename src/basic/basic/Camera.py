#common lib
import os
import sys
import time
import cv2 as cv
from cv_bridge import CvBridge
import numpy

#ros2 lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Bool
from sensor_msgs.msg import Image, CompressedImage

# print("import lib success")

class Camera(Node):
    def __init__(self,name='Camera'):
        super().__init__(name)
        # self.publisher_ = self.create_publisher(Image, 'camera/raw', 10)
        self.subscriptions_ = self.create_subscription(Bool, 'camera/enable', self.enable_callback, 10)
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/raw', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.i = 0
        self.enable=True

        # get camera device
        self.cap = cv.VideoCapture(0)

        # get video file
        # self.cap = cv.VideoCapture("./video/file_example_MP4_640_3MG.mp4")
        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()
        else:
            print("Camera open success")

        self.bridge = CvBridge()

        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    
    def enable_callback(self, msg):
        self.enable=msg.data
    def timer_callback(self):
        if not self.enable:
            return
        ret, frame = self.cap.read()

        # cv.imshow("frame", frame)
        # msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        msg = self.bridge.cv2_to_compressed_imgmsg(frame)

        # print(msg)
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
    
