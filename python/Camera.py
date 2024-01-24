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
from std_msgs.msg import String
from sensor_msgs.msg import Image

# print("import lib success")

class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

        # get camera device
        self.cap = cv.VideoCapture(0)

        # get video file
        #self.cap = cv.VideoCapture("/video/file_example_MP4_640_3MG.mp4")

        self.bridge = CvBridge()
        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()
        else:
            print("Camera open success")
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        

        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
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
    
