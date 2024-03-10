#common lib
import os
import sys
import time
import cv2 as cv
from cv_bridge import CvBridge
import numpy

from .lib.DOGZILLALib import DOGZILLA

#ros2 lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32;
from .lib.oled_dogzilla import main as oled_main

# print("import lib success")

class Oled(Node):
    def __init__(self,name='Oled'):
        super().__init__(name)
        oled_main(self.get_namespace())
        
        

def main():
    rclpy.init()
    oled = Oled()
    rclpy.spin(oled)
    oled.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
