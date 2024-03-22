#common lib
import os
import sys
import time
import cv2 as cv
from cv_bridge import CvBridge
import numpy

# from .lib.DOGZILLALib import DOGZILLA

#ros2 lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32;
from geometry_msgs.msg import Twist

# print("import lib success")

class Action(Node):
    def __init__(self,name='Action'):
        super().__init__(name)
        self.subAction= self.create_subscription(Int32,'action',self.action_callback,10)
        self.subCmdVel= self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        
    def action_callback(self,msg):
        self._logger.info("action_callback")
        self._logger.info(str(msg.data))
    def cmd_vel_callback(self,msg):
        self._logger.info("cmd_vel_callback")
        self._logger.info(str(msg.linear.x))
        self._logger.info(str(msg.linear.y))
        self._logger.info(str(msg.angular.z))


        
        

        

def main():
    rclpy.init()
    action = Action()
    rclpy.spin(action)
    action.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
