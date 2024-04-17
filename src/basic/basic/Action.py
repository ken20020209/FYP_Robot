#common lib
import os
import sys
import time
from cv_bridge import CvBridge
import numpy


#ros2 lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32;

# print("import lib success")

class Action(Node):
    def __init__(self,name='Action'):
        super().__init__(name)
        self.subAction= self.create_subscription(Int32,'action',self.action_callback,10)
        
    def action_callback(self,msg):
        
        if(msg.data<=0 or msg.data>255):
            print("out of action range 1-8 , 255") 
        else:
            self.dog.action(msg.data)

        


def main():
    rclpy.init()
    action = Action()
    rclpy.spin(action)
    action.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
