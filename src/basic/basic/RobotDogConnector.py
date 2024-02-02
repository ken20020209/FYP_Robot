
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
from service.srv import RegisterDog,UnregisterDog

class RobotDogConnector(Node):
    
    name:str 
    port:int = 0
    rosDomainId:int = 0

    def __init__(self,name='RobotDogConnector'):
        super().__init__(name)
        self.name=self.get_name()
        self.registerClient = self.create_client(RegisterDog,'dog/reg')
        self.unregisterDogClient= self.create_client(UnregisterDog,'dog/list')

        self.registerDog()

    def startController(self):
        raise NotImplementedError

    def registerDog(self):
        # self.get_logger().info('waiting for service')
        while not self.registerClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service available')
        request = RegisterDog.Request()
        request.dog_id = self.name
        
        future = self.registerClient.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        # if future.done():
        if future.result() is not None:
            self.get_logger().info('result of registerDog domain id: %s' % future.result().id)
            id=future.result().id
            if id ==-1:
                self.get_logger().info('registerDog failed')
                return
            self.rosDomainId=id
            self.get_logger().info('registerDog success')

            #TODO: start controller
            # self.startController()
        else:
            self.get_logger().error('exception while calling registerDog service: %r' % future.exception())
        


    def unregisterDog(self):
        while not self.unregisterDogClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service available')
        request = UnregisterDog.Request()
        request.dog_id = self.name
        future = self.unregisterDogClient.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('result of unregisterDog: %s' % future.result().id)
            id=future.result().id
            if id ==0:
                self.get_logger().info('unregisterDog failed')
                return
            self.get_logger().info('unregisterDog success')
    
        else:
            self.get_logger().error('exception while calling unregisterDog service: %r' % future.exception())

def main(args=None):
    rclpy.init(args=args)
    robotDogConnector = RobotDogConnector()
    print('init robotDogConnector node')
    rclpy.spin(robotDogConnector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()