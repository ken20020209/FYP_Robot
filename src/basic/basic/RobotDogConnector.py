
#common lib
import os
import signal
import sys
import time
import cv2 as cv
from cv_bridge import CvBridge
import subprocess
import numpy



#ros2 lib
import rclpy
from rclpy.node import Node
from message.srv import RegisterDog,UnregisterDog
from message.msg import DogStatus
from std_msgs.msg import Int32
from .lib.DOGZILLALib import DOGZILLA

ServerLife=15
class RobotDogConnector(Node):
    
    name:str 
    port:int = 0
    rosDomainId:int = 0
    controller:subprocess.Popen = None

    def __init__(self,name='RobotDogConnector'):
        super().__init__(name)
        self.name=self.get_namespace()
        self.g_dogzilla = DOGZILLA()
        self.serverLife=ServerLife

        self.g_dogzilla.action(11)

        self.declare_parameter('type','dog_s2')
        self.declare_parameter('discoverServer','127.0.0.1')

        # self.get_logger().info(self.name)
        #create client
        self.registerClient = self.create_client(RegisterDog,'/dog/reg')
        # self.unregisterDogClient= self.create_client(UnregisterDog,'/dog/list')

        #oled
        # self.oled = subprocess.Popen(["python3","./lib/oled_dogzilla.py"])

        #create service
        self.statusTopic = self.create_publisher(DogStatus,'dog/status',10)

        #create subscription
        self.subscription = self.create_subscription(Int32,'/server/status',self.serverStatusCallback ,10)

        self.registerDog()

        #create timer
        self.timer = self.create_timer(1, self.statusCallback)

        

    def serverStatusCallback(self,msg):
        if msg.data==-1:
            self.serverLife=-1
            return
        self.serverLife=ServerLife
    def statusCallback(self):
        msg = DogStatus()
        msg.battery = 100
        msg.status = 1
        self.statusTopic.publish(msg)

        self.serverLife-=1
        if self.serverLife<=0:
            self.g_dogzilla.reset()
            self.get_logger().info('server dead')
            self.stopController()
            
            exit()
            # self.unregisterDog()
            # self.registerDog()
    
    def startController(self,ip):
        self.get_logger().info(f'start controller romain id: {self.rosDomainId}')
        sp_env=os.environ.copy()
        sp_env['ROS_DOMAIN_ID'] = str(self.rosDomainId)
        if(self.get_parameter('discoverServer').value != '127.0.0.1'):
            sp_env['ROS_DISCOVERY_SERVER'] = str(self.get_parameter('discoverServer').value)+f':{11811+self.rosDomainId}'
        if(ip != ''):
            sp_env['ROS_DISCOVERY_SERVER'] = ip+f':{11811+self.rosDomainId}'
        self.controller = subprocess.Popen(["ros2","launch","basic","RobotDogController.launch.py"],env=sp_env)
    def stopController(self):
        self.get_logger().info('stop controller')
        if self.controller is not None:
            self.controller.send_signal(signal.SIGINT)
            self.controller = None

    def registerDog(self):
        self.serverLife=ServerLife
        # self.get_logger().info('waiting for service')
        count = 0
        while not self.registerClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'({count}): service not available, waiting again...')
            count+=1
        self.get_logger().info('service available')
        request = RegisterDog.Request()
        request.dog_id = self.name
        request.type= str(self.get_parameter('type').value)
        
        future = self.registerClient.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        
        def callback(future):
            # if future.done():
            if future.result() is not None:
                self.get_logger().info('result of registerDog domain id: %s' % future.result().id)
                self.get_logger().info('result of registerDog ip: %s' % future.result().ip)
                id=future.result().id
                ip=future.result().ip
                if id ==-1:
                    self.get_logger().info('registerDog failed')
                    
                    exit()
                    return
                self.rosDomainId=id
                self.get_logger().info('registerDog success')

                # start controller
                self.startController(ip)
            else:
                self.get_logger().error('exception while calling registerDog service: %r' % future.exception())
        future.add_done_callback(callback)
        


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
        self.stopController()
    
def main(args=None):
    rclpy.init(args=args)
    robotDogConnector = RobotDogConnector()
    print('init robotDogConnector node')
    rclpy.spin(robotDogConnector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()