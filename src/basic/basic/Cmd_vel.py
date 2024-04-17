import rclpy   #python for ros2
from rclpy.node import Node   #import
from std_msgs.msg import String   #String message
from geometry_msgs.msg import Twist
import roslibpy
import json
# import .lib.DOGZILLALib.DOGZILLA as dog
# from .lib import DOGZILLALib as dog

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_sub')
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.cmdvel_callback,1)

        self.ros=roslibpy.Ros(host='localhost',port=9090)
        self.cmd_vel_topic = roslibpy.Topic(self.ros,'cmd_vel','geometry_msgs/Twist')
        self.ros.run()

        
    def cmdvel_callback(self,msg):
        self.vel_x_=msg.linear.x
        self.vel_y_=msg.linear.y
        self.angular_z_=msg.angular.z

        # publish to rosbridge
        data={
            'linear':{
                'x':self.vel_x_,
                'y':self.vel_y_,
                'z':0
            },
            'angular':{
                'x':0,
                'y':0,
                'z':self.angular_z_
            }
        }

        message=roslibpy.Message(data)
        self.cmd_vel_topic.publish(message)
            

        #end set dog motion
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()	
