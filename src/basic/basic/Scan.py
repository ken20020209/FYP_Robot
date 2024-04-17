import rclpy   #python for ros2
from rclpy.node import Node   #import
from std_msgs.msg import String   #String message
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import roslibpy
import json
# import .lib.DOGZILLALib.DOGZILLA as dog
# from .lib import DOGZILLALib as dog

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_sub')
        # push scan
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)


        self.ros=roslibpy.Ros(host='localhost',port=9090)
        self.scan_sub = roslibpy.Topic(self.ros,'scan','sensor_msgs/LaserScan')
        self.scan_sub.subscribe(self.scan_sub_process)
        self.ros.run()

        
    def scan_sub_process(self,msg):

        print(msg)
        header=Header()
        header.frame_id=msg['header']['frame_id']
        header.stamp.sec=msg['header']['stamp']['secs']
        header.stamp.nanosec=msg['header']['stamp']['nsecs']

        scan=LaserScan()

        scan.header=header
        scan.angle_min=msg['angle_min']
        scan.angle_max=msg['angle_max']
        scan.angle_increment=msg['angle_increment']
        scan.time_increment=msg['time_increment']
        scan.scan_time=msg['scan_time']
        scan.range_min=msg['range_min']
        scan.range_max=msg['range_max']
        scan.ranges=msg['ranges']
        scan.intensities=msg['intensities']


        self.scan_pub.publish(scan)
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()	
