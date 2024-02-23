import os
from time import sleep
from threading import Thread
from multiprocessing import Process,Manager,Value


# os.environ['ROS_DOMAIN_ID'] = '16'

# os.system(f"source /home/ken20020209/fyp/FYP_Server/ROS_Server_WS/install/setup.bash")
# os.system(f"ros2 launch basic Slam.launch.py ekf_node:=true")

#output my current file directory not pwd path
print(os.path.dirname(os.path.abspath(__file__)))

# print(os.path.dirname(os.path.abspath(__file__)))



os.system(f"/bin/bash -c 'source {os.path.dirname(os.path.abspath(__file__))}/install/setup.bash && ros2 launch basic RobotDogConnector.launch.py'")
