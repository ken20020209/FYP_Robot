import os
from time import sleep
from threading import Thread
from multiprocessing import Process,Manager,Value

import json


# os.environ['ROS_DOMAIN_ID'] = '16'

# os.system(f"source /home/ken20020209/fyp/FYP_Server/ROS_Server_WS/install/setup.bash")
# os.system(f"ros2 launch basic Slam.launch.py ekf_node:=true")

#output my current file directory not pwd path
print(os.path.dirname(os.path.abspath(__file__)))

# get name for env file
name = "dog_s2_0"
robot_type = "dog_s2"
with open(f"{os.path.dirname(os.path.abspath(__file__))}/env.json") as f:
    data = json.load(f)
    name = data["name"]
    robot_type = data["type"]
# print(os.path.dirname(os.path.abspath(__file__)))

os.system(f" sudo systemctl stop yahboom_oled.service")
os.system(f"sudo systemctl stop YahboomStart.service")
os.system(f"/bin/bash -c 'source {os.path.dirname(os.path.abspath(__file__))}/install/setup.bash && ros2 launch basic RobotDogConnector.launch.py name:={name} tpye:={robot_type}'")
