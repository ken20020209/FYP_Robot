import os
from time import sleep
from threading import Thread
from multiprocessing import Process,Manager,Value

import socket
import json


discovery_server = "discoveryserver.ddns.net"
# os.environ['ROS_DOMAIN_ID'] = '16'

# os.system(f"source /home/ken20020209/fyp/FYP_Server/ROS_Server_WS/install/setup.bash")
# os.system(f"ros2 launch basic Slam.launch.py ekf_node:=true")

#output my current file directory not pwd path
# print(os.path.dirname(os.path.abspath(__file__)))

# get name for env file
name = "dog_s2_0"
robot_type = "dog_s2"

name=os.getenv('name',name)
# if os.environ['name']:
#     name = os.environ['name']
# with open(f"{os.path.dirname(os.path.abspath(__file__))}/env.json") as f:
#     data = json.load(f)
#     name = data["name"]
#     robot_type = data["type"]
print(os.path.dirname(os.path.abspath(__file__)))
def get_ip_address(domain_name):
    try:
        return socket.gethostbyname(domain_name)
    except:
        return "127.0.0.1"

# print(get_ip_address(discovery_server))
# os.system(f"sudo systemctl stop yahboom_oled.service")
# os.system(f"sudo systemctl stop YahboomStart.service")

discovery_server_ip=get_ip_address(discovery_server)
print(f"{discovery_server}:{get_ip_address(discovery_server)}")
cmd=f"/bin/bash -c"
cmd+=f" '"
cmd+=f"source {os.path.dirname(os.path.abspath(__file__))}/install/setup.bash"
if(discovery_server_ip!="127.0.0.1"):
    cmd+=f' && export ROS_DISCOVERY_SERVER={discovery_server_ip}:11811'
    # cmd+=f' && export DISCOVERY_SERVER_IP={discovery_server_ip}'
    # cmd+=f' && export DISCOVERY_SERVER_PORT=11811'
    # cmd+=f' && export export FASTRTPS_DEFAULT_PROFILES_FILE={os.path.dirname(os.path.abspath(__file__))}/super_client_configuration_file.xml'
cmd+=f" && ros2 launch basic RobotDogConnector.launch.py name:={name} type:={robot_type} discoverServer:={discovery_server_ip}"
cmd+=f"'"
os.system(cmd)
# os.system(f"/bin/bash -c 'source {os.path.dirname(os.path.abspath(__file__))}/install/setup.bash && ros2 launch basic RobotDogConnector.launch.py name:={name} tpye:={robot_type} discoverServer:={get_ip_address(discovery_server)}'")
