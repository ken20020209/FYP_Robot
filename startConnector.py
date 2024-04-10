import os
from time import sleep
from threading import Thread
from multiprocessing import Process,Manager,Value

from subprocess import Popen

import socket
import json


discovery_server = "discoveryserver.ddns.net"
# os.environ['ROS_DOMAIN_ID'] = '16'

# os.system(f"source /home/ken20020209/fyp/FYP_Server/ROS_Server_WS/install/setup.bash")
# os.system(f"ros2 launch basic Slam.launch.py ekf_node:=true")

#output my current file directory not pwd path
# print(os.path.dirname(os.path.abspath(__file__)))

# get name for env file
name = "muto_s2_0"
robot_type = "muto_s2"

sp_oled=Popen(["ros2","launch","basic","Oled.launch.py"])

#for docker
if os.path.exists(f"{os.path.dirname(os.path.abspath(__file__))}/env.json"):
    with open(f"{os.path.dirname(os.path.abspath(__file__))}/env.json") as f:
        data = json.load(f)
        name = data["name"]
        robot_type = data["type"]
    os.system(f"docker run -it --rm --net host --device /dev/myserial:/dev/myserial --device /dev/video0:/dev/video0 -e ROBOT_NAME={name} -e ROBOT_TYPE={robot_type}  ken20020209/fyp_robot:muto_s2" )
# for local
else:
    name=os.environ['ROBOT_NAME']
    robot_type=os.environ['ROBOT_TYPE']
    print(os.path.dirname(os.path.abspath(__file__)))
    def get_ip_address(domain_name):
        try:
            return socket.gethostbyname(domain_name)
        except:
            return "127.0.0.1"

    # print(get_ip_address(discovery_server))
    os.system(f"sudo systemctl stop yahboom_oled.service")
    os.system(f"sudo systemctl stop YahboomStart.service")

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
    while True:
        os.system(cmd)
        sleep(5)
    # os.system(f"/bin/bash -c 'source {os.path.dirname(os.path.abspath(__file__))}/install/setup.bash && ros2 launch basic RobotDogConnector.launch.py name:={name} tpye:={robot_type} discoverServer:={get_ip_address(discovery_server)}'")
    
sp_oled.terminate()
sp_oled.wait()