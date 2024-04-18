#!/bin/bash

. /opt/ros/melodic/setup.sh
echo 127.0.0.1 nx >> /etc/hosts

roslaunch rosbridge_server rosbridge_websocket.launch