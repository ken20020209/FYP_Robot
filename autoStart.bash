#!/bin/bsah

sleep 8
# 切換到您的儲存庫目錄
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

# 更新遠端參考
#bash update.bash

# start startConnector python
#python3 startConnector.py

# gnome-terminal -- bash -c "bash update.bash;python3 startConnector.py;exec bash"

gnome-terminal -- bash -c "docker run -it --rm --net host --device /dev/myserial:/dev/myserial --device /dev/video0:/dev/video0 -e ROBOT_NAME=muto_s2_1 -e ROBOT_TYPE=muto_s2 ken20020209/fyp_robot:muto_s2;exec bash"