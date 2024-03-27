#!/bin/bash

# 切換到您的儲存庫目錄 
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

# 更新遠端參考
git fetch

# 獲取本地和遠端分支的最新提交
LOCAL=$(git rev-parse @)
REMOTE=$(git rev-parse @{u})

# 比較本地和遠端分支
if [ $LOCAL = $REMOTE ]; then
    echo "分支已經是最新的。"
else
    echo "分支需要更新。"
    # 如果需要，這裡可以執行 git pull 或其他命令
    git pull
    # colcon build 
    # colcon build
    # echo "colcon build 完成"

    docker build -t muto_ros2:latest -f docker/Dockerfile .
fi
