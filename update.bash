#!/bin/bash

# 切換到您的儲存庫目錄
cd /path/to/your/repo

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
fi

# # git pull
# git pull
# if [ $? -eq 0 ]; then
#     echo "git pull successful"
#     # colcon build --symlink-install
#     colcon build
#     if [ $? -eq 0 ]; then
#         echo "colcon build successful"
#         # source install/setup.bash
#         source install/setup.bash
#     else
#         echo "colcon build failed"
#     fi
# else
#     echo "git pull failed"
# fi