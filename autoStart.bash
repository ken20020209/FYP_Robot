#!/bin/bsah

# 切換到您的儲存庫目錄
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

# 更新遠端參考
bash update.bash

# start startConnector python
python3 startConnector.py