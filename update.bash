# git pull 
# if git pull is successful then colcon build --symlink-install
# if colcon build is successful then source install/setup.bash

# git pull
git pull
if [ $? -eq 0 ]; then
    echo "git pull successful"
    # colcon build --symlink-install
    colcon build
    if [ $? -eq 0 ]; then
        echo "colcon build successful"
        # source install/setup.bash
        source install/setup.bash
    else
        echo "colcon build failed"
    fi
else
    echo "git pull failed"
fi