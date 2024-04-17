# FYP_Robot

build:docker buildx build --platform linux/arm64 -t ken20020209/fyp_robot:go_1 -f docker/dockerfile .
build:docker buildx build --platform linux/arm64 -t ken20020209/fyp_robot:go_1_bridge -f docker/dockerfileBridge .

docker push ken20020209/fyp_robot:go_1
docker push ken20020209/fyp_robot:go_1_bridge

docker pull ken20020209/fyp_robot:go_1
docker pull ken20020209/fyp_robot:go_1_bridge

for running

docker run -d --rm --net host -e ROBOT_NAME=go_1_0 -e ROBOT_TYPE=go_1 ken20020209/fyp_robot:go_1
docker run -d --rm --net host ken20020209/fyp_robot:go_1_bridge

for debugging

docker run -it --rm --net host -e ROBOT_NAME=go_1_0 -e ROBOT_TYPE=go_1 --entrypoint /bin/bash ken20020209/fyp_robot:go_1
