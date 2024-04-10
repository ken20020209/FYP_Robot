# FYP_Robot

build:docker buildx build --platform linux/arm64 -t ken20020209/fyp_robot:muto_s2 -f docker/dockerfile .

docker push ken20020209/fyp_robot:muto_s2


docker pull ken20020209/fyp_robot:muto_s2

for running

docker run -it --rm --net host --device /dev/myserial:/dev/myserial --device /dev/video0:/dev/video0 -e ROBOT_NAME=muto_s2_1 -e ROBOT_TYPE=muto_s2 ken20020209/fyp_robot:muto_s2

for debugging

docker run -it --rm --net host --device /dev/myserial:/dev/myserial --device /dev/video0:/dev/video0 -e ROBOT_NAME=muto_s2_1 -e ROBOT_TYPE=muto_s2 --entrypoint /bin/bash ken20020209/fyp_robot:muto_s2