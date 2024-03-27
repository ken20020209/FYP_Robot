# FYP_RobotDog

build:docker build -t fyprobot -f docker/Dockerfile .

run:docker run -it --rm --net=host -e "{name=var}" fyprobot