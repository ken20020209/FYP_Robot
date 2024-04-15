# FYP_RobotDog

build:docker build -t ken20020209/fyp_robot:TestWithoutRobot -f docker/dockerfile .

run:docker run -it --rm --net=host -e "{name=var}"  ken20020209/fyp_robot:TestWithoutRobot

e.g

docker run -it --rm --net=host -e "name=robot1"  ken20020209/fyp_robot:TestWithoutRobot

docker run -it --rm --net=host -e "name=robot1"  ken20020209/fyp_robot:TestWithoutRobot

docker run -it --rm --net=host -e "name=robot1"  ken20020209/fyp_robot:TestWithoutRobot