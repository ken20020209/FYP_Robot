FROM ros:foxy

LABEL maintainer="ken20020209 <ken20020209@gmail.com>"

ADD /src /fyp/src

ADD /MutoLib /fyp/MutoLib
ADD startConnector.py /fyp

# WORKDIR /fyp/MutoLib
RUN python3 /fyp/MutoLib/setup.py install && \
    apt-get update && \
    apt-get install -y python3-pip && \ 
    pip3 install pyserial && \
    pip install opencv-python


WORKDIR /fyp
RUN . /opt/ros/foxy/setup.sh && \
    cd /fyp && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build


ENV ROS_DISTRO=foxy
ENV ROS_DOMAIN_ID=16

# CMD [ "source","/install/setup.bash" ]
# CMD [ "ros2","launch","basic","Connector.launch.py" ]

COPY /docker/entrypoint.sh /
ENTRYPOINT [ "sh","/entrypoint.sh" ]
CMD [ "/bin/bash" ]