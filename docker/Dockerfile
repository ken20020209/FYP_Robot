FROM osrf/ros:foxy-desktop

LABEL maintainer="ken20020209 <ken20020209@gmail.com>"

ADD /src /fyp/src
# ADD /super_client_configuration_file.xml /fyp
ADD startConnector.py /fyp

RUN . /opt/ros/foxy/setup.sh && \
    cd /fyp && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build


ENV ROS_DISTRO=foxy
ENV ROS_DOMAIN_ID=16

# CMD [ "source","/install/setup.bash" ]
# CMD [ "ros2","launch","basic","Connector.launch.py" ]

COPY ./startConnectorWithDiscoveryServery.py /fyp
COPY /docker/entrypoint.sh /
ENTRYPOINT [ "sh","/entrypoint.sh" ]
CMD [ "/bin/bash" ]
