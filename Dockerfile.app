FROM ros2-base-image

# set up ROS2 packages
SHELL ["/bin/bash", "-c"]
WORKDIR /ros_workspace

COPY ros_workspace/src/ /ros_workspace/src/

# RUN source /opt/ros/humble/setup.bash && colcon build
# tmp solution
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select my_msgs radar_driver_cpp

# set up enterpoint & environment for ROS2
COPY enterpoint.sh /
RUN chmod +x /enterpoint.sh
ENTRYPOINT [ "/enterpoint.sh" ]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc\
    && echo "source /ros_workspace/install/setup.bash" >> ~/.bashrc

#ARG SUBNET=11
#ENV ROS_DOMAIN_ID=0 \
#    ROS_DISCOVERY_SERVER=192.168.${SUBNET}.1:11811

CMD ["bash", "-c", "tail -f /dev/null"]
