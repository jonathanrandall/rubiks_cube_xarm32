FROM ros:noetic

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y git

RUN echo "YEEH"
# Set up the catkin workspace directory
RUN mkdir -p /root/catkin_ws/src && \
    echo "Created directory structure:" && \
    ls -R /root/catkin_ws

RUN apt-get update && apt-get install -y python3 python3-pip && \
    ln -s /usr/bin/python3 /usr/bin/python

RUN pip install redis
# Set the working directory
WORKDIR /root/catkin_ws/src
RUN pwd && ls -la

# Clone the ROS TCP Endpoint and ros2quest repository
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git && \
    echo "Cloned ROS-TCP-Endpoint:" && \
    ls -la && \
    git clone https://github.com/MichaelRazum/quest2ros.git && \
    echo "Cloned quest2ros:" && \
    ls -la /root/catkin_ws/src

# Build the catkin workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make" && \
    echo "Catkin workspace built:" && \
    ls -R /root/catkin_ws

# Set up the ROS environment and run the ros2quest demo script
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=192.168.68.101 tcp_port:=10000 \
      & source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun quest2ros ros2quest.py"]

