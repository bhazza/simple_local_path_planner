# Only tested with $ROS_VERSION so far, but may work with other versions
ARG ROS_VERSION

# For convenience, use ROS desktop full. Could optimise in future to use minimal ros:$ROS_VERSION install, or even start with Ubuntu 20.04 base image.
FROM ros:$ROS_VERSION

# (ARG reset due to "FROM" above)
ARG ROS_VERSION 

# Install Turtlebot3 prerequisites, as defined in: https://github.com/ROBOTIS-GIT/robotis_tools/blob/master/install_ros_$ROS_VERSION.sh
RUN apt-get update 
RUN apt-get install -y \
  chrony \
  # ntpdate \
  curl \
  build-essential 
# RUN ntpdate ntp.ubuntu.com

RUN DEBIAN_FRONTEND=noninteractive && apt-get install -y --no-install-recommends \
  ros-$ROS_VERSION-rqt-* \
  ros-$ROS_VERSION-gazebo-* \
  ros-$ROS_VERSION-gmapping \
  ros-$ROS_VERSION-dwa-local-planner
RUN apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential git
RUN apt install python3-rosdep
# RUN /bin/bash -c "rosdep init"
RUN rosdep update

# Install Turtlebot3 as per: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
RUN apt-get install -y \
  ros-$ROS_VERSION-dynamixel-sdk \
  ros-$ROS_VERSION-turtlebot3-msgs \
  ros-$ROS_VERSION-turtlebot3 \
  ros-$ROS_VERSION-turtlebot3-simulations

# #OR install from source:
# RUN mkdir -p /catkin_turtlebot_ws/src/
# WORKDIR /catkin_turtlebot_ws/src/
# RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
#     git clone -b $ROS_VERSION-devel https://github.com/ROBOTIS-GIT/turtlebot3.git && \
#     git clone -b $ROS_VERSION-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
# WORKDIR $HOME/catkin_turtlebot_ws/
# RUN /bin/bash -c ". /opt/ros/$ROS_VERSION/setup.bash; catkin_make"

RUN mkdir -p /catkin_ws/src/simple_local_path_planner
ADD . /catkin_ws/src/simple_local_path_planner
WORKDIR /catkin_ws/
RUN /bin/bash -c '. /opt/ros/$ROS_VERSION/setup.bash; catkin_make'

# Install optional useful tools
# RUN apt-get install -y \
#   vim \
#   gdb

# Entrypoint
ADD entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]