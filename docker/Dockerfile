FROM ros:noetic

LABEL Maintainer="Kota Kondo <kkondo@mit.edu>" \
      Description="Ubuntu 20.04, ROS Noetic for Ego-Swarm"

ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive

RUN rm -rf /var/lib/apt/lists/* && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -q -y --no-install-recommends \
      git apt-utils wget tmux vim tmuxp make \
      python3-pip python3-catkin-tools cmake \
      libarmadillo-dev libglfw3-dev libglew-dev libeigen3-dev libdw-dev \
      libncurses-dev libignition-math4-dev \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs \
      ros-${ROS_DISTRO}-gazebo-ros-control \
      ros-${ROS_DISTRO}-mavros-msgs \
      ros-${ROS_DISTRO}-mavros \
      ros-${ROS_DISTRO}-tf2-sensor-msgs \
      ros-${ROS_DISTRO}-control-toolbox \
      ros-${ROS_DISTRO}-ros-control \
      ros-${ROS_DISTRO}-robot-localization \
      ros-${ROS_DISTRO}-lms1xx \
      ros-${ROS_DISTRO}-interactive-marker-twist-server \
      ros-${ROS_DISTRO}-hector-gazebo-plugins \
      ros-${ROS_DISTRO}-move-base \
      ros-${ROS_DISTRO}-ros-controllers \
      ros-${ROS_DISTRO}-pointgrey-camera-description \
      ros-${ROS_DISTRO}-hardware-interface \
      ros-${ROS_DISTRO}-message-to-tf \
      ros-${ROS_DISTRO}-rqt-gui \
      ros-${ROS_DISTRO}-rqt-gui-py \
      ros-${ROS_DISTRO}-rqt-graph \
      ros-${ROS_DISTRO}-rqt* \
      ros-${ROS_DISTRO}-rviz \
      ros-${ROS_DISTRO}-joy \
      ros-${ROS_DISTRO}-pcl-ros \
      ros-${ROS_DISTRO}-pcl-conversions \
      ros-${ROS_DISTRO}-rosfmt \
      ros-${ROS_DISTRO}-roslint \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install pyquaternion pandas

# Set up Ego-Swarm workspace
RUN mkdir -p /home/kota/ego_swarm_v2_ws/src

# Install dependencies
# Livox-SDK2
WORKDIR /home/kota/
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /home/kota/Livox-SDK2
RUN mkdir build
WORKDIR /home/kota/Livox-SDK2/build
RUN cmake .. && make -j
RUN sudo make install

# Install Ego-Swarm
WORKDIR /home/kota/ego_swarm_v2_ws/src
ARG DUMMY=unknown
RUN DUMMY=${DUMMY} git clone https://github.com/kotakondo/EGO-Planner-v2.git

# acl-gazebo
WORKDIR /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws/src
RUN git clone https://gitlab.com/mit-acl/lab/acl-gazebo.git
WORKDIR /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws/src/acl-gazebo
RUN git checkout ego_swarm_noetic

# snapstack_msgs
WORKDIR /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws/src
RUN git clone https://gitlab.com/mit-acl/fsw/snap-stack/snapstack_msgs.git

# Build the workspace
WORKDIR /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws
RUN bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release"

# livox_ros_driver2
RUN mkdir -p /home/kota/livox_ros_ws/src
WORKDIR /home/kota/livox_ros_ws/src
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git
WORKDIR /home/kota/livox_ros_ws/src/livox_ros_driver2
RUN ./build.sh ROS1

# Mid360_simulation_plugin
RUN mkdir -p /home/kota/mid360_ws/src
WORKDIR /home/kota/mid360_ws/src
RUN git clone https://github.com/fratopa/Mid360_simulation_plugin.git
WORKDIR /home/kota/mid360_ws
RUN bash -c "source /opt/ros/noetic/setup.bash && source /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws/devel/setup.bash && catkin_make -DBUILD_TYPE=Release"

# Set up bashrc
RUN echo "source /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws/devel/setup.bash" >> ~/.bashrc

# Set up ROS
# RUN echo "export ROS_MASTER_URI=http://192.168.0.1:11311" >> ~/.bashrc
# RUN echo "export ROS_MASTER_URI=http://172.17.0.2:11311" >> ~/.bashrc
# RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://127.0.0.1:11311" >> ~/.bashrc
RUN echo "export ROS_IP=localhost" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc

# shortcut 
RUN echo "alias tks='tmux kill-server'" >> ~/.bashrc
RUN echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
RUN echo "alias eb='code ~/.bashrc'" >> ~/.bashrc
RUN echo "alias gs='git status'" >> ~/.bashrc
RUN echo "alias gp='git push'" >> ~/.bashrc
RUN echo "alias roscd='cd /home/kota/ego_swarm_v2_ws'" >> ~/.bashrc
RUN echo "alias cb='roscd && catkin build && sb'" >> ~/.bashrc
RUN echo "alias ss='roscd && source devel/setup.bash'" >> ~/.bashrc
RUN echo "alias cbf='clear && roscd && catkin build && ss'" >> ~/.bashrc

# environment variables
RUN echo "export DISABLE_ROS1_EOL_WARNINGS=1" >> ~/.bashrc

# Go to the ego_swarm_v2_ws
WORKDIR /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws

# Copy tmux session configs
COPY ./run_benchmark.yaml /
COPY ./ego_swarm_v2_sim.yml /
COPY ./ego_swarm_v2_dynamic_sim.yml /
COPY ./playback_bag.yml /
COPY ./single_drone_waypoints_dynamic.launch \
     /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws/src/planner/plan_manage/launch/
COPY ./playback_rviz.rviz /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/docker/
COPY ./record_video.sh /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/docker/

# Default to bash; Makefile controls what runs
CMD ["/bin/bash"]

