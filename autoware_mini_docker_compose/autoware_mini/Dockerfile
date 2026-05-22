FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04

ARG DISPLAY
ARG QT_X11_NO_MITSHM
ARG NVIDIA_DRIVER_CAPABILITIES

ENV DISPLAY=${DISPLAY}
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES}

# Minimal setup
RUN apt-get update && apt-get install -y locales lsb-release git 
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
SHELL ["/bin/bash", "-c"]
RUN source /root/.bashrc

# Install Autoware Mini
RUN mkdir -p /opt/catkin_ws/src
WORKDIR /opt/catkin_ws/src 
RUN git clone https://github.com/UT-ADL/autoware_mini.git
RUN git clone https://github.com/UT-ADL/vehicle_platform.git
RUN git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git carla_ros_bridge
SHELL ["/bin/bash", "-c"]
RUN . /opt/ros/noetic/setup.bash
RUN apt-get install apt-utils -y 
RUN rosdep fix-permissions
RUN rosdep update -y 
ENV ROS_DISTRO=noetic
RUN apt install software-properties-common -y 
RUN add-apt-repository universe -y 
RUN apt update -y 
RUN apt install python2 -y 
RUN apt install python3-pip -y 
RUN pip3 install -r /opt/catkin_ws/src/autoware_mini/requirements.txt
RUN rosdep install --from-paths . --ignore-src -r -y 
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    ros-noetic-ros-numpy \
    ros-noetic-jsk-recognition-msgs \
    ros-noetic-lanelet2 \
    ros-noetic-lanelet2-python \
    ros-noetic-lanelet2-core \
    ros-noetic-lanelet2-io \
    ros-noetic-lanelet2-routing \
    ros-noetic-lanelet2-traffic-rules \
    ros-noetic-lanelet2-projection \
    ros-noetic-ackermann-msgs \
    ros-noetic-derived-object-msgs \
    ros-noetic-jsk-rviz-plugins
RUN pip3 install --upgrade pip
RUN apt-get install wget -y 
WORKDIR /opt/
RUN wget https://files.pythonhosted.org/packages/ab/60/afc0c4486302e04195200079c6297b8075c5b7abc5b977d2db0399d0cb32/carla-0.9.13-cp38-cp38-manylinux_2_27_x86_64.whl
RUN pip3 install carla-0.9.13-cp38-cp38-manylinux_2_27_x86_64.whl 
RUN rm carla-0.9.13-cp38-cp38-manylinux_2_27_x86_64.whl
# multi terminals split  
RUN apt install tmux -y 
ARG NVIDIA_CONTAINER_TOOLKIT_VERSION=1.19.0-1
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2
RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
 && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    > /etc/apt/sources.list.d/nvidia-container-toolkit.list
RUN apt-get update \
 && apt-get install -y \
    nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
    nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
    libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
    libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
WORKDIR /opt/catkin_ws
RUN source /opt/ros/noetic/setup.bash && catkin build
RUN echo "source /opt/catkin_ws/devel/setup.bash" >> /root/.bashrc


